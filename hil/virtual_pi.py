#!/usr/bin/env python3
"""Virtual Raspberry Pi for hardware-in-the-loop.

Speaks the same UDP protocol as pi/pi_turtlebot_node.py (see pi/tb_proto.py), with no
OpenCR and no ROS:

    UDP :8888   in   V:<m/s>,W:<rad/s>,S:<seq>   and   HELLO
    UDP         out  webcam JPEG datagrams + status, to whoever sent HELLO

The camera is this laptop's webcam. Velocity packets integrate hil/plant.py instead of
spinning the wheels, so the LiPo can stay on the charger. The pose map is a small HTTP
page on :5001, a laptop-only bench aid that has no counterpart on the real Pi.
"""

import multiprocessing as mp
import os
import socket
import sys
import threading
import time
from pathlib import Path

# Allow `python hil/virtual_pi.py` as well as `python -m hil.virtual_pi`.
_ROOT = Path(__file__).resolve().parents[1]
for _p in (_ROOT, _ROOT / "pi"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import cv2
import numpy as np
from flask import Flask, Response, jsonify

import tb_proto
from hil.camera_worker import camera_process
from hil.map_view import render_map
from hil.plant import BurgerPlant

MAP_PORT = 5001
FRAME_W = 640
FRAME_H = 480
TARGET_FPS = 30
JPEG_QUALITY = 55
PEER_TIMEOUT = 3.0

app = Flask(__name__)
plant = BurgerPlant()
_state_lock = threading.Lock()
_last_packet = ""
_last_seq = None
_camera_ok = False
_latest_frame = None
_frame_no = 0
_peer = None
_peer_time = 0.0
_cmd_count = 0
_send_fps = 0.0
_stop = False
_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def _plant_loop():
    dt = 0.02  # 50 Hz
    while not _stop:
        start = time.time()
        plant.step(dt)
        remain = dt - (time.time() - start)
        if remain > 0:
            time.sleep(remain)


def _udp_loop():
    global _last_packet, _last_seq, _peer, _peer_time, _cmd_count
    _sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    _sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
    try:
        _sock.ioctl(socket.SIO_UDP_CONNRESET, False)
    except (AttributeError, OSError, ValueError):
        pass
    try:
        _sock.bind(("0.0.0.0", tb_proto.CMD_PORT))
    except OSError as exc:
        print(f"[HIL] UDP port {tb_proto.CMD_PORT} is already in use: {exc}")
        os._exit(1)
    print(f"[HIL] UDP cmd_vel listening on {tb_proto.CMD_PORT}", flush=True)
    while not _stop:
        try:
            data, addr = _sock.recvfrom(512)
        except OSError:
            continue
        if data.startswith(b"HELLO"):
            with _state_lock:
                _peer, _peer_time = addr, time.monotonic()
            continue
        parsed = tb_proto.parse_command(data)
        if parsed is None:
            continue
        lin, ang, seq = parsed
        with _state_lock:
            if not tb_proto.seq_is_newer(seq, _last_seq) and time.time() - plant._last_cmd_time < 0.5:
                continue
            if seq is not None:
                _last_seq = seq
            _last_packet = data.decode("ascii", errors="ignore").strip()
            _cmd_count += 1
        plant.set_body_velocity(lin, ang)


def _placeholder(message):
    frame = np.full((FRAME_H, FRAME_W, 3), (28, 32, 38), dtype=np.uint8)
    cv2.putText(frame, "HIL camera", (24, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (220, 230, 235), 2, cv2.LINE_AA)
    cv2.putText(frame, message, (24, 140),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 200, 180), 1, cv2.LINE_AA)
    cv2.putText(frame, "MANUAL teleop on :8000 still moves the map.", (24, 190),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 200, 180), 1, cv2.LINE_AA)
    return frame


def _camera_loop():
    global _camera_ok, _latest_frame, _frame_no
    print("[HIL] Opening laptop webcam...", flush=True)
    ctx = mp.get_context("spawn")
    frame_queue = ctx.Queue(maxsize=2)
    proc = ctx.Process(target=camera_process, args=(frame_queue,), daemon=True)
    proc.start()
    try:
        first = frame_queue.get(timeout=8.0)
    except Exception:
        first = None
    if not isinstance(first, dict) or first.get("status") != "ready":
        if proc.is_alive():
            proc.terminate()
        _camera_ok = False
        print("[HIL] No webcam in time. Map and teleop still work.", flush=True)
        proc = None
    else:
        _camera_ok = True
        print(f"[HIL] Laptop webcam opened ({first.get('backend')}).", flush=True)

    while not _stop:
        frame = None
        if proc is not None and proc.is_alive():
            try:
                frame = frame_queue.get(timeout=0.05)
                while True:
                    frame = frame_queue.get_nowait()
            except Exception:
                pass
        if not isinstance(frame, np.ndarray):
            with _state_lock:
                if _latest_frame is None or not _camera_ok:
                    _latest_frame = _placeholder("Waiting for the laptop webcam.")
                    _frame_no += 1
            time.sleep(0.1)
            continue
        with _state_lock:
            _latest_frame = frame
            _frame_no += 1

    if proc is not None and proc.is_alive():
        proc.terminate()


def _sender_loop():
    """Encode the newest webcam frame and send it to whoever said HELLO."""
    global _send_fps
    params = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
    last_no = -1
    fid = 0
    sent, t0 = 0, time.monotonic()
    last_status = 0.0
    gap = 1.0 / TARGET_FPS
    last_send = 0.0
    while not _stop:
        time.sleep(0.004)
        now = time.monotonic()
        with _state_lock:
            dest = _peer if (_peer is not None and now - _peer_time < PEER_TIMEOUT) else None
            frame_no = _frame_no
            frame = _latest_frame
        if dest is None:
            continue
        if now - last_status >= 0.5:
            last_status = now
            snap = plant.snapshot()
            status = {"ros": "HIL", "subs": 1, "serial": False, "motors": False, "camera": _camera_ok,
                      "cam_fps": TARGET_FPS, "send_fps": round(_send_fps, 1), "raw": False,
                      "cmd_age": None, "cmds": _cmd_count,
                      "lin": snap["linear"], "ang": snap["angular"]}
            try:
                _sock.sendto(tb_proto.pack_status(status, time.time()), dest)
            except OSError:
                pass
        if frame is None or frame_no == last_no or now - last_send < gap:
            continue
        ok, buf = cv2.imencode(".jpg", frame, params)
        if not ok:
            continue
        last_no = frame_no
        fid += 1
        try:
            for pkt in tb_proto.pack_frame(buf.tobytes(), fid, time.time()):
                _sock.sendto(pkt, dest)
        except OSError:
            continue
        last_send = now
        sent += 1
        if now - t0 >= 1.0:
            _send_fps = sent / (now - t0)
            sent, t0 = 0, now


def _mjpeg_map():
    params = [cv2.IMWRITE_JPEG_QUALITY, 70]
    while not _stop:
        ok, buf = cv2.imencode(".jpg", render_map(plant.snapshot()), params)
        if ok:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + buf.tobytes()
                + b"\r\n"
            )
        time.sleep(0.05)


@app.route("/")
def index():
    return """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>TurtleBot HIL</title>
<style>
  body { margin:0; background:#12151a; color:#d7e2ea; font-family:Segoe UI,sans-serif; }
  header { padding:16px 22px; border-bottom:1px solid #243041; }
  h1 { margin:0; font-size:20px; }
  p { margin:6px 0 0; color:#8ea0b0; }
  .grid { display:grid; grid-template-columns: 1.1fr 0.9fr; gap:16px; padding:16px; }
  img { width:100%; background:#000; border-radius:8px; }
  .card { background:#1a2029; border-radius:10px; padding:12px; }
  a { color:#7dffe0; }
  #pose { font-family:Consolas,monospace; white-space:pre; line-height:1.5; }
</style></head>
<body>
<header>
  <h1>Hardware-in-the-loop &mdash; TurtleBot3 Burger</h1>
  <p>Motors are not powered. Charge the battery. Vision dashboard is
     <a href="http://127.0.0.1:8000/">http://127.0.0.1:8000/</a>.</p>
</header>
<div class="grid">
  <div class="card"><h2>Kinematic map</h2><img src="/map_feed" alt="plant map"></div>
  <div class="card">
    <h2>Pose</h2>
    <div id="pose">waiting</div>
  </div>
</div>
<script>
async function tick() {
  try {
    const r = await fetch('/odom');
    const o = await r.json();
    document.getElementById('pose').textContent =
      'x      ' + o.x.toFixed(3) + ' m\\n' +
      'y      ' + o.y.toFixed(3) + ' m\\n' +
      'yaw    ' + (o.theta * 180 / Math.PI).toFixed(1) + ' deg\\n' +
      'v      ' + o.linear.toFixed(3) + ' m/s\\n' +
      'w      ' + o.angular.toFixed(3) + ' rad/s\\n' +
      'camera ' + o.camera;
  } catch (e) {}
}
setInterval(tick, 200);
tick();
</script>
</body></html>"""


@app.route("/map_feed")
def map_feed():
    return Response(_mjpeg_map(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/odom")
def odom():
    snap = plant.snapshot()
    snap.pop("trail", None)
    snap["camera"] = "webcam" if _camera_ok else "placeholder"
    snap["model"] = "turtlebot3_burger"
    snap["wheel_separation_m"] = plant.wheel_separation
    snap["wheel_radius_m"] = plant.wheel_radius
    return jsonify(snap)


@app.route("/reset", methods=["POST"])
def reset():
    plant.reset()
    return jsonify({"status": "reset"})


@app.route("/health")
def health():
    snap = plant.snapshot()
    with _state_lock:
        peer = _peer
        last = _last_packet
    return jsonify({
        "status": "hil",
        "camera": _camera_ok,
        "udp_port": tb_proto.CMD_PORT,
        "peer": f"{peer[0]}:{peer[1]}" if peer else None,
        "send_fps": round(_send_fps, 1),
        "cmds": _cmd_count,
        "linear": snap["linear"],
        "angular": snap["angular"],
        "last_packet": last,
    })


def main():
    threading.Thread(target=_plant_loop, daemon=True).start()
    threading.Thread(target=_udp_loop, daemon=True).start()
    threading.Thread(target=_camera_loop, daemon=True).start()
    threading.Thread(target=_sender_loop, daemon=True).start()
    print("[HIL] Virtual Pi: UDP %d, pose map http://127.0.0.1:%d/  (no motors)" % (tb_proto.CMD_PORT, MAP_PORT), flush=True)
    app.run(host="0.0.0.0", port=MAP_PORT, threaded=True, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
