"""
Laptop Processing Node & Web Dashboard for TurtleBot Human Following.

The Pi only talks UDP. This process receives its video, runs the CV stack in a
child process, sends velocity back over UDP, and serves the dashboard:
  /stream/video      newest camera JPEG, forwarded untouched (no re-encode)
  /api/stream        Server-Sent Events: telemetry pushed as each pose result lands
The browser draws the pose and boxes on a canvas over the video.
"""

import json
import math
import time
import os
import queue
import threading
import multiprocessing as mp
from flask import Flask, render_template, Response, jsonify, request

from config import Config
from video_streamer import VideoStreamer
from cv_worker import run_cv
from lidar import LidarMap, sectors as lidar_sectors
from navigation import Planner
import cv2

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
app = Flask(
    __name__,
    template_folder=os.path.join(_ROOT, "templates"),
    static_folder=os.path.join(_ROOT, "static"),
)
HIL_MODE = os.environ.get("HIL", "0") == "1"

streamer = None
jpeg_q = None
out_q = None
cmd_q = None
reply_q = None
cv_proc = None
is_processing = True

latest_telemetry = {
    "connected": False,
    "source_mode": "network",
    "stream_url": Config.get_stream_url(),
    "fps": 0.0,
    "kbps": 0,
    "dropped": 0,
    "frame_age_ms": None,
    "pi": {},
    "target_state": "SEARCHING",
    "target_id": None,
    "detected": False,
    "distance_m": 0.0,
    "dynamic_target_distance": Config.TARGET_DISTANCE,
    "offset_deg": 0.0,
    "offset_x_norm": 0.0,
    "active_gesture": "NONE",
    "target_color": "#00f0ff",
    "people_count": 0,
    "safety_status": "SAFE",
    "safety_alerts": [],
    "events": [],
    "phase2_cmd_vel": {"linear_x": 0.0, "angular_z": 0.0},
    "robot_telemetry": {"mode": "AUTO", "emergency_stop": False, "current_linear": 0.0, "current_angular": 0.0, "target_linear": 0.0, "target_angular": 0.0},
    "last_update": time.time(),
    "overlay": [],
    "pose_fps": 0.0,
    "calibration": 210.0,
    "owner_enrolled": False,
}
telemetry_lock = threading.Lock()
telemetry_cond = threading.Condition(telemetry_lock)
telemetry_version = 0

_TELEM_KEYS = (
    "target_state", "target_id", "detected", "distance_m", "dynamic_target_distance",
    "offset_deg", "offset_x_norm", "active_gesture", "target_color", "owner_enrolled",
    "is_owner_matched", "owner_match_score", "people_count", "safety_status",
    "safety_alerts", "phase2_cmd_vel", "obstacle_distances", "nav_status",
    "robot_telemetry", "events", "overlay", "pose_fps", "calibration", "following_enabled",
    "person", "fusion", "reid",
)

lidar_map = LidarMap()
planner = Planner()
latest_plan = None
lidar_lock = threading.Lock()
lidar_jpeg = None
lidar_cond = threading.Condition(lidar_lock)
lidar_seq = 0


def _bump():
    """Caller holds telemetry_lock."""
    global telemetry_version
    telemetry_version += 1
    telemetry_cond.notify_all()


def _feed_pose():
    """Hand the pose process the newest JPEG only and keep the link stats fresh."""
    last_seq = -1
    last_stats = 0.0
    while is_processing:
        try:
            seq, jpg = streamer.wait_new(last_seq, 0.25)
            if jpg is not None and seq != last_seq:
                last_seq = seq
                try:
                    if jpeg_q.full():
                        try:
                            jpeg_q.get_nowait()
                        except Exception:
                            pass
                    jpeg_q.put_nowait(jpg)
                except Exception:
                    pass
            now = time.time()
            if now - last_stats >= 0.2:
                last_stats = now
                snap = streamer.snapshot()
                with telemetry_cond:
                    latest_telemetry.update(snap)
                    _bump()
        except Exception:
            time.sleep(0.05)


def _plan_loop():
    """Plans a way to the follow goal at 4 Hz and tells the vision process whether the straight line is blocked."""
    global latest_plan
    while is_processing:
        time.sleep(0.25)
        with telemetry_lock:
            person = latest_telemetry.get("person")
            enabled = latest_telemetry.get("following_enabled", True)
        with lidar_lock:
            fresh = lidar_map.pose is not None and time.time() - lidar_map.stamp < 1.0
            if not fresh or not person or not enabled:
                latest_plan = None
                continue
            plan = planner.plan(lidar_map.grid, lidar_map.origin, lidar_map.pose,
                                (person["x"], person["y"]), Config.TARGET_DISTANCE)
            latest_plan = plan
        try:
            cmd_q.put_nowait({"op": "plan", "wait": False, "plan": {
                "blocked": plan["blocked"], "wp_bearing": plan["wp_bearing"], "status": plan["status"]}})
        except Exception:
            pass


def _feed_lidar():
    """Registers each scan into the occupancy grid, passes it to the vision process, and renders the map."""
    global lidar_jpeg, lidar_seq
    last = -1
    last_render = 0.0
    while is_processing:
        count, scan = streamer.wait_scan(last, 0.25)
        now = time.time()
        if scan is not None and count != last:
            last = count
            try:
                cmd_q.put_nowait({"op": "lidar", "wait": False, "bins": scan["bins"]})
            except Exception:
                pass
            with lidar_lock:
                lidar_map.update(scan["bins"], scan["pose"])
        if now - last_render >= 0.1:
            last_render = now
            with telemetry_lock:
                person = latest_telemetry.get("person")
                fusion = latest_telemetry.get("fusion") or {}
            with lidar_lock:
                sec = None
                if lidar_map.bins is not None and now - lidar_map.stamp < 1.0:
                    ex = None
                    if person:
                        ex = (math.radians(person["bearing_deg"]), 0.25, person["range"])
                    sec = lidar_sectors(lidar_map.bins, exclude=ex)
                view = lidar_map.render(person=person, sec=sec, plan=latest_plan)
                loc = None
                if lidar_map.pose is not None:
                    loc = {"x": round(lidar_map.pose[0] - lidar_map.origin[0], 2), "y": round(lidar_map.pose[1] - lidar_map.origin[1], 2),
                           "yaw": round(math.degrees(lidar_map.pose[2]), 1), "score": round(lidar_map.loc_score, 2),
                           "shift": round(lidar_map.loc_shift, 2), "scans": lidar_map.scans,
                           "moving": sum(1 for l in lidar_map.labels if l == "moving")}
            with telemetry_cond:
                latest_telemetry["localization"] = loc
                latest_telemetry["plan_status"] = (latest_plan or {}).get("status", "--")
                _bump()
            ok, buf = cv2.imencode(".jpg", view, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with lidar_cond:
                    lidar_jpeg = buf.tobytes()
                    lidar_seq += 1
                    lidar_cond.notify_all()


def generate_lidar_stream():
    last = -1
    try:
        while True:
            with lidar_cond:
                if lidar_seq == last:
                    lidar_cond.wait(1.0)
                seq, jpg = lidar_seq, lidar_jpeg
            if jpg is None or seq == last:
                continue
            last = seq
            yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpg)).encode("ascii")
                   + b"\r\n\r\n" + jpg + b"\r\n")
    except (GeneratorExit, BrokenPipeError, ConnectionResetError):
        pass


def _read_pose():
    while is_processing:
        try:
            tel = out_q.get(timeout=0.2)
        except queue.Empty:
            continue
        except Exception:
            time.sleep(0.05)
            continue
        if not isinstance(tel, dict):
            continue
        with telemetry_cond:
            for key in _TELEM_KEYS:
                if key in tel:
                    latest_telemetry[key] = tel[key]
            latest_telemetry["last_update"] = time.time()
            _bump()


def _start_cv():
    global cv_proc
    ctx = mp.get_context("spawn")
    cv_proc = ctx.Process(target=run_cv, args=(jpeg_q, out_q, cmd_q, reply_q), daemon=True)
    cv_proc.start()


def _supervise_cv():
    """If the vision process ever dies, restart it. The Pi stops the wheels within 0.6 s meanwhile."""
    while is_processing:
        time.sleep(1.0)
        if cv_proc is not None and not cv_proc.is_alive():
            print(f"[Dashboard] Vision process exited (code {cv_proc.exitcode}); restarting.")
            with telemetry_cond:
                latest_telemetry["status_message"] = "Vision process restarted"
                _bump()
            _start_cv()


def _cv_call(op, wait=False, timeout=1.0, **kwargs):
    cmd_q.put({"op": op, "wait": wait, **kwargs})
    if not wait:
        return {"ok": True, "op": op}
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            msg = reply_q.get(timeout=0.1)
        except queue.Empty:
            continue
        except Exception:
            break
        if isinstance(msg, dict) and msg.get("op") == op:
            return msg
    return {"ok": False, "op": op}


def generate_video_stream():
    """Forwards each new camera JPEG as it arrives. No decode, no re-encode, no polling."""
    last = -1
    try:
        while True:
            seq, jpg = streamer.wait_new(last, 1.0)
            if jpg is None or seq == last:
                continue
            last = seq
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(jpg)).encode("ascii") + b"\r\n\r\n"
                + jpg
                + b"\r\n"
            )
    except (GeneratorExit, BrokenPipeError, ConnectionResetError):
        pass


def generate_telemetry_events():
    """SSE. Pushes on each pose result or link update, at most ~30 per second."""
    seen = -1
    last_sent = 0.0
    try:
        while True:
            with telemetry_cond:
                if telemetry_version == seen:
                    telemetry_cond.wait(1.0)
                seen = telemetry_version
                payload = json.dumps(latest_telemetry)
            wait = 0.033 - (time.time() - last_sent)
            if wait > 0:
                time.sleep(wait)
            last_sent = time.time()
            yield f"data: {payload}\n\n"
    except (GeneratorExit, BrokenPipeError, ConnectionResetError):
        pass


@app.route("/")
def index():
    return render_template(
        "index.html",
        pi_ip=Config.PI_IP,
        stream_url=Config.get_stream_url(),
        target_dist=Config.TARGET_DISTANCE,
        hil=HIL_MODE,
    )


@app.route("/stream/video")
def stream_video():
    return Response(
        generate_video_stream(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store"},
    )


@app.route("/stream/lidar")
def stream_lidar():
    return Response(generate_lidar_stream(), mimetype="multipart/x-mixed-replace; boundary=frame",
                    headers={"Cache-Control": "no-store"})


@app.route("/api/lidar/reset", methods=["POST"])
def api_lidar_reset():
    with lidar_lock:
        lidar_map.reset()
    return jsonify({"status": "reset"})


@app.route("/api/stream")
def api_stream():
    return Response(
        generate_telemetry_events(),
        mimetype="text/event-stream",
        headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"},
    )


@app.route("/api/telemetry")
def api_telemetry():
    with telemetry_lock:
        data = dict(latest_telemetry)
    return jsonify(data)


@app.route("/api/events")
def api_events():
    with telemetry_lock:
        events = list(latest_telemetry.get("events") or [])
    return jsonify({"events": events})


@app.route("/api/unlock_target", methods=["POST"])
def api_unlock_target():
    _cv_call("unlock")
    return jsonify({"status": "unlocked"})


@app.route("/api/enroll_owner", methods=["POST"])
def api_enroll_owner():
    result = _cv_call("enroll", wait=True)
    success = bool(result.get("ok"))
    return jsonify({
        "status": "success" if success else "failed",
        "enrolled": bool(result.get("enrolled")),
        "message": "Owner registered and recognized!" if success else "No person detected in view to enroll"
    })


@app.route("/api/clear_owner", methods=["POST"])
def api_clear_owner():
    _cv_call("release")
    return jsonify({"status": "success", "enrolled": False})


@app.route("/api/release_owner", methods=["POST"])
def api_release_owner():
    result = _cv_call("release", wait=True)
    return jsonify({"status": "success", "state": result.get("state")})


@app.route("/api/start_following", methods=["POST"])
def api_start_following():
    result = _cv_call("start", wait=True)
    return jsonify({"status": "success", "following_enabled": True, "state": result.get("state")})


@app.route("/api/pause_following", methods=["POST"])
def api_pause_following():
    result = _cv_call("pause", wait=True)
    return jsonify({"status": "success", "following_enabled": False, "state": result.get("state")})


@app.route("/api/robot_mode", methods=["POST"])
def api_robot_mode():
    data = request.get_json() or {}
    mode = data.get("mode", "AUTO")
    _cv_call("mode", mode=mode)
    return jsonify({"status": "success", "mode": mode})


@app.route("/api/teleop", methods=["POST"])
def api_teleop():
    data = request.get_json() or {}
    lin = float(data.get("linear", 0.0))
    ang = float(data.get("angular", 0.0))
    _cv_call("teleop", linear=lin, angular=ang)
    return jsonify({"status": "received", "linear": lin, "angular": ang})


@app.route("/api/estop", methods=["POST"])
def api_estop():
    _cv_call("estop")
    return jsonify({"status": "stopped", "emergency_stop": True})


@app.route("/api/test_motors", methods=["POST"])
def api_test_motors():
    """Commands a safe 1.2s spin through the same UDP path as everything else."""
    with telemetry_lock:
        prev_mode = (latest_telemetry.get("robot_telemetry") or {}).get("mode", "AUTO")
    _cv_call("mode", mode="MANUAL")

    def _spin():
        end = time.time() + 1.2
        while time.time() < end:
            _cv_call("teleop", linear=0.0, angular=0.5)
            time.sleep(0.1)
        _cv_call("teleop", linear=0.0, angular=0.0)
        time.sleep(0.1)
        _cv_call("mode", mode=prev_mode)
    threading.Thread(target=_spin, daemon=True).start()

    _cv_call("log", event="MOTOR_TEST", message="Hardware motor spin test initiated (1.2s @ 0.5 rad/s).")
    return jsonify({"status": "testing", "message": "Motor spin test in progress."})


def _config_view():
    with telemetry_lock:
        calibration = latest_telemetry.get("calibration", 210.0)
    return {
        "pi_ip": Config.PI_IP,
        "stream_url": streamer.stream_url,
        "source_mode": streamer.source_mode,
        "calibration": calibration,
        "target_distance": Config.TARGET_DISTANCE,
    }


@app.route("/api/config", methods=["GET", "POST"])
def api_config():
    if request.method == "POST":
        req_data = request.get_json() or {}

        if "pi_ip" in req_data:
            Config.PI_IP = req_data["pi_ip"].strip()
            streamer.update_pi_ip(Config.PI_IP)
            _cv_call("pi_ip", ip=Config.PI_IP)

        if "source_mode" in req_data:
            streamer.set_source_mode(req_data["source_mode"])

        if "calibration" in req_data:
            try:
                _cv_call("calibration", value=float(req_data["calibration"]))
            except ValueError:
                pass

        if "target_distance" in req_data:
            try:
                Config.TARGET_DISTANCE = float(req_data["target_distance"])
                _cv_call("target_distance", value=Config.TARGET_DISTANCE)
            except ValueError:
                pass

        return jsonify({"status": "success", "config": _config_view()})

    return jsonify(_config_view())


def main():
    global streamer, jpeg_q, out_q, cmd_q, reply_q, cv_proc, is_processing
    streamer = VideoStreamer(Config.get_stream_url())
    ctx = mp.get_context("spawn")
    jpeg_q = ctx.Queue(1)
    out_q = ctx.Queue(1)
    cmd_q = ctx.Queue()
    reply_q = ctx.Queue()
    _start_cv()
    threading.Thread(target=_supervise_cv, daemon=True).start()
    threading.Thread(target=_feed_pose, daemon=True).start()
    threading.Thread(target=_feed_lidar, daemon=True).start()
    threading.Thread(target=_plan_loop, daemon=True).start()
    threading.Thread(target=_read_pose, daemon=True).start()

    print("=" * 65)
    print("  HUMAN FOLLOWING ROBOT - COMPUTER VISION ENGINE & DASHBOARD")
    print("=" * 65)
    print(f"Pi (UDP only): {Config.PI_IP}  commands -> :{Config.UDP_PORT}   video <- :{Config.VIDEO_PORT}")
    print(f"Opening Web Dashboard on: http://localhost:{Config.DASHBOARD_PORT}")
    print("=" * 65)
    try:
        app.run(
            host=Config.DASHBOARD_HOST,
            port=Config.DASHBOARD_PORT,
            threaded=True,
            debug=False,
            use_reloader=False
        )
    finally:
        is_processing = False
        try:
            jpeg_q.put_nowait(None)
        except Exception:
            pass
        if streamer is not None:
            streamer.stop()
        if cv_proc is not None:
            cv_proc.join(timeout=1.0)
            if cv_proc.is_alive():
                cv_proc.terminate()


if __name__ == "__main__":
    main()
