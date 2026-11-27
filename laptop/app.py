"""
Laptop Processing Node & Web Dashboard for TurtleBot Human Following.

The Pi only talks UDP. This process receives its video, runs the CV stack in a
child process, sends velocity back over UDP, and serves the dashboard:
  /stream/video      newest camera JPEG, forwarded untouched (no re-encode)
  /api/stream        Server-Sent Events: telemetry pushed as each pose result lands
The browser draws the pose and boxes on a canvas over the video.
"""

import ipaddress
import json
import math
import time
import os
import queue
import threading
import multiprocessing as mp
import traceback
from flask import Flask, render_template, Response, jsonify, request

from config import Config
from video_streamer import VideoStreamer
from cv_worker import run_cv
from lidar import LidarMap, sectors as lidar_sectors, orient_bins as lidar_orient
from navigation import Planner, PersonMemory, plan_with_memory
import cv2
import numpy as np

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
    "target_state": Config.STATE_IDLE,
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
    "calibration": Config.DISTANCE_SCALE,
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
    "person", "fusion", "reid", "lidar_cam", "camera", "control", "safety_latched", "ambiguous",
    "video_lost", "settings", "mirror_suspect", "hand_raised", "hold_to_follow", "sensor_fault",
    "tracking_confidence", "follow_state",
)

lidar_map = LidarMap()
planner = Planner()
person_memory = PersonMemory()
latest_plan = None
lidar_lock = threading.Lock()
lidar_jpeg = None
lidar_cond = threading.Condition(lidar_lock)
lidar_seq = 0
lidar_view_clients = 0          # the OpenCV map is only drawn while someone watches /stream/lidar
scene_cond = threading.Condition()
scene_json = None               # RViz-style scene for the browser, 10 Hz
scene_seq = 0
map_png = (None, -1)            # (bytes, scans) cache for /api/map.png


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
            if seq == last_seq:
                jpg = None                      # timed out: nothing new, never re-send the same frame
            elif jpg is not None:
                last_seq = seq
                if not streamer.camera_frame_ok():
                    # offline pattern or the Pi's "no camera" picture: perception must not act on it
                    jpg = None
            if jpg is not None:
                t_rx = streamer.last_frame_time
                try:
                    if jpeg_q.full():
                        try:
                            jpeg_q.get_nowait()
                        except Exception:
                            pass
                    jpeg_q.put_nowait((jpg, t_rx))
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
            enrolled = latest_telemetry.get("owner_enrolled", False)
        now = time.time()
        with lidar_lock:
            fresh = lidar_map.pose is not None and now - lidar_map.stamp < 1.0
            if not enrolled:
                person_memory.clear()
            if not fresh or not enabled or not enrolled:
                latest_plan = None
                continue
            plan = plan_with_memory(planner, lidar_map.grid, lidar_map.origin, lidar_map.pose, person,
                                    person_memory, now, Config.TARGET_DISTANCE, recall_desired=Config.RECALL_APPROACH_M)
            latest_plan = plan
        if plan is None:
            continue
        try:
            cmd_q.put_nowait({"op": "plan", "wait": False, "plan": {
                k: plan.get(k) for k in ("blocked", "wp_bearing", "status", "memory", "mem_bearing", "mem_range", "goal_dist", "age")}})
        except Exception:
            pass


def _feed_lidar():
    """Registers each scan into the occupancy grid, passes it to the vision process, and renders the map."""
    global lidar_jpeg, lidar_seq
    last = -1
    last_render = 0.0
    last_error = 0.0
    while is_processing:
        try:
            last, last_render = _lidar_step(last, last_render)
        except Exception:
            # Never let the map thread die: a dead thread leaves the dashboard map blank for the rest of the session.
            if time.time() - last_error > 5.0:
                last_error = time.time()
                traceback.print_exc()
            with lidar_lock:
                lidar_map.reset()
            time.sleep(0.2)


def _lidar_step(last, last_render):
    global lidar_jpeg, lidar_seq
    if True:
        count, scan = streamer.wait_scan(last, 0.25)
        now = time.time()
        if scan is not None and count != last:
            last = count
            if Config.LIDAR_MIRROR or Config.LIDAR_YAW_DEG:
                scan = dict(scan, bins=lidar_orient(scan["bins"], Config.LIDAR_MIRROR, Config.LIDAR_YAW_DEG))
            try:
                cmd_q.put_nowait({"op": "lidar", "wait": False, "bins": scan["bins"]})
            except Exception:
                pass
            with telemetry_lock:
                person = latest_telemetry.get("person")
            with lidar_lock:
                lidar_map.update(scan["bins"], scan["pose"], person=(person["x"], person["y"]) if person else None,
                                 pose_src=scan.get("pose_src", "odom"))
        if now - last_render >= 0.1:
            last_render = now
            with telemetry_lock:
                person = latest_telemetry.get("person")
                fusion = latest_telemetry.get("fusion") or {}
                camera = latest_telemetry.get("camera") or {}
            with lidar_lock:
                sec = None
                if lidar_map.bins is not None and now - lidar_map.stamp < 1.0:
                    ex = None
                    if person:
                        ex = (math.radians(person["bearing_deg"]), 0.25, person["range"])
                    sec = lidar_sectors(lidar_map.bins, exclude=ex)
                view = None
                if lidar_view_clients > 0:
                    view = lidar_map.render(person=person, sec=sec, plan=latest_plan, hfov_deg=Config.CAMERA_HFOV_DEG,
                                            cam_yaw_deg=camera.get("yaw_deg", 0.0))
                scene = _build_scene(person, camera)
                loc = None
                if lidar_map.pose is not None:
                    loc = {"x": round(lidar_map.pose[0] - lidar_map.origin[0], 2), "y": round(lidar_map.pose[1] - lidar_map.origin[1], 2),
                           "yaw": round(math.degrees(lidar_map.pose[2]), 1), "score": round(lidar_map.loc_score, 2),
                           "shift": round(lidar_map.loc_shift, 2), "scans": lidar_map.scans,
                           "moving": sum(1 for l in lidar_map.labels if l == "moving"),
                           "source": lidar_map.source, "ok": lidar_map.loc_ok, "recenters": lidar_map.recenters,
                           "resets": lidar_map.resets}
            with telemetry_cond:
                latest_telemetry["localization"] = loc
                latest_telemetry["clearance"] = sec
                lp = latest_plan or {}
                latest_telemetry["plan_status"] = (("MEMORY " if lp.get("memory") else "") + lp.get("status", "--")) if lp else "--"
                latest_telemetry["memory_age"] = round(lp["age"], 1) if lp.get("memory") else None
                _bump()
            _publish_scene(scene)
            ok, buf = cv2.imencode(".jpg", view, [cv2.IMWRITE_JPEG_QUALITY, 80]) if view is not None else (False, None)
            if ok:
                with lidar_cond:
                    lidar_jpeg = buf.tobytes()
                    lidar_seq += 1
                    lidar_cond.notify_all()
    return last, last_render


def _build_scene(person, camera):
    """Everything the RViz-style view draws, in the map frame (metres). Caller holds lidar_lock."""
    m = lidar_map
    if m.pose is None or m.origin is None:
        return {"ok": False, "stamp": time.time(), "hz": round(m.hz, 1)}
    px, py, yaw = m.pose
    c, s_ = math.cos(yaw), math.sin(yaw)

    def to_map(x, y):
        return (round(px + c * x - s_ * y, 3), round(py + s_ * x + c * y, 3))

    pts = []
    if m.bins is not None:
        ang = (np.arange(len(m.bins)) + 0.5) * (2 * math.pi / len(m.bins))
        kind = np.zeros(len(m.bins), dtype=np.int8)          # 0 static, 1 wall, 2 moving, 3 person
        for cl, label in zip(m.clusters, m.labels):
            k = {"wall": 1, "moving": 2}.get(label, 0)
            if person and math.hypot(cl["cx"] - person["x"], cl["cy"] - person["y"]) < 0.6:
                k = 3
            kind[cl["idx"]] = k
        for i in np.nonzero(np.isfinite(m.bins))[0]:
            r = float(m.bins[i])
            pts.append([*to_map(r * math.cos(ang[i]), r * math.sin(ang[i])), int(kind[i])])
    pers = None
    if person:
        cx, cy = to_map(person["x"], person["y"])
        vx, vy = c * person["vx"] - s_ * person["vy"], s_ * person["vx"] + c * person["vy"]
        cov = person.get("cov")
        if cov:   # rotate the robot-frame covariance into the map frame: R P R^T
            P = np.array([[cov[0], cov[1]], [cov[1], cov[2]]])
            R = np.array([[c, -s_], [s_, c]])
            Pm = R @ P @ R.T
            cov = [float(Pm[0, 0]), float(Pm[0, 1]), float(Pm[1, 1])]
        pers = {"x": cx, "y": cy, "vx": round(vx, 3), "vy": round(vy, 3), "cov": cov,
                "range": round(person.get("range", 0.0), 2)}
    lp = latest_plan or {}
    return {
        "ok": True, "stamp": time.time(), "hz": round(m.hz, 1), "frame": "map",
        "pose": [round(px, 3), round(py, 3), round(yaw, 4)],
        "map": {"origin": [m.origin[0], m.origin[1]], "res": m.RES, "n": m.N, "scans": m.scans},
        "source": m.source, "score": round(m.loc_score, 2), "loc_ok": m.loc_ok,
        "scan": pts, "person": pers,
        "trail": [[round(x, 2), round(y, 2)] for x, y in m.trail[-300:]],
        "path": [[round(x, 2), round(y, 2)] for x, y in (lp.get("path") or [])],
        "goal": lp.get("goal"), "plan_status": lp.get("status"), "memory": bool(lp.get("memory")),
        "last_seen": lp.get("mem_world"),
        "fov": {"hfov_deg": Config.CAMERA_HFOV_DEG, "yaw_deg": camera.get("yaw_deg", 0.0)},
    }


def _publish_scene(scene):
    global scene_json, scene_seq
    payload = json.dumps(scene)
    with scene_cond:
        scene_json = payload
        scene_seq += 1
        scene_cond.notify_all()


def generate_scene_events():
    last = -1
    try:
        while True:
            with scene_cond:
                if scene_seq == last:
                    scene_cond.wait(1.0)
                seq, payload = scene_seq, scene_json
            if payload is None or seq == last:
                continue
            last = seq
            yield "data: " + payload + "\n\n"
    except (GeneratorExit, BrokenPipeError, ConnectionResetError):
        pass


def _map_png():
    """Occupancy grid as an RGBA PNG in RViz 'map' colours: free light grey, occupied black, unknown transparent.
    Row 0 is the +y edge of the grid (image top = north), column 0 the -x edge."""
    global map_png
    with lidar_lock:
        if map_png[0] is not None and lidar_map.scans - map_png[1] < 5 and lidar_map.scans >= map_png[1]:
            return map_png[0]
        g = lidar_map.grid.copy()
        scans = lidar_map.scans
        origin = lidar_map.origin or (0.0, 0.0)
    img = np.zeros((g.shape[0], g.shape[1], 4), dtype=np.uint8)
    img[g < -0.3] = (205, 205, 205, 235)       # BGRA for cv2
    img[g > 0.9] = (0, 0, 0, 255)
    ok, buf = cv2.imencode(".png", img)
    data = (buf.tobytes() if ok else b"", origin, lidar_map.RES, g.shape[0])
    map_png = (data, scans)
    return data


def generate_lidar_stream():
    global lidar_view_clients
    last = -1
    lidar_view_clients += 1
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
    finally:
        lidar_view_clients -= 1


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
    started = time.time()
    quick_failures = 0
    while is_processing:
        time.sleep(1.0)
        if cv_proc is not None and not cv_proc.is_alive():
            lived = time.time() - started
            quick_failures = quick_failures + 1 if lived < 15.0 else 0
            if quick_failures >= 3:
                msg = (f"Vision process keeps failing at startup (exit code {cv_proc.exitcode}). "
                       "See the [Vision] message above. Fix the install, then restart the dashboard.")
                print(f"[Dashboard] {msg}")
                with telemetry_cond:
                    latest_telemetry["status_message"] = msg
                    _bump()
                time.sleep(30.0)
                quick_failures = 2          # try again, but at most once every 30 s
            else:
                print(f"[Dashboard] Vision process exited (code {cv_proc.exitcode}); restarting.")
                with telemetry_cond:
                    latest_telemetry["status_message"] = "Vision process restarted"
                    _bump()
            started = time.time()
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


@app.route("/api/scene")
def api_scene():
    return Response(generate_scene_events(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})


@app.route("/api/map.png")
def api_map_png():
    data, origin, res, n = _map_png()
    # the origin the PNG was drawn at travels with it, so a grid recentre can never misplace the picture
    return Response(data, mimetype="image/png", headers={
        "Cache-Control": "no-store", "X-Map-Origin": f"{origin[0]:.4f},{origin[1]:.4f}",
        "X-Map-Res": str(res), "X-Map-N": str(n), "Access-Control-Expose-Headers": "X-Map-Origin, X-Map-Res, X-Map-N"})


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
    data = request.get_json(silent=True) or {}
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
    return {
        "pi_ip": Config.PI_IP,
        "stream_url": streamer.stream_url,
        "source_mode": streamer.source_mode,
        "calibration": Config.DISTANCE_SCALE,
        "target_distance": Config.TARGET_DISTANCE,
        "settings": Config.tunables(),
    }


def _apply_settings(values):
    """Validate in this process first (types, ranges, IP), then hand the accepted ones to the vision process."""
    accepted, rejected = {}, {}
    limits = {"DESIRED_DISTANCE": (Config.MIN_FOLLOW_DISTANCE, 3.5), "DISTANCE_SCALE": (0.5, 2.0),
              "CAMERA_HFOV_DEG": (30.0, 160.0), "CAM_HEIGHT_M": (0.0, 1.5), "LIDAR_HEIGHT_M": (0.0, 1.0),
              "CAM_X_M": (-0.5, 0.5), "CAM_Y_M": (-0.5, 0.5), "CAM_YAW_DEG": (-45.0, 45.0), "CAM_PITCH_DEG": (-45.0, 45.0),
              "MAX_LINEAR_VEL": (0.0, 0.26), "MAX_ANGULAR_VEL": (0.0, 2.84), "LIDAR_YAW_DEG": (-180.0, 180.0),
              "MAX_ACCEL_ANGULAR": (1.0, 20.0)}
    for name, raw in (values or {}).items():
        name = str(name).upper()
        if name not in Config.TUNABLE:
            rejected[name] = "not tunable"
            continue
        try:
            v = Config.coerce(name, raw)
            if name in limits:
                lo, hi = limits[name]
                if not lo <= v <= hi:
                    raise ValueError(f"must be in [{lo}, {hi}]")
            if name.startswith("PID_") and not 0.0 <= v <= 10.0:
                raise ValueError("must be in [0, 10]")
            if name == "PI_IP":
                ipaddress.ip_address(v)
        except (ValueError, TypeError, KeyError) as e:
            rejected[name] = str(e)
            continue
        Config.set(name, v)
        accepted[name] = v
    if "PI_IP" in accepted:
        streamer.update_pi_ip(Config.PI_IP)
    if accepted:
        _cv_call("set", values=accepted)
    return accepted, rejected


@app.route("/api/config", methods=["GET", "POST"])
def api_config():
    if request.method == "POST":
        req_data = request.get_json(silent=True) or {}
        values = dict(req_data.get("settings") or {})
        # older dashboard keys
        if "pi_ip" in req_data:
            values["PI_IP"] = req_data["pi_ip"]
        if "calibration" in req_data:
            values["DISTANCE_SCALE"] = req_data["calibration"]
        if "target_distance" in req_data:
            values["DESIRED_DISTANCE"] = req_data["target_distance"]
        if "source_mode" in req_data and req_data["source_mode"] in ("network", "sim"):
            streamer.set_source_mode(req_data["source_mode"])
        accepted, rejected = _apply_settings(values)
        saved = False
        if req_data.get("save") and accepted:
            try:
                Config.save_local(accepted)
                saved = True
            except OSError as e:
                rejected["_save"] = str(e)
        status = "success" if not rejected else ("partial" if accepted else "error")
        return jsonify({"status": status, "accepted": accepted, "rejected": rejected, "saved": saved,
                        "config": _config_view()}), (200 if accepted or not rejected else 400)

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
