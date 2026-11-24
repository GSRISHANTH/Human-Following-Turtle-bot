"""Pose process. Kept off the dashboard process so the live picture is not stuck behind MediaPipe."""

import os
import sys
import time
import queue
import traceback


def _apply(engine, cmd):
    from config import Config
    op = cmd.get("op")
    ok = True
    if op == "unlock":
        engine.unlock_target()
    elif op == "enroll":
        ok = bool(engine.enroll_owner())
    elif op == "release":
        engine.release_owner()
    elif op == "start":
        engine.start_following()
    elif op == "pause":
        engine.pause_following()
    elif op == "mode":
        engine.robot.set_mode(cmd.get("mode", "AUTO"))
    elif op == "teleop":
        engine.robot.update_manual_teleop(float(cmd.get("linear", 0.0)), float(cmd.get("angular", 0.0)))
    elif op == "estop":
        engine.robot.trigger_emergency_stop()
        engine._log_event("EMERGENCY_STOP", "Hardware Emergency Stop triggered from Dashboard.")
    elif op == "pi_ip":
        engine.robot.update_pi_ip(cmd.get("ip", ""))
    elif op == "calibration":
        Config.DISTANCE_SCALE = max(0.5, min(2.0, float(cmd["value"])))
        engine.distance_calibration = Config.DISTANCE_SCALE
    elif op == "target_distance":
        engine.base_target_distance = max(Config.MIN_FOLLOW_DISTANCE, min(3.5, float(cmd["value"])))
    elif op == "set":
        # live tuning from the dashboard settings panel; the parent process has validated the names
        for name, value in (cmd.get("values") or {}).items():
            try:
                v = Config.set(name, value)
            except (KeyError, ValueError, TypeError):
                ok = False
                continue
            if name == "DESIRED_DISTANCE":
                engine.base_target_distance = max(Config.MIN_FOLLOW_DISTANCE, min(3.5, v))
            elif name == "MAX_LINEAR_VEL":
                engine.robot.max_linear = v
            elif name == "MAX_ANGULAR_VEL":
                engine.robot.max_angular = v
            elif name == "PI_IP":
                engine.robot.update_pi_ip(v)
            elif name == "DISTANCE_SCALE":
                engine.distance_calibration = v
    elif op == "lidar":
        engine.set_lidar(cmd["bins"])
    elif op == "plan":
        engine.set_plan(cmd["plan"])
    elif op == "log":
        engine._log_event(cmd.get("event", "INFO"), cmd.get("message", ""))
    return {
        "op": op,
        "ok": ok,
        "enrolled": engine.owner_enrolled,
        "state": engine.target_state,
        "mode": engine.robot.mode,
        "following_enabled": engine.following_enabled,
        "calibration": engine.distance_calibration,
    }


def _drain_commands(engine, cmd_q, reply_q):
    while True:
        try:
            cmd = cmd_q.get_nowait()
        except queue.Empty:
            return
        except Exception:
            return
        if not isinstance(cmd, dict):
            continue
        result = _apply(engine, cmd)
        if cmd.get("wait"):
            try:
                reply_q.put_nowait(result)
            except Exception:
                pass


def _publish(out_q, item):
    try:
        while True:
            out_q.get_nowait()
    except Exception:
        pass
    try:
        out_q.put_nowait(item)
    except Exception:
        pass


def run_cv(jpeg_q, out_q, cmd_q, reply_q):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    try:
        import cv2
        import numpy as np
        from cv_pipeline import CVPipeline
        from config import Config
    except ImportError as exc:
        print(f"[Vision] Cannot start: {exc}. This Python ({sys.executable}) is missing a package; "
              "run `python -m pip install -r requirements.txt`.", flush=True)
        sys.exit(3)

    engine = CVPipeline()
    engine.render_hud = False          # the browser draws pose, boxes and the LiDAR overlay itself
    last_frame_at = time.time()
    video_lost = False
    pose_count = 0
    pose_window = time.time()
    pose_fps = 0.0
    last_error = 0.0

    while True:
        _drain_commands(engine, cmd_q, reply_q)
        t_frame = None
        try:
            jpeg = jpeg_q.get(timeout=0.05)
        except queue.Empty:
            # No real camera frame: never keep driving on old perception, and say so on the dashboard.
            if not video_lost and time.time() - last_frame_at > Config.STALE_FRAME_TIMEOUT:
                video_lost = True
                engine.robot.update_auto_commands(0.0, 0.0, can_move=False)
                engine._log_event("VIDEO_LOST", "No camera frames. Robot stopped; it resumes when video returns.")
                _publish(out_q, {"video_lost": True, "events": list(engine.event_log[-12:]),
                                 "robot_telemetry": engine.robot.get_telemetry(), "pose_fps": 0.0,
                                 "phase2_cmd_vel": {"linear_x": 0.0, "angular_z": 0.0}})
            continue
        except Exception:
            continue

        while True:
            try:
                newer = jpeg_q.get_nowait()
            except queue.Empty:
                break
            except Exception:
                break
            if newer is None:
                jpeg = None
                break
            jpeg = newer

        if jpeg is None:
            break
        jpeg, t_frame = jpeg
        last_frame_at = time.time()
        if video_lost:
            video_lost = False
            engine._log_event("VIDEO_RESTORED", "Camera frames are back.")

        _drain_commands(engine, cmd_q, reply_q)
        frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            continue

        try:
            tel, _annotated = engine.process_frame(frame, t_frame)
        except Exception:
            # One bad frame must not kill the process: video, commands and the UI all depend on it.
            engine.robot.update_auto_commands(0.0, 0.0, can_move=False)
            if time.time() - last_error > 5.0:
                last_error = time.time()
                traceback.print_exc()
            continue
        pose_count += 1
        now = time.time()
        if now - pose_window >= 1.0:
            pose_fps = round(pose_count / (now - pose_window), 1)
            pose_count = 0
            pose_window = now

        tel["events"] = list(engine.event_log[-12:])
        tel["calibration"] = engine.distance_calibration
        tel["following_enabled"] = engine.following_enabled
        tel["pose_fps"] = pose_fps
        tel["video_lost"] = False
        tel["settings"] = Config.tunables()
        _publish(out_q, tel)
        _drain_commands(engine, cmd_q, reply_q)
