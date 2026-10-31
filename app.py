"""
Laptop Processing Node & Dual-Stream Web Dashboard for TurtleBot Human Following.
Integrates full Computer Vision stack:
- Multi-person detection & disambiguation (HSV appearance Re-ID)
- 33-point MediaPipe Pose estimation & dynamic distance tracking
- Lost-human recovery with Kalman filter motion projection
- Natural gesture recognition (FOLLOW_ME, STOP_HALT, RELEASE)
- Visual proximity and obstacle safety monitoring
- Real-time event log streaming
"""

import time
import os
import threading
import urllib.request
import traceback
from flask import Flask, render_template, Response, jsonify, request
import cv2
import numpy as np

from config import Config
from video_streamer import VideoStreamer
from cv_pipeline import CVPipeline

app = Flask(__name__)

# Initialize subsystems
streamer = VideoStreamer(Config.get_stream_url())
cv_engine = CVPipeline(Config.MODEL_PATH)

# Global telemetry cache
latest_telemetry = {
    "connected": False,
    "source_mode": "network",
    "stream_url": Config.get_stream_url(),
    "fps": 0.0,
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
    "last_update": time.time()
}
telemetry_lock = threading.Lock()

# Worker thread cache
processed_frame_cache = None
frame_lock = threading.Lock()
is_processing = True

def processing_worker():
    global processed_frame_cache, latest_telemetry
    fps_counter = 0
    fps_start = time.time()
    current_fps = 0.0

    while is_processing:
        try:
            frame, is_connected, status_msg = streamer.get_latest_frame()
            
            if frame is None:
                time.sleep(0.02)
                continue

            # Run Complete Computer Vision Pipeline
            tel_data, annotated_frame = cv_engine.process_frame(frame)
            
            # Laptop FPS calculation
            fps_counter += 1
            now = time.time()
            if now - fps_start >= 1.0:
                current_fps = round(fps_counter / (now - fps_start), 1)
                fps_counter = 0
                fps_start = now

            # Update telemetry
            with telemetry_lock:
                latest_telemetry.update({
                    "connected": is_connected,
                    "status_message": status_msg,
                    "source_mode": streamer.source_mode,
                    "stream_url": streamer.stream_url,
                    "fps": current_fps,
                    "target_state": tel_data["target_state"],
                    "target_id": tel_data["target_id"],
                    "detected": tel_data["detected"],
                    "distance_m": tel_data["distance_m"],
                    "dynamic_target_distance": tel_data["dynamic_target_distance"],
                    "offset_deg": tel_data["offset_deg"],
                    "offset_x_norm": tel_data["offset_x_norm"],
                    "active_gesture": tel_data["active_gesture"],
                    "target_color": tel_data["target_color"],
                    "owner_enrolled": tel_data.get("owner_enrolled", False),
                    "is_owner_matched": tel_data.get("is_owner_matched", False),
                    "owner_match_score": tel_data.get("owner_match_score", 0.0),
                    "people_count": tel_data["people_count"],
                    "safety_status": tel_data["safety_status"],
                    "safety_alerts": tel_data["safety_alerts"],
                    "events": list(cv_engine.event_log[-12:]), # latest 12 events
                    "phase2_cmd_vel": tel_data["phase2_cmd_vel"],
                    "obstacle_distances": tel_data.get("obstacle_distances", {"left": 3.0, "center": 3.0, "right": 3.0}),
                    "nav_status": tel_data.get("nav_status", "CLEAR"),
                    "robot_telemetry": tel_data.get("robot_telemetry", cv_engine.robot.get_telemetry()),
                    "last_update": now
                })

            # Cache processed frame
            with frame_lock:
                processed_frame_cache = annotated_frame

            time.sleep(0.01)
        except Exception as e:
            print(f"[Worker Exception] {e}")
            traceback.print_exc()
            time.sleep(0.05)

# Start background CV processing thread
proc_thread = threading.Thread(target=processing_worker, daemon=True)
proc_thread.start()


@app.route("/")
def index():
    return render_template(
        "index.html",
        pi_ip=Config.PI_IP,
        stream_url=Config.get_stream_url(),
        target_dist=Config.TARGET_DISTANCE
    )


def generate_raw_stream():
    """Generates untouched MJPEG multipart stream from Raspberry Pi camera"""
    try:
        while True:
            frame, is_connected, _ = streamer.get_latest_frame()
            if frame is None:
                time.sleep(0.04)
                continue

            success, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if not success:
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + buffer.tobytes()
                + b"\r\n"
            )
            time.sleep(0.033)
    except (GeneratorExit, BrokenPipeError, ConnectionResetError):
        pass
    except Exception:
        pass


def generate_processed_stream():
    """Generates AI-processed stream with full skeleton, target reticle, HUD"""
    try:
        while True:
            with frame_lock:
                frame_to_send = processed_frame_cache.copy() if processed_frame_cache is not None else None

            if frame_to_send is None:
                time.sleep(0.03)
                continue

            success, buffer = cv2.imencode(".jpg", frame_to_send, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if not success:
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + buffer.tobytes()
                + b"\r\n"
            )
            time.sleep(0.033)
    except (GeneratorExit, BrokenPipeError, ConnectionResetError):
        pass
    except Exception:
        pass


@app.route("/stream/raw")
def stream_raw():
    return Response(
        generate_raw_stream(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/stream/processed")
def stream_processed():
    return Response(
        generate_processed_stream(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/api/telemetry")
def api_telemetry():
    with telemetry_lock:
        data = dict(latest_telemetry)
    return jsonify(data)


@app.route("/api/events")
def api_events():
    return jsonify({"events": cv_engine.event_log})


@app.route("/api/unlock_target", methods=["POST"])
def api_unlock_target():
    cv_engine.unlock_target()
    return jsonify({"status": "unlocked"})


@app.route("/api/enroll_owner", methods=["POST"])
def api_enroll_owner():
    success = cv_engine.enroll_owner()
    return jsonify({
        "status": "success" if success else "failed",
        "enrolled": cv_engine.owner_enrolled,
        "message": "Owner registered and recognized!" if success else "No person detected in view to enroll"
    })


@app.route("/api/clear_owner", methods=["POST"])
def api_clear_owner():
    cv_engine.release_owner()
    return jsonify({"status": "success", "enrolled": False})


@app.route("/api/release_owner", methods=["POST"])
def api_release_owner():
    cv_engine.release_owner()
    return jsonify({"status": "success", "state": cv_engine.target_state})


@app.route("/api/start_following", methods=["POST"])
def api_start_following():
    cv_engine.start_following()
    return jsonify({"status": "success", "following_enabled": True, "state": cv_engine.target_state})


@app.route("/api/pause_following", methods=["POST"])
def api_pause_following():
    cv_engine.pause_following()
    return jsonify({"status": "success", "following_enabled": False, "state": cv_engine.target_state})


@app.route("/api/robot_mode", methods=["POST"])
def api_robot_mode():
    data = request.get_json() or {}
    mode = data.get("mode", "AUTO")
    cv_engine.robot.set_mode(mode)
    return jsonify({"status": "success", "mode": cv_engine.robot.mode})


@app.route("/api/teleop", methods=["POST"])
def api_teleop():
    data = request.get_json() or {}
    lin = float(data.get("linear", 0.0))
    ang = float(data.get("angular", 0.0))
    cv_engine.robot.update_manual_teleop(lin, ang)
    return jsonify({"status": "received", "linear": lin, "angular": ang})


@app.route("/api/estop", methods=["POST"])
def api_estop():
    cv_engine.robot.trigger_emergency_stop()
    cv_engine._log_event("EMERGENCY_STOP", "Hardware Emergency Stop triggered from Dashboard.")
    return jsonify({"status": "stopped", "emergency_stop": True})


@app.route("/api/test_motors", methods=["POST"])
def api_test_motors():
    """Commands a safe 1.2s spin to verify physical motor operation"""
    try:
        test_url = f"http://{Config.PI_IP}:{Config.PI_STREAM_PORT}/test_motors"
        req = urllib.request.Request(test_url, headers={"User-Agent": "TurtleBot-Dashboard"})
        urllib.request.urlopen(req, timeout=1.5)
    except Exception:
        pass
    prev_mode = cv_engine.robot.mode
    cv_engine.robot.set_mode("MANUAL")
    cv_engine.robot.update_manual_teleop(0.0, 0.5)
    
    def _reset():
        time.sleep(1.2)
        cv_engine.robot.update_manual_teleop(0.0, 0.0)
        time.sleep(0.1)
        cv_engine.robot.set_mode(prev_mode)
    threading.Thread(target=_reset, daemon=True).start()
    
    cv_engine._log_event("MOTOR_TEST", "Hardware motor spin test initiated (1.2s @ 0.5 rad/s).")
    return jsonify({"status": "testing", "message": "Motor spin test in progress."})


@app.route("/api/config", methods=["GET", "POST"])
def api_config():
    if request.method == "POST":
        req_data = request.get_json() or {}
        
        if "pi_ip" in req_data:
            Config.PI_IP = req_data["pi_ip"].strip()
            new_url = f"http://{Config.PI_IP}:{Config.PI_STREAM_PORT}{Config.PI_STREAM_PATH}"
            streamer.update_stream_url(new_url)
            cv_engine.robot.update_url(f"http://{Config.PI_IP}:{Config.PI_STREAM_PORT}/cmd_vel")

        if "source_mode" in req_data:
            streamer.set_source_mode(req_data["source_mode"])

        if "calibration" in req_data:
            try:
                cv_engine.distance_calibration = float(req_data["calibration"])
            except ValueError:
                pass

        if "target_distance" in req_data:
            try:
                Config.TARGET_DISTANCE = float(req_data["target_distance"])
                cv_engine.base_target_distance = Config.TARGET_DISTANCE
            except ValueError:
                pass

        return jsonify({"status": "success", "config": {
            "pi_ip": Config.PI_IP,
            "stream_url": streamer.stream_url,
            "source_mode": streamer.source_mode,
            "calibration": cv_engine.distance_calibration,
            "target_distance": Config.TARGET_DISTANCE
        }})

    return jsonify({
        "pi_ip": Config.PI_IP,
        "stream_url": streamer.stream_url,
        "source_mode": streamer.source_mode,
        "calibration": cv_engine.distance_calibration,
        "target_distance": Config.TARGET_DISTANCE
    })


if __name__ == "__main__":
    print("=" * 65)
    print("  HUMAN FOLLOWING ROBOT - COMPUTER VISION ENGINE & DASHBOARD")
    print("=" * 65)
    print(f"Connecting to Raspberry Pi stream at: {Config.get_stream_url()}")
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
        streamer.stop()
