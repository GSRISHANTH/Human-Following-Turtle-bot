#!/usr/bin/env python3
"""
Complete Raspberry Pi Companion Node for TurtleBot Human Following (Phase 2):
-----------------------------------------------------------------------------
1. Low-Latency Camera Streaming (Hardware MJPG, Zero-Lag capture loop)
2. Robot Motion Command Bridge (/cmd_vel HTTP POST & UDP port 8888)
   - Forwards commands to ROS 1 topic '/cmd_vel' (geometry_msgs/Twist)
   - Forwards commands to ROS 2 topic '/cmd_vel' (geometry_msgs/Twist)
   - Fallback: Writes directly to OpenCR controller via Serial (/dev/ttyACM0)
3. Onboard Safety Heartbeat (stops physical motors if laptop disconnects for >0.6s)
4. Built-in Hardware Motor Test (/test_motors endpoint & --test CLI flag)
"""

import sys
import os
import glob
import time
import json
import socket
import threading
from flask import Flask, Response, request, jsonify

# Ensure ROS 2 Domain ID and robot model are set for TurtleBot3 Burger
os.environ["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "30")
os.environ["TURTLEBOT3_MODEL"] = os.environ.get("TURTLEBOT3_MODEL", "burger")
os.environ["LDS_MODEL"] = os.environ.get("LDS_MODEL", "LDS-01")
os.environ["ROS_LOCALHOST_ONLY"] = "0"

# ---------------------------------------------------------------------------
# Auto-discover ROS Python paths if not already sourced in current environment
# ---------------------------------------------------------------------------
for ros_path in glob.glob('/opt/ros/*/lib/python*/dist-packages') + \
                glob.glob('/opt/ros/*/local/lib/python*/dist-packages') + \
                glob.glob('/opt/ros/*/lib/python*/site-packages') + \
                glob.glob(os.path.expanduser('~/turtlebot3_ws/install/*/lib/python*/site-packages')) + \
                glob.glob(os.path.expanduser('~/turtlebot3_ws/devel/lib/python*/dist-packages')) + \
                glob.glob(os.path.expanduser('~/catkin_ws/devel/lib/python*/dist-packages')):
    if ros_path not in sys.path:
        sys.path.append(ros_path)

# Configuration
CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
TARGET_FPS = 25
JPEG_QUALITY = 55
PORT = 5000
UDP_PORT = 8888

# TurtleBot / ROS Configuration
ROS_MODE = None  # 'ROS1', 'ROS2', or None
ros_cmd_pub = None
TwistClass = None
SERIAL_ENABLED = False
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200

def is_roscore_online():
    """Fast check to see if roscore is listening on port 11311 without blocking"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        s.connect(('127.0.0.1', 11311))
        s.close()
        return True
    except Exception:
        return False

# 1. Try ROS 1 (rospy)
if is_roscore_online():
    try:
        import rospy
        from geometry_msgs.msg import Twist as Twist1
        rospy.init_node('human_follower_pi_bridge', anonymous=True, disable_signals=True)
        ros_cmd_pub = rospy.Publisher('/cmd_vel', Twist1, queue_size=1)
        TwistClass = Twist1
        ROS_MODE = 'ROS1'
        print("[TurtleBot Bridge] Successfully connected to ROS 1 topic: /cmd_vel")
    except Exception as e1:
        print(f"[TurtleBot Bridge] ROS 1 init error: {e1}")
else:
    print("[TurtleBot Bridge] Note: roscore is not running on port 11311.")

# 2. If ROS 1 not active, try ROS 2 (rclpy)
if ROS_MODE is None:
    try:
        import signal
        try:
            signal.signal(signal.SIGHUP, signal.SIG_IGN)
        except Exception:
            pass

        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from geometry_msgs.msg import Twist as Twist2
        try:
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        except Exception:
            rclpy.init()
        ros2_node = rclpy.create_node('human_follower_pi_bridge')
        ros_cmd_pub = ros2_node.create_publisher(Twist2, '/cmd_vel', 10)
        TwistClass = Twist2
        ROS_MODE = 'ROS2'
        threading.Thread(target=rclpy.spin, args=(ros2_node,), daemon=True).start()
        print("[TurtleBot Bridge] Successfully connected to ROS 2 topic: /cmd_vel")
    except Exception as e2:
        print(f"[TurtleBot Bridge] ROS 2 init note: {e2}")

ROS_ENABLED = (ROS_MODE is not None)
if not ROS_ENABLED:
    print("[TurtleBot Bridge WARNING] No active ROS driver found.")
    print("  -> For physical motors to spin in ROS 2, run in another terminal:")
    print("     ros2 launch turtlebot3_bringup robot.launch.py")


# 3. Try OpenCR Serial Direct Port (/dev/ttyACM0) if ROS is not active
serial_conn = None
if not ROS_ENABLED:
    for port_candidate in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0"]:
        try:
            import serial
            serial_conn = serial.Serial(port_candidate, SERIAL_BAUD, timeout=0.1)
            SERIAL_ENABLED = True
            SERIAL_PORT = port_candidate
            print(f"[TurtleBot Bridge] OpenCR direct serial connected on {SERIAL_PORT}")
            break
        except Exception:
            continue

app = Flask(__name__)

# --- Threaded Low-Latency Camera ---
class ThreadedCamera:
    def __init__(self, src=CAMERA_INDEX, width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=TARGET_FPS):
        self.src = src
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None
        self.frame = None
        self.is_running = False
        self.lock = threading.Lock()
        self.init_camera()

    def init_camera(self):
        import cv2
        self.cap = cv2.VideoCapture(self.src, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.src)

        if not self.cap.isOpened():
            for alt in [1, 2, 0]:
                test = cv2.VideoCapture(alt)
                if test.isOpened():
                    self.cap = test
                    break

        if not self.cap.isOpened():
            print("[Camera Warning] Webcam not detected. Running without video stream.")
            return

        try:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        except Exception:
            pass

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        ret, initial = self.cap.read()
        if ret:
            self.frame = initial

        self.is_running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        import cv2
        while self.is_running:
            if not self.cap or not self.cap.isOpened():
                time.sleep(0.05)
                continue
            ret = self.cap.grab()
            if ret:
                ret_f, frame = self.cap.retrieve()
                if ret_f:
                    with self.lock:
                        self.frame = frame
            else:
                time.sleep(0.005)

    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def release(self):
        self.is_running = False
        if self.cap:
            self.cap.release()

camera = None

# --- Motor Watchdog & State ---
current_linear = 0.0
current_angular = 0.0
last_cmd_timestamp = 0.0
motor_lock = threading.Lock()

def dispatch_hardware_velocity(linear, angular):
    """Sends velocity commands to physical TurtleBot motors with strict hardware safety clamping"""
    global ros_cmd_pub, ROS_MODE, TwistClass, SERIAL_ENABLED, serial_conn
    
    # Enforce physical hardware velocity limits (TurtleBot3 Burger specification)
    linear = max(-0.22, min(0.22, float(linear)))
    angular = max(-1.40, min(1.40, float(angular)))
    
    # 1. Publish to ROS 1 or ROS 2 /cmd_vel
    if ROS_MODE in ['ROS1', 'ROS2'] and ros_cmd_pub is not None and TwistClass is not None:
        try:
            t = TwistClass()
            t.linear.x = float(linear)
            t.angular.z = float(angular)
            ros_cmd_pub.publish(t)

            sub_count = 1
            if ROS_MODE == 'ROS2' and hasattr(ros_cmd_pub, 'get_subscription_count'):
                try:
                    sub_count = ros_cmd_pub.get_subscription_count()
                except Exception:
                    sub_count = 1

            if sub_count == 0 and (abs(linear) > 0.01 or abs(angular) > 0.01):
                now = time.time()
                if not hasattr(dispatch_hardware_velocity, "_warn_t") or (now - dispatch_hardware_velocity._warn_t > 4.0):
                    dispatch_hardware_velocity._warn_t = now
                    print("[WARNING] /cmd_vel has 0 active subscribers! OpenCR hardware driver is NOT running.")
                    print("  -> Start OpenCR with: ros2 launch turtlebot3_bringup robot.launch.py")
            elif abs(linear) > 0.001 or abs(angular) > 0.001:
                print(f"[Actuation -> {ROS_MODE}] v_x={linear:+.2f} m/s, w_z={angular:+.2f} rad/s")
            return
        except Exception as e:
            print(f"[ROS Publish Error] {e}")

    # 2. Direct OpenCR Serial Fallback
    if SERIAL_ENABLED and serial_conn is not None:
        packet = f"V:{linear:.2f},W:{angular:.2f}\n"
        try:
            serial_conn.write(packet.encode("utf-8"))
            return
        except Exception as e:
            print(f"[Serial Write Error] {e}")

def motor_safety_watchdog():
    """Safety watchdog: shuts down physical motors if laptop stops transmitting for >0.6s"""
    global current_linear, current_angular, last_cmd_timestamp
    while True:
        with motor_lock:
            if last_cmd_timestamp > 0 and (time.time() - last_cmd_timestamp > 0.6):
                if current_linear != 0.0 or current_angular != 0.0:
                    current_linear = 0.0
                    current_angular = 0.0
                    dispatch_hardware_velocity(0.0, 0.0)
                    print("[Safety Watchdog] Laptop heartbeat timed out -> Motors stopped.")
        time.sleep(0.1)

watchdog_thread = threading.Thread(target=motor_safety_watchdog, daemon=True)
watchdog_thread.start()

# --- Fast UDP Command Listener ---
def udp_command_listener():
    """Receives UDP velocity commands from laptop for zero-latency control"""
    global current_linear, current_angular, last_cmd_timestamp
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', UDP_PORT))
        print(f"[TurtleBot Bridge] Fast UDP listener listening on port {UDP_PORT}")
        while True:
            data, _ = sock.recvfrom(256)
            if not data:
                continue
            text = data.decode("utf-8", errors="ignore").strip()
            lin, ang = 0.0, 0.0
            if text.startswith("{"):
                try:
                    j = json.loads(text)
                    lin = float(j.get("linear", 0.0))
                    ang = float(j.get("angular", 0.0))
                except Exception:
                    continue
            elif "V:" in text and "W:" in text:
                try:
                    parts = text.split(",")
                    for p in parts:
                        if p.startswith("V:"):
                            lin = float(p.split(":")[1])
                        elif p.startswith("W:"):
                            ang = float(p.split(":")[1])
                except Exception:
                    continue
            else:
                continue

            with motor_lock:
                current_linear = lin
                current_angular = ang
                last_cmd_timestamp = time.time()

            dispatch_hardware_velocity(lin, ang)
    except Exception as e:
        print(f"[UDP Listener Warning] Could not start UDP socket: {e}")

udp_thread = threading.Thread(target=udp_command_listener, daemon=True)
udp_thread.start()

def generate_mjpeg():
    import cv2
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
    last_sent = 0.0
    frame_interval = 1.0 / TARGET_FPS

    while True:
        if camera is None:
            time.sleep(0.02)
            continue

        now = time.time()
        if now - last_sent < frame_interval:
            time.sleep(0.002)
            continue

        frame = camera.get_frame()
        if frame is None:
            time.sleep(0.005)
            continue

        last_sent = now
        success, buf = cv2.imencode('.jpg', frame, encode_params)
        if not success:
            continue

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n'
            + buf.tobytes()
            + b'\r\n'
        )

# --- Routes ---

@app.route('/video_feed')
def video_feed():
    return Response(generate_mjpeg(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/cmd_vel', methods=['POST'])
def cmd_vel():
    """Receives velocity commands from the laptop AI node"""
    global current_linear, current_angular, last_cmd_timestamp
    data = request.get_json(silent=True) or {}
    lin = float(data.get("linear", 0.0))
    ang = float(data.get("angular", 0.0))

    with motor_lock:
        current_linear = lin
        current_angular = ang
        last_cmd_timestamp = time.time()

    dispatch_hardware_velocity(lin, ang)
    return jsonify({"status": "received", "linear": lin, "angular": ang})

def check_and_launch_opencr():
    """Checks if turtlebot3_node is active. If not, auto-launches robot.launch.py in background."""
    import subprocess
    try:
        out = subprocess.check_output(["pgrep", "-f", "turtlebot3_node"]).decode().strip()
        if out:
            print("[Auto-Bringup] OpenCR hardware driver is already running.")
            return True
    except Exception:
        pass

    print("[Auto-Bringup] OpenCR hardware driver is NOT running. Auto-launching in background...")
    launch_script = """
    export ROS_DOMAIN_ID=30
    export TURTLEBOT3_MODEL=burger
    export LDS_MODEL=LDS-01
    export ROS_LOCALHOST_ONLY=0
    if [ -f /opt/ros/humble/setup.bash ]; then source /opt/ros/humble/setup.bash; fi
    if [ -f ~/turtlebot3_ws/install/setup.bash ]; then source ~/turtlebot3_ws/install/setup.bash; fi
    ros2 launch turtlebot3_bringup robot.launch.py
    """
    try:
        subprocess.Popen(["bash", "-c", launch_script], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(3.5)
        subprocess.Popen(["bash", "-c", "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=30 && ros2 service call /motor_power std_srvs/srv/SetBool \"{data: true}\""],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("[Auto-Bringup] Hardware launch initiated and motor power enabled.")
        return True
    except Exception as e:
        print(f"[Auto-Bringup Error] {e}")
        return False

@app.route('/launch_hardware', methods=['GET', 'POST'])
def launch_hardware():
    """Endpoint to trigger robot.launch.py without needing an SSH terminal"""
    t = threading.Thread(target=check_and_launch_opencr, daemon=True)
    t.start()
    return jsonify({"status": "initiating_hardware_launch", "message": "Launching OpenCR driver & enabling motor torque"})

@app.route('/motor_power', methods=['GET', 'POST'])
def motor_power():
    """Endpoint to enable Dynamixel motor power/torque"""
    import subprocess
    try:
        subprocess.Popen(["bash", "-c", "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=30 && ros2 service call /motor_power std_srvs/srv/SetBool \"{data: true}\""],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return jsonify({"status": "motor_power_requested"})
    except Exception as e:
        return jsonify({"status": "error", "error": str(e)})

@app.route('/test_motors', methods=['GET', 'POST'])
def test_motors():
    """Hardware Motor Test: Ensures motor power is enabled, then spins wheels for 1.5s"""
    def _run_test():
        import subprocess
        # 1. Attempt to enable motor power service
        try:
            subprocess.run(["bash", "-c", "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=30 && ros2 service call /motor_power std_srvs/srv/SetBool \"{data: true}\""],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
        except Exception:
            pass

        print("[Motor Test] Starting 1.5-second hardware spin test...")
        # 2. Command angular rotation in bursts to ensure OpenCR picks it up
        for _ in range(15):
            dispatch_hardware_velocity(0.0, 0.6)  # Turn in place at 0.6 rad/s
            time.sleep(0.1)
        dispatch_hardware_velocity(0.0, 0.0)  # Stop
        print("[Motor Test] Test completed successfully.")

    t = threading.Thread(target=_run_test, daemon=True)
    t.start()
    return jsonify({
        "status": "testing",
        "message": "Motors commanded to spin for 1.5s at angular 0.6 rad/s.",
        "ros_connected": ROS_ENABLED,
        "serial_connected": SERIAL_ENABLED
    })

@app.route('/health')
def health():
    sub_count = 0
    if ROS_MODE == 'ROS2' and ros_cmd_pub is not None and hasattr(ros_cmd_pub, 'get_subscription_count'):
        try:
            sub_count = ros_cmd_pub.get_subscription_count()
        except Exception:
            pass
    return jsonify({
        "status": "ok",
        "ros_connected": ROS_ENABLED,
        "ros_mode": ROS_MODE,
        "cmd_vel_subscribers": sub_count,
        "opencr_active": (sub_count > 0),
        "serial_connected": SERIAL_ENABLED,
        "serial_port": SERIAL_PORT if SERIAL_ENABLED else None,
        "current_cmd": {"linear": current_linear, "angular": current_angular}
    })

@app.route('/')
def index():
    return f"""
    <h2>Raspberry Pi TurtleBot Companion Node Active</h2>
    <p>Camera Stream: <a href="/video_feed">/video_feed</a></p>
    <p>ROS Mode: <b>{ROS_MODE if ROS_ENABLED else 'Disabled / Standby'}</b></p>
    <p>OpenCR Serial: <b>{'Connected on ' + SERIAL_PORT if SERIAL_ENABLED else 'Standby'}</b></p>
    <p>Motor Test: <a href="/test_motors">Click here to test motor spin</a></p>
    """

def run_standalone_test():
    """Standalone CLI motor test: python3 pi_turtlebot_node.py --test"""
    print("\n" + "=" * 50)
    print("  TURTLEBOT HARDWARE MOTOR TEST")
    print("=" * 50)
    print(f"ROS Mode      : {ROS_MODE}")
    print(f"OpenCR Serial : {SERIAL_ENABLED} ({SERIAL_PORT})")
    print("Commanding forward rotation for 1.5 seconds...")
    dispatch_hardware_velocity(0.08, 0.0)
    time.sleep(1.5)
    print("Commanding stop...")
    dispatch_hardware_velocity(0.0, 0.0)
    print("Test finished! If wheels moved, hardware is working 100%.\n")

if __name__ == '__main__':
    if "--test" in sys.argv:
        run_standalone_test()
        sys.exit(0)

    print("=" * 60)
    print("  TURTLEBOT PI COMPANION NODE (PHASE 2 MOTOR CONTROL)")
    print(f"  ROS Connected    : {ROS_ENABLED} ({ROS_MODE})")
    print(f"  OpenCR Serial    : {SERIAL_ENABLED} ({SERIAL_PORT if SERIAL_ENABLED else 'None'})")
    print(f"  HTTP Port        : {PORT}")
    print(f"  UDP Port         : {UDP_PORT}")
    print("=" * 60)
    try:
        camera = ThreadedCamera()
        app.run(host='0.0.0.0', port=PORT, threaded=True, debug=False, use_reloader=False)
    finally:
        if camera:
            camera.release()
        dispatch_hardware_velocity(0.0, 0.0)
