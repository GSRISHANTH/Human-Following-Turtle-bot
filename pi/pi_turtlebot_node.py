#!/usr/bin/env python3
"""
Raspberry Pi companion node for the TurtleBot human follower. Pure UDP, nothing is hosted.

    in   UDP :8888   "V:<m/s>,W:<rad/s>,S:<seq>"  velocity, also the heartbeat
    in   UDP :8888   "HELLO"                      the laptop announces where to send video
    out  UDP -> laptop  JPEG frames in datagrams, plus a small status packet at 2 Hz

Velocity goes to ROS 2 (or ROS 1) /cmd_vel, with OpenCR serial as a last resort.
If no packet arrives for 0.6 s the wheels are stopped.

    python3 pi_turtlebot_node.py                 camera + motors
    python3 pi_turtlebot_node.py --no-motors     camera and logging only
    python3 pi_turtlebot_node.py --raw-mjpeg     forward the camera's own MJPEG, no Pi re-encode
    python3 pi_turtlebot_node.py --test          spin the wheels briefly and exit
"""

import glob
import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tb_proto

os.environ["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "30")
os.environ["TURTLEBOT3_MODEL"] = os.environ.get("TURTLEBOT3_MODEL", "burger")
os.environ["LDS_MODEL"] = os.environ.get("LDS_MODEL", "LDS-02")
os.environ["ROS_LOCALHOST_ONLY"] = "0"

for ros_path in glob.glob('/opt/ros/*/lib/python*/dist-packages') + \
                glob.glob('/opt/ros/*/local/lib/python*/dist-packages') + \
                glob.glob('/opt/ros/*/lib/python*/site-packages') + \
                glob.glob(os.path.expanduser('~/turtlebot3_ws/install/*/lib/python*/site-packages')) + \
                glob.glob(os.path.expanduser('~/turtlebot3_ws/devel/lib/python*/dist-packages')) + \
                glob.glob(os.path.expanduser('~/catkin_ws/devel/lib/python*/dist-packages')):
    if ros_path not in sys.path:
        sys.path.append(ros_path)

NO_MOTORS = "--no-motors" in sys.argv
RAW_MJPEG = "--raw-mjpeg" in sys.argv

CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
TARGET_FPS = 30
JPEG_QUALITY = 50
CMD_TIMEOUT = 0.35  # about ten missed packets at 30 Hz
PEER_TIMEOUT = 3.0
MAX_LINEAR = 0.22   # TurtleBot3 Burger limits
MAX_ANGULAR = 1.40

ROS_MODE = None  # 'ROS1', 'ROS2', or None
ros_cmd_pub = None
TwistClass = None
SERIAL_ENABLED = False
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUD = 115200
serial_conn = None


def is_roscore_online():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        s.connect(('127.0.0.1', 11311))
        s.close()
        return True
    except Exception:
        return False


if is_roscore_online():
    try:
        import rospy
        from geometry_msgs.msg import Twist as Twist1
        rospy.init_node('human_follower_pi_bridge', anonymous=True, disable_signals=True)
        ros_cmd_pub = rospy.Publisher('/cmd_vel', Twist1, queue_size=1)
        TwistClass = Twist1
        ROS_MODE = 'ROS1'
        print("[Pi] Connected to ROS 1 /cmd_vel")
    except Exception as e1:
        print(f"[Pi] ROS 1 init error: {e1}")

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
        print("[Pi] Connected to ROS 2 /cmd_vel")
    except Exception as e2:
        print(f"[Pi] ROS 2 not available: {e2}")

ROS_ENABLED = ROS_MODE is not None

# --- LiDAR (/scan) and odometry (/odom), ROS 2 only. Read-only: this never moves the robot. ---
scan_lock = threading.Lock()
scan_bins = None      # list of SCAN_BINS ints, millimetres
scan_time = 0.0
scan_hz = 0.0
odom_pose = None      # (x, y, yaw)


def _scan_cb(msg):
    global scan_bins, scan_time, scan_hz
    import numpy as np
    r = np.asarray(msg.ranges, dtype=np.float32)
    ang = msg.angle_min + np.arange(r.size, dtype=np.float32) * msg.angle_increment
    ok = np.isfinite(r) & (r >= max(msg.range_min, 0.05)) & (r <= msg.range_max)
    if not ok.any():
        return
    idx = (np.mod(ang[ok], 2 * np.pi) / (2 * np.pi) * tb_proto.SCAN_BINS).astype(np.int32) % tb_proto.SCAN_BINS
    mm = np.full(tb_proto.SCAN_BINS, 65535, dtype=np.uint32)
    np.minimum.at(mm, idx, (r[ok] * 1000).astype(np.uint32))
    mm[mm == 65535] = 0
    now = time.monotonic()
    with scan_lock:
        if scan_time:
            scan_hz = 0.8 * scan_hz + 0.2 / max(now - scan_time, 1e-3)
        scan_bins = [int(v) for v in mm]
        scan_time = now


def _odom_cb(msg):
    global odom_pose
    import math
    q = msg.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    with scan_lock:
        odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)


if ROS_MODE == 'ROS2':
    try:
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry
        ros2_node.create_subscription(LaserScan, '/scan', _scan_cb, qos_profile_sensor_data)
        ros2_node.create_subscription(Odometry, '/odom', _odom_cb, qos_profile_sensor_data)
        print("[Pi] Subscribed to /scan and /odom")
    except Exception as e:
        print(f"[Pi] LiDAR subscriptions failed: {e}")

if NO_MOTORS:
    print("[Pi] --no-motors: /cmd_vel and OpenCR serial are never written.")
elif not ROS_ENABLED:
    print("[Pi] No ROS found. Trying OpenCR serial. For ROS 2: ros2 launch turtlebot3_bringup robot.launch.py")
    for candidate in ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0"]:
        try:
            import serial
            serial_conn = serial.Serial(candidate, SERIAL_BAUD, timeout=0.1)
            SERIAL_ENABLED = True
            SERIAL_PORT = candidate
            print(f"[Pi] OpenCR serial on {SERIAL_PORT}")
            break
        except Exception:
            continue


# ---------------------------------------------------------------------------
# Motors
# ---------------------------------------------------------------------------
motor_lock = threading.Lock()
cur_lin = 0.0
cur_ang = 0.0
last_cmd_time = 0.0      # monotonic
last_seq = None
cmd_count = 0
_warned = 0.0


def dispatch_hardware_velocity(linear, angular):
    global _warned
    if NO_MOTORS:
        return
    linear = max(-MAX_LINEAR, min(MAX_LINEAR, float(linear)))
    angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, float(angular)))

    if ros_cmd_pub is not None and TwistClass is not None:
        try:
            t = TwistClass()
            t.linear.x = linear
            t.angular.z = angular
            ros_cmd_pub.publish(t)
            if ROS_MODE == 'ROS2' and (abs(linear) > 0.01 or abs(angular) > 0.01):
                subs = ros_cmd_pub.get_subscription_count()
                if subs == 0 and time.monotonic() - _warned > 4.0:
                    _warned = time.monotonic()
                    print("[Pi] WARNING: /cmd_vel has no subscriber. OpenCR bringup is not running.")
            return
        except Exception as e:
            print(f"[Pi] publish error: {e}")

    if SERIAL_ENABLED and serial_conn is not None:
        try:
            serial_conn.write(f"V:{linear:.2f},W:{angular:.2f}\n".encode("ascii"))
        except Exception as e:
            print(f"[Pi] serial error: {e}")


def apply_command(lin, ang):
    global cur_lin, cur_ang, last_cmd_time
    with motor_lock:
        cur_lin, cur_ang = lin, ang
        last_cmd_time = time.monotonic()
    dispatch_hardware_velocity(lin, ang)


def watchdog_loop():
    global cur_lin, cur_ang
    while True:
        time.sleep(0.05)
        with motor_lock:
            stale = last_cmd_time > 0 and (time.monotonic() - last_cmd_time) > CMD_TIMEOUT
            moving = cur_lin != 0.0 or cur_ang != 0.0
            if stale and moving:
                cur_lin = cur_ang = 0.0
            else:
                stale = False
        if stale:
            dispatch_hardware_velocity(0.0, 0.0)
            print("[Pi] Command heartbeat lost -> motors stopped.")


# ---------------------------------------------------------------------------
# Network
# ---------------------------------------------------------------------------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
peer = None          # (ip, port) of the laptop video socket
peer_time = 0.0      # monotonic
peer_lock = threading.Lock()


def udp_rx_loop():
    global peer, peer_time, last_seq, cmd_count
    while True:
        try:
            data, addr = sock.recvfrom(512)
        except OSError:
            return
        if data.startswith(b"HELLO"):
            with peer_lock:
                if peer != addr:
                    print(f"[Pi] Video peer: {addr[0]}:{addr[1]}")
                peer = addr
                peer_time = time.monotonic()
            continue
        parsed = tb_proto.parse_command(data)
        if parsed is None:
            continue
        lin, ang, seq = parsed
        # Out-of-order packets are dropped, but after half a second of silence the sender has restarted
        # (its counter starts over), so its first packet must be accepted.
        if not tb_proto.seq_is_newer(seq, last_seq) and (time.monotonic() - last_cmd_time) < 0.5:
            continue
        if seq is not None:
            last_seq = seq
        cmd_count += 1
        apply_command(lin, ang)


def current_peer():
    with peer_lock:
        if peer is not None and time.monotonic() - peer_time < PEER_TIMEOUT:
            return peer
    return None


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
class Camera:
    """Keeps only the newest frame. Encoding happens in the sender, not here."""

    def __init__(self):
        self.cap = None
        self.raw = False
        self.latest = None      # ndarray, or bytes when raw
        self.frame_no = 0
        self.cond = threading.Condition()
        self.running = True
        self.capture_fps = 0.0
        self._open()
        if self.cap is not None:
            threading.Thread(target=self._loop, daemon=True).start()

    def _open(self):
        import cv2
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap = cv2.VideoCapture(CAMERA_INDEX)
        if not cap.isOpened():
            for alt in (1, 2):
                cap = cv2.VideoCapture(alt)
                if cap.isOpened():
                    break
        if not cap.isOpened():
            print("[Pi] Camera not detected.")
            return
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if RAW_MJPEG:
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
            ok, frame = cap.read()
            if ok and frame is not None and frame.size > 4 and bytes(frame.reshape(-1)[:2]) == b"\xff\xd8":
                self.raw = True
                print("[Pi] Raw MJPEG passthrough: the Pi does not decode or re-encode.")
            else:
                print("[Pi] Camera gave no raw MJPEG; falling back to re-encode.")
                cap.release()
                cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap = cap
        print(f"[Pi] Camera open ({int(cap.get(3))}x{int(cap.get(4))}).")

    def _loop(self):
        n, t0 = 0, time.monotonic()
        while self.running:
            if not self.cap.grab():
                time.sleep(0.004)
                continue
            ok, frame = self.cap.retrieve()
            if not ok or frame is None:
                continue
            item = bytes(frame.reshape(-1)) if self.raw else frame
            with self.cond:
                self.latest = item
                self.frame_no += 1
                self.cond.notify_all()
            n += 1
            now = time.monotonic()
            if now - t0 >= 1.0:
                self.capture_fps = n / (now - t0)
                n, t0 = 0, now

    def wait_new(self, last_no, timeout):
        with self.cond:
            if self.frame_no == last_no:
                self.cond.wait(timeout)
            return self.frame_no, self.latest


def no_camera_jpeg():
    import cv2
    import numpy as np
    img = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    cv2.rectangle(img, (40, 160), (600, 320), (30, 40, 55), -1)
    cv2.putText(img, "TURTLEBOT CAMERA NOT DETECTED", (70, 220),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 180, 255), 2, cv2.LINE_AA)
    cv2.putText(img, "Plug the camera into the Pi.", (150, 270),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 210, 220), 1, cv2.LINE_AA)
    return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 60])[1].tobytes()


def sender_loop(camera):
    import cv2
    params = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
    placeholder = None
    last_no = -1
    fid = 0
    sent = 0
    t0 = time.monotonic()
    send_fps = 0.0
    last_status = 0.0
    min_gap = 1.0 / TARGET_FPS
    last_send = 0.0
    last_scan_sent = 0.0

    while True:
        if camera.cap is None:
            time.sleep(0.2)
            if placeholder is None:
                placeholder = no_camera_jpeg()
            frame_no, item = last_no + 1, placeholder
        else:
            frame_no, item = camera.wait_new(last_no, 0.25)

        dest = current_peer()
        now = time.monotonic()

        if dest is not None and now - last_status >= 0.5:
            last_status = now
            subs = 0
            if ROS_MODE == 'ROS2' and ros_cmd_pub is not None:
                try:
                    subs = ros_cmd_pub.get_subscription_count()
                except Exception:
                    pass
            with motor_lock:
                cmd_age = (now - last_cmd_time) if last_cmd_time else None
                lin, ang = cur_lin, cur_ang
            status = {
                "ros": ROS_MODE, "subs": subs, "serial": SERIAL_ENABLED,
                "motors": not NO_MOTORS, "camera": camera.cap is not None,
                "cam_fps": round(camera.capture_fps, 1), "send_fps": round(send_fps, 1),
                "raw": camera.raw, "cmd_age": None if cmd_age is None else round(cmd_age, 2),
                "cmds": cmd_count, "lin": lin, "ang": ang,
                "scan_hz": round(scan_hz, 1) if scan_time and now - scan_time < 1.0 else 0.0,
            }
            try:
                sock.sendto(tb_proto.pack_status(status, time.time()), dest)
            except OSError:
                pass

        if dest is not None:
            with scan_lock:
                fresh = scan_bins is not None and scan_time > last_scan_sent
                bins, pose, stamp = scan_bins, odom_pose, scan_time
            if fresh:
                last_scan_sent = stamp
                try:
                    sock.sendto(tb_proto.pack_scan(bins, pose, time.time()), dest)
                except OSError:
                    pass

        if frame_no == last_no or item is None or dest is None:
            continue
        if now - last_send < min_gap:
            time.sleep(min_gap - (now - last_send))
            frame_no, item = (frame_no, item) if camera.cap is None else camera.wait_new(last_no, 0.0)
        last_no = frame_no
        if isinstance(item, bytes):
            jpeg = item
        else:
            ok, buf = cv2.imencode(".jpg", item, params)
            if not ok:
                continue
            jpeg = buf.tobytes()

        fid += 1
        try:
            for i, pkt in enumerate(tb_proto.pack_frame(jpeg, fid, time.time())):
                sock.sendto(pkt, dest)
                if i % 6 == 5:
                    time.sleep(0.0007)  # pace the burst so Wi-Fi does not drop the tail of a frame
        except OSError:
            continue
        last_send = now
        sent += 1
        if now - t0 >= 1.0:
            send_fps = sent / (now - t0)
            sent, t0 = 0, now


def run_standalone_test():
    print(f"[Pi] Motor test. ROS={ROS_MODE} serial={SERIAL_ENABLED} no_motors={NO_MOTORS}")
    for _ in range(15):
        dispatch_hardware_velocity(0.0, 0.6)
        time.sleep(0.1)
    dispatch_hardware_velocity(0.0, 0.0)
    print("[Pi] Test finished.")


def main():
    if "--test" in sys.argv:
        run_standalone_test()
        return
    try:
        sock.bind(("0.0.0.0", tb_proto.CMD_PORT))
    except OSError as e:
        print(f"[Pi] Cannot bind UDP {tb_proto.CMD_PORT}: {e}")
        sys.exit(1)

    print("=" * 56)
    print("  TURTLEBOT PI NODE  (UDP only, nothing hosted)")
    print(f"  ROS: {ROS_MODE}   serial: {SERIAL_ENABLED}   motors: {not NO_MOTORS}")
    print(f"  UDP in  : {tb_proto.CMD_PORT} (commands + HELLO)")
    print("  UDP out : video + status to whoever sends HELLO")
    print("=" * 56)

    threading.Thread(target=watchdog_loop, daemon=True).start()
    threading.Thread(target=udp_rx_loop, daemon=True).start()
    camera = Camera()
    try:
        sender_loop(camera)
    except KeyboardInterrupt:
        pass
    finally:
        camera.running = False
        dispatch_hardware_velocity(0.0, 0.0)


if __name__ == '__main__':
    main()
