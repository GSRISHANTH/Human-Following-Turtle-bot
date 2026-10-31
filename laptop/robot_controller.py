"""
Robot Controller & Command Dispatcher (Phase 2):
Dispatches velocity commands (cmd_vel: linear_x, angular_z) to TurtleBot.
Supports:
1. HTTP REST (POST http://<pi_ip>:5000/cmd_vel) - Universal JSON endpoint
2. UDP Socket (Direct low-latency packets on port 8888)
3. Non-blocking async queue to guarantee jitter-free 20Hz control loop

Features:
- Acceleration/Slew-rate limiting (prevents jerky robot motion and wheel slip)
- Safety Watchdog / Deadman's Switch (halts robot if no signal for >0.5s)
- Emergency Stop interlock
- Search Recovery 360-degree rotation commander
"""

import time
import json
import queue
import socket
import threading
import urllib.request
import urllib.error
from urllib.parse import urlparse
from config import Config

class RobotController:
    def __init__(self):
        self.lock = threading.Lock()
        
        # Operational mode: 'AUTO', 'MANUAL', 'STOP', 'SEARCH_ROTATING'
        self.mode = "AUTO"
        self.protocol = Config.COMMAND_MODE  # 'http', 'rosbridge', 'udp', 'mock'
        
        # Current and Target Velocities
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Physical Velocity Limits (TurtleBot3 Burger / Waffle)
        self.max_linear = Config.MAX_LINEAR_VEL    # 0.22 m/s
        self.max_angular = Config.MAX_ANGULAR_VEL  # 1.5 rad/s
        
        # Acceleration limits (m/s^2 and rad/s^2) - tuned for snappy, immediate response
        self.max_accel_linear = 0.90
        self.max_accel_angular = 3.60
        
        # Safety & Watchdog
        self.last_command_time = time.time()
        self.watchdog_timeout = 0.6  # seconds
        self.emergency_stop = False
        
        # Hardware connection endpoints
        self.http_url = f"http://{Config.PI_IP}:{Config.PI_STREAM_PORT}/cmd_vel"
        self.udp_ip = Config.PI_IP
        self.udp_port = Config.UDP_PORT
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Non-blocking async HTTP worker queue
        self.http_queue = queue.Queue(maxsize=1)
        self.is_running = True
        
        self.http_thread = threading.Thread(target=self._http_worker, daemon=True)
        self.http_thread.start()
        
        # Background control loop (runs at ~20 Hz)
        self.dispatch_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.dispatch_thread.start()

    def set_mode(self, new_mode):
        """Sets controller mode: 'AUTO', 'MANUAL', 'STOP'"""
        with self.lock:
            self.mode = new_mode
            if new_mode == "STOP":
                self.emergency_stop = True
                self.target_linear = 0.0
                self.target_angular = 0.0
                self.current_linear = 0.0
                self.current_angular = 0.0
            else:
                self.emergency_stop = False

    def trigger_emergency_stop(self):
        """Immediately halts robot and locks motors"""
        self.set_mode("STOP")
        self._transmit_to_robot(0.0, 0.0)

    def release_emergency_stop(self):
        """Releases emergency stop and returns to AUTO mode"""
        self.set_mode("AUTO")

    def update_url(self, new_url):
        with self.lock:
            self.http_url = new_url
            try:
                parsed = urlparse(new_url)
                if parsed.hostname:
                    self.udp_ip = parsed.hostname
            except Exception:
                pass

    def update_auto_commands(self, linear_v, angular_w, can_move=True):
        """Called by CV pipeline when in AUTO_FOLLOW mode"""
        with self.lock:
            if self.mode != "AUTO":
                return

            if self.emergency_stop or not can_move:
                self.target_linear = 0.0
                self.target_angular = 0.0
                return

            self.target_linear = max(-0.10, min(self.max_linear, linear_v))
            self.target_angular = max(-self.max_angular, min(self.max_angular, angular_w))
            self.last_command_time = time.time()

    def update_manual_teleop(self, linear_v, angular_w):
        """Called by web UI joystick or WASD keyboard controls"""
        with self.lock:
            if self.mode != "MANUAL" or self.emergency_stop:
                return

            self.target_linear = max(-self.max_linear, min(self.max_linear, linear_v))
            self.target_angular = max(-self.max_angular, min(self.max_angular, angular_w))
            self.last_command_time = time.time()

    def command_search_rotation(self, direction=1):
        """Rotates robot in place (slow 360 scan) during target loss recovery"""
        with self.lock:
            if self.emergency_stop:
                return
            self.target_linear = 0.0
            self.target_angular = 0.38 * direction
            self.last_command_time = time.time()

    def get_telemetry(self):
        with self.lock:
            return {
                "mode": self.mode,
                "protocol": self.protocol,
                "emergency_stop": self.emergency_stop,
                "current_linear": round(self.current_linear, 2),
                "current_angular": round(self.current_angular, 2),
                "target_linear": round(self.target_linear, 2),
                "target_angular": round(self.target_angular, 2)
            }

    def _control_loop(self):
        """Control loop applying acceleration ramp and dispatching at 20Hz"""
        dt = 0.05  # 20 Hz
        while self.is_running:
            start_t = time.time()
            
            with self.lock:
                # Watchdog check: zero target if timeout exceeded
                if time.time() - self.last_command_time > self.watchdog_timeout:
                    self.target_linear = 0.0
                    self.target_angular = 0.0

                # Acceleration Slew Rate Limiter for Linear Velocity
                diff_lin = self.target_linear - self.current_linear
                step_lin = self.max_accel_linear * dt
                if abs(diff_lin) <= step_lin:
                    self.current_linear = self.target_linear
                else:
                    self.current_linear += step_lin if diff_lin > 0 else -step_lin

                # Acceleration Slew Rate Limiter for Angular Velocity
                diff_ang = self.target_angular - self.current_angular
                step_ang = self.max_accel_angular * dt
                if abs(diff_ang) <= step_ang:
                    self.current_angular = self.target_angular
                else:
                    self.current_angular += step_ang if diff_ang > 0 else -step_ang

                send_v = round(self.current_linear, 3)
                send_w = round(self.current_angular, 3)

            # Transmit velocities to robot hardware (non-blocking)
            self._transmit_to_robot(send_v, send_w)

            elapsed = time.time() - start_t
            sleep_time = max(0.01, dt - elapsed)
            time.sleep(sleep_time)

    def _transmit_to_robot(self, linear, angular):
        """Transmits velocity commands via UDP and queues for HTTP"""
        if self.protocol == "mock":
            return

        payload = {"linear": linear, "angular": angular, "timestamp": time.time()}

        # 1. Direct UDP transmission (ultra fast, non-blocking, ~0.05ms)
        self._send_udp(payload)

        # 2. Enqueue for HTTP transmission without blocking control loop
        if self.protocol == "http":
            try:
                # If queue has older pending item, drop it to ensure latest-only transmission
                if self.http_queue.full():
                    try:
                        self.http_queue.get_nowait()
                    except queue.Empty:
                        pass
                self.http_queue.put_nowait(payload)
            except Exception:
                pass

    def _http_worker(self):
        """Dedicated background worker for HTTP POST requests"""
        while self.is_running:
            try:
                payload = self.http_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                req = urllib.request.Request(
                    self.http_url,
                    data=json.dumps(payload).encode("utf-8"),
                    headers={"Content-Type": "application/json"}
                )
                urllib.request.urlopen(req, timeout=0.25)
            except Exception:
                pass
            finally:
                self.http_queue.task_done()

    def _send_udp(self, payload):
        """UDP datagram to Raspberry Pi port 8888"""
        try:
            msg = f"V:{payload['linear']:.2f},W:{payload['angular']:.2f}\n".encode("utf-8")
            self.udp_sock.sendto(msg, (self.udp_ip, self.udp_port))
        except Exception:
            pass

    def stop(self):
        self.is_running = False
        self._transmit_to_robot(0.0, 0.0)
