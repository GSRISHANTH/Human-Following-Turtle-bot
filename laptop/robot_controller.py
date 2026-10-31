"""
Robot Controller & Command Dispatcher (Phase 2):
Dispatches velocity commands (cmd_vel: linear_x, angular_z) to TurtleBot as UDP
datagrams to the Pi on port 8888, at a fixed rate that doubles as the Pi's heartbeat.

Features:
- Acceleration/Slew-rate limiting (prevents jerky robot motion and wheel slip)
- Safety Watchdog / Deadman's Switch (halts robot if no signal for >0.5s)
- Emergency Stop interlock
- Search Recovery 360-degree rotation commander
"""

import time
import socket
import threading
from config import Config
from video_streamer import tb_proto

class RobotController:
    def __init__(self):
        self.lock = threading.Lock()
        
        # Operational mode: 'AUTO', 'MANUAL', 'STOP', 'SEARCH_ROTATING'
        self.mode = "AUTO"
        self.protocol = Config.COMMAND_MODE  # 'udp' or 'mock'
        
        # Current and Target Velocities
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Physical Velocity Limits (TurtleBot3 Burger / Waffle)
        self.max_linear = Config.MAX_LINEAR_VEL    # 0.22 m/s
        self.max_angular = Config.MAX_ANGULAR_VEL  # 1.5 rad/s
        
        # Acceleration limits (m/s^2 and rad/s^2) - tuned for snappy, immediate response
        self.max_accel_linear = Config.MAX_ACCEL_LINEAR
        self.max_accel_angular = Config.MAX_ACCEL_ANGULAR
        
        # Safety & Watchdog
        self.last_command_time = time.time()
        self.watchdog_timeout = 0.6  # seconds
        self.emergency_stop = False
        self.front_clear = None       # LiDAR nearest return straight ahead (m), person included
        self.front_clear_time = 0.0
        
        # Hardware connection endpoints
        self.udp_ip = Config.PI_IP
        self.udp_port = Config.UDP_PORT
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.udp_sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        except (AttributeError, OSError, ValueError):
            pass
        # Starts at the wall-clock second so a restarted laptop is never older than the Pi's last seq.
        self.seq = int(time.time() * 100) & 0xFFFFFFFF
        self.is_running = True

        # Background control loop
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

    def set_front_clearance(self, meters, stamp):
        with self.lock:
            self.front_clear = meters
            self.front_clear_time = stamp

    def update_pi_ip(self, ip):
        with self.lock:
            self.udp_ip = ip

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
        """Control loop applying acceleration ramp and dispatching at Config.COMMAND_HZ"""
        period = 1.0 / Config.COMMAND_HZ
        last_t = time.time()
        while self.is_running:
            start_t = time.time()
            dt = min(0.2, max(0.001, start_t - last_t))
            last_t = start_t

            with self.lock:
                # Watchdog check: zero target if timeout exceeded
                if time.time() - self.last_command_time > self.watchdog_timeout:
                    self.target_linear = 0.0
                    self.target_angular = 0.0

                # LiDAR hard stop, independent of vision: never drive into something right ahead
                if (self.front_clear is not None and time.time() - self.front_clear_time < 0.6
                        and self.front_clear < Config.LIDAR_HARD_STOP_M and self.target_linear > 0.0):
                    self.target_linear = 0.0
                    self.current_linear = min(self.current_linear, 0.0)

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
            time.sleep(max(0.005, period - elapsed))

    def _transmit_to_robot(self, linear, angular):
        """One UDP datagram. Sent every cycle even when zero, so silence on the Pi means a dead link."""
        if self.protocol == "mock":
            return
        with self.lock:
            self.seq = (self.seq + 1) & 0xFFFFFFFF
            seq, ip = self.seq, self.udp_ip
        try:
            self.udp_sock.sendto(tb_proto.encode_command(linear, angular, seq), (ip, self.udp_port))
        except OSError:
            pass

    def stop(self):
        self.is_running = False
        self._transmit_to_robot(0.0, 0.0)
