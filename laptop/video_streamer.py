"""
Video Streamer Module:
Receives the Raspberry Pi camera as UDP datagrams (see pi/tb_proto.py), keeps only the
newest JPEG, and wakes consumers through a condition instead of making them poll.
Also carries the Pi's 2 Hz status packet and a fallback simulation generator.
"""

import os
import socket
import sys
import time
import math
import threading
import cv2
import numpy as np
from config import Config

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pi"))
import tb_proto  # noqa: E402  (wire format shared with the Pi node)


class VideoStreamer:
    def __init__(self, stream_url=None):
        self.stream_url = stream_url or Config.get_stream_url()
        self.lock = threading.RLock()
        self.cond = threading.Condition(self.lock)
        self.latest_jpeg = None
        self.is_connected = False
        self.is_running = True
        self.source_mode = "network"  # 'network', 'sim'
        self.fps = 0.0
        self.last_frame_time = 0.0
        self.status_message = "Initializing..."
        self.frame_count = 0
        self.sim_tick = 0
        self.pi_status = {}
        self.pi_status_time = 0.0
        self.scan = None              # {"bins": [...], "pose": (x, y, yaw) or None}
        self.scan_count = 0
        self.kbps = 0.0
        self.dropped = 0
        self.pi_ip = Config.PI_IP
        self.sock = None
        self._last_standby = 0.0

        threading.Thread(target=self._stream_loop, daemon=True).start()
        threading.Thread(target=self._hello_loop, daemon=True).start()

    def update_pi_ip(self, ip):
        with self.lock:
            self.pi_ip = ip
            self.stream_url = f"udp://{ip}:{tb_proto.CMD_PORT}"
            self.status_message = f"Waiting for video from {ip}..."
            self.is_connected = False

    def set_source_mode(self, mode):
        with self.lock:
            self.source_mode = mode
            self.status_message = f"Switched source mode to {mode}"
            self.is_connected = mode == "sim"
            self.cond.notify_all()

    def wait_new(self, last_count, timeout=0.5):
        """Block until a frame newer than last_count exists. Returns (count, jpeg)."""
        with self.cond:
            if self.frame_count == last_count:
                self.cond.wait(timeout)
            return self.frame_count, self.latest_jpeg

    def wait_scan(self, last_count, timeout=0.5):
        with self.cond:
            if self.scan_count == last_count:
                self.cond.wait(timeout)
            return self.scan_count, self.scan

    def snapshot(self):
        with self.lock:
            now = time.time()
            return {
                "connected": self.is_connected,
                "status_message": self.status_message,
                "source_mode": self.source_mode,
                "stream_url": self.stream_url,
                "fps": round(self.fps, 1),
                "kbps": round(self.kbps),
                "dropped": self.dropped,
                "frame_age_ms": int((now - self.last_frame_time) * 1000) if self.last_frame_time else None,
                "pi": dict(self.pi_status) if now - self.pi_status_time < 3.0 else {},
            }

    def _publish(self, jpeg, connected=None):
        with self.cond:
            self.latest_jpeg = jpeg
            self.frame_count += 1
            self.last_frame_time = time.time()
            if connected is not None:
                self.is_connected = connected
            self.cond.notify_all()

    # --- network ---

    def _open_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        try:
            # Windows reports an ICMP "port unreachable" from the Pi as a recv error. Ignore it.
            sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        except (AttributeError, OSError, ValueError):
            pass
        sock.bind(("0.0.0.0", Config.VIDEO_PORT))
        sock.settimeout(0.2)
        return sock

    def _hello_loop(self):
        """Tells the Pi where to send video. Sent from the video socket so the reply path is open."""
        while self.is_running:
            sock = self.sock
            if sock is not None and self.source_mode == "network":
                try:
                    sock.sendto(tb_proto.HELLO, (self.pi_ip, tb_proto.CMD_PORT))
                except OSError:
                    pass
            time.sleep(0.5)

    def _stream_loop(self):
        while self.is_running:
            if self.source_mode == "sim":
                self._generate_simulation_frame()
                time.sleep(1.0 / Config.TARGET_FPS)
                continue
            try:
                sock = self._open_socket()
            except OSError as e:
                with self.lock:
                    self.is_connected = False
                    self.status_message = f"Cannot listen on UDP {Config.VIDEO_PORT}: {e}"
                self._generate_standby_frame(f"UDP {Config.VIDEO_PORT} is busy")
                time.sleep(1.0)
                continue
            self.sock = sock
            try:
                self._receive(sock)
            finally:
                self.sock = None
                sock.close()

    def _receive(self, sock):
        asm = tb_proto.Reassembler()
        window_t = time.time()
        window_frames = 0
        window_bytes = 0
        while self.is_running and self.source_mode == "network":
            try:
                data, _addr = sock.recvfrom(2048)
            except socket.timeout:
                data = None
            except ConnectionResetError:
                continue
            except OSError:
                return
            now = time.time()
            if data:
                window_bytes += len(data)
                result = asm.feed(data)
                if result is not None:
                    kind, payload, _ts = result
                    if kind == "video":
                        window_frames += 1
                        with self.lock:
                            self.status_message = f"Receiving video from {self.pi_ip}"
                        self._publish(payload, connected=True)
                    elif kind == "scan":
                        with self.cond:
                            self.scan = payload
                            self.scan_count += 1
                            self.cond.notify_all()
                    else:
                        with self.lock:
                            self.pi_status = payload
                            self.pi_status_time = now
            if now - window_t >= 1.0:
                dt = now - window_t
                with self.lock:
                    self.fps = window_frames / dt
                    self.kbps = window_bytes * 8 / 1000.0 / dt
                    self.dropped = asm.frames_dropped
                window_t, window_frames, window_bytes = now, 0, 0
            if now - self.last_frame_time > Config.STALE_FRAME_TIMEOUT:
                with self.lock:
                    if self.is_connected:
                        self.status_message = f"No video from {self.pi_ip}"
                    self.is_connected = False
                    self.fps = 0.0
                if now - self._last_standby > 0.5:
                    self._last_standby = now
                    self._generate_standby_frame(f"Waiting for Pi at {self.pi_ip} (UDP {tb_proto.CMD_PORT})")

    # --- generated frames ---

    def _generate_standby_frame(self, message):
        """Test pattern shown while the Pi is silent."""
        h, w = Config.FRAME_HEIGHT, Config.FRAME_WIDTH
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        for x in range(0, w, 40):
            cv2.line(frame, (x, 0), (x, h), (25, 30, 45), 1)
        for y in range(0, h, 40):
            cv2.line(frame, (0, y), (w, y), (25, 30, 45), 1)

        pulse = int(127 + 127 * math.sin(time.time() * 3))
        cx, cy = w // 2, h // 2
        cv2.rectangle(frame, (cx - 230, cy - 65), (cx + 230, cy + 65), (35, 45, 65), -1)
        cv2.rectangle(frame, (cx - 230, cy - 65), (cx + 230, cy + 65), (0, pulse, 255), 2)
        cv2.putText(frame, "RASPBERRY PI FEED OFFLINE", (cx - 190, cy - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)
        cv2.putText(frame, message[:52], (cx - 215, cy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1)
        cv2.putText(frame, "Switch to [SIMULATION MODE] in Dashboard to test tracking", (cx - 215, cy + 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 180), 1)

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if ok:
            self._publish(buf.tobytes())

    def _generate_simulation_frame(self):
        """Animated humanoid figure walking back and forth."""
        h, w = Config.FRAME_HEIGHT, Config.FRAME_WIDTH
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        for y in range(h):
            val = int(15 + 25 * (y / h))
            frame[y, :] = (val, val + 5, val + 15)
        for y in range(h // 2, h, 25):
            cv2.line(frame, (0, y), (w, y), (35, 45, 65), 1)

        self.sim_tick += 1
        t = self.sim_tick * 0.05
        pos_x = int(w / 2 + math.sin(t * 0.7) * 170)
        scale = 0.95 + 0.35 * math.cos(t * 0.4)
        base_h = int(250 * scale)
        top_y = int(100 - (scale - 1.0) * 35)
        bot_y = top_y + base_h

        head_radius = int(22 * scale)
        head_cy = top_y + head_radius
        cv2.circle(frame, (pos_x, head_cy), head_radius, (190, 220, 255), -1)
        cv2.circle(frame, (pos_x, head_cy), head_radius, (255, 255, 255), 2)

        torso_top = head_cy + head_radius
        torso_bot = torso_top + int(90 * scale)
        cv2.line(frame, (pos_x, torso_top), (pos_x, torso_bot), (150, 180, 220), int(16 * scale))

        shoulder_w = int(38 * scale)
        arm_swing = math.sin(t * 4) * 22 * scale
        cv2.line(frame, (pos_x - shoulder_w, torso_top + 10),
                 (int(pos_x - shoulder_w - arm_swing), torso_bot), (120, 160, 200), int(8 * scale))
        cv2.line(frame, (pos_x + shoulder_w, torso_top + 10),
                 (int(pos_x + shoulder_w + arm_swing), torso_bot), (120, 160, 200), int(8 * scale))

        leg_swing = math.sin(t * 4) * 30 * scale
        cv2.line(frame, (pos_x - 14, torso_bot),
                 (int(pos_x - 20 + leg_swing), bot_y), (100, 140, 180), int(10 * scale))
        cv2.line(frame, (pos_x + 14, torso_bot),
                 (int(pos_x + 20 - leg_swing), bot_y), (100, 140, 180), int(10 * scale))

        cv2.putText(frame, "SIMULATION TEST FEED ACTIVE", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 180), 2)
        cv2.putText(frame, f"Simulated Distance: ~{round(1.8 / scale, 2)}m | Move X: {pos_x - w//2}px", (20, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
        if ok:
            with self.lock:
                self.status_message = "Simulation Mode Active"
                self.fps = Config.TARGET_FPS
            self._publish(buf.tobytes(), connected=True)

    def stop(self):
        self.is_running = False
