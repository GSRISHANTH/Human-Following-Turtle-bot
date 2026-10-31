"""
Video Streamer Module:
Handles low-latency MJPEG stream ingestion from Raspberry Pi,
auto-reconnect, fallback simulation generator, and frame caching.
Uses RLock to prevent any recursive deadlock.
"""

import time
import math
import threading
import urllib.request
import urllib.error
import cv2
import numpy as np
from config import Config

class VideoStreamer:
    def __init__(self, stream_url=None):
        self.stream_url = stream_url or Config.get_stream_url()
        self.lock = threading.RLock()
        self.latest_frame = None
        self.is_connected = False
        self.is_running = True
        self.source_mode = "network" # 'network', 'sim'
        self.fps = 0.0
        self.last_frame_time = 0
        self.status_message = "Initializing..."
        self.reconnect_interval = 2.0 # seconds
        self.frame_count = 0
        self.sim_tick = 0
        
        # Start background worker thread
        self.worker_thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.worker_thread.start()

    def update_stream_url(self, new_url):
        with self.lock:
            self.stream_url = new_url
            self.status_message = f"Switching to {new_url}..."
            self.is_connected = False

    def set_source_mode(self, mode):
        """Mode: 'network', 'sim'"""
        with self.lock:
            self.source_mode = mode
            self.status_message = f"Switched source mode to {mode}"
            self.is_connected = False

    def get_latest_frame(self):
        with self.lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy(), self.is_connected, self.status_message
            return None, self.is_connected, self.status_message

    def _stream_loop(self):
        """Main loop that continuously feeds frames to self.latest_frame"""
        while self.is_running:
            with self.lock:
                current_mode = self.source_mode

            if current_mode == "sim":
                self._generate_simulation_frame()
                time.sleep(1.0 / Config.TARGET_FPS)
                continue

            # Try network stream
            success = self._read_mjpeg_stream()
            if not success:
                time.sleep(self.reconnect_interval)

    def _read_mjpeg_stream(self):
        """Reads MJPEG stream from HTTP endpoint with zero-latency buffer drop"""
        with self.lock:
            req_url = self.stream_url
            self.status_message = f"Connecting to {req_url}..."

        try:
            req = urllib.request.Request(
                req_url,
                headers={"User-Agent": "HumanFollower-Laptop/1.0"}
            )
            response = urllib.request.urlopen(req, timeout=3.0)
            
            with self.lock:
                self.status_message = f"Connected to {req_url}"
                self.is_connected = True

            byte_buffer = b""
            last_fps_calc = time.time()
            frames_in_second = 0

            while self.is_running:
                with self.lock:
                    if self.source_mode != "network" or req_url != self.stream_url:
                        break

                chunk = response.read(8192)
                if not chunk:
                    break
                byte_buffer += chunk

                # Prevent buffer growth and stale frame accumulation
                if len(byte_buffer) > 131072:
                    byte_buffer = byte_buffer[-65536:]

                # Zero-Latency Purge: Always jump to the NEWEST complete JPEG frame
                b = byte_buffer.rfind(b"\xff\xd9")
                if b != -1:
                    a = byte_buffer.rfind(b"\xff\xd8", 0, b)
                    if a != -1:
                        jpg_data = byte_buffer[a:b + 2]
                        byte_buffer = byte_buffer[b + 2:] # purge all older frames!

                        frame = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        now = time.time()
                        frames_in_second += 1
                        if now - last_fps_calc >= 1.0:
                            calc_fps = frames_in_second / (now - last_fps_calc)
                            frames_in_second = 0
                            last_fps_calc = now
                        else:
                            calc_fps = self.fps

                        with self.lock:
                            self.latest_frame = frame
                            self.last_frame_time = now
                            self.frame_count += 1
                            self.is_connected = True
                            self.fps = calc_fps

            return True

        except urllib.error.URLError as e:
            err_msg = str(e.reason) if hasattr(e, "reason") else str(e)
            with self.lock:
                self.is_connected = False
                self.status_message = f"Stream offline ({err_msg})"
            self._generate_standby_frame(f"Waiting for Pi at {req_url}")
            return False

        except Exception as e:
            with self.lock:
                self.is_connected = False
                self.status_message = f"Stream error: {e}"
            self._generate_standby_frame(f"Stream error: {e}")
            return False

    def _generate_standby_frame(self, message):
        """Generates a high-tech standby test pattern when Pi is offline"""
        h, w = Config.FRAME_HEIGHT, Config.FRAME_WIDTH
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        
        # Grid lines (dark blue/slate)
        for x in range(0, w, 40):
            cv2.line(frame, (x, 0), (x, h), (25, 30, 45), 1)
        for y in range(0, h, 40):
            cv2.line(frame, (0, y), (w, y), (25, 30, 45), 1)

        t = time.time()
        pulse = int(127 + 127 * math.sin(t * 3))
        
        cx, cy = w // 2, h // 2
        cv2.rectangle(frame, (cx - 230, cy - 65), (cx + 230, cy + 65), (35, 45, 65), -1)
        cv2.rectangle(frame, (cx - 230, cy - 65), (cx + 230, cy + 65), (0, pulse, 255), 2)

        cv2.putText(frame, "RASPBERRY PI FEED OFFLINE", (cx - 190, cy - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)
        cv2.putText(frame, message[:48], (cx - 205, cy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1)
        cv2.putText(frame, "Switch to [SIMULATION MODE] in Dashboard to test tracking", (cx - 215, cy + 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 180), 1)

        with self.lock:
            self.latest_frame = frame
            self.last_frame_time = time.time()

    def _generate_simulation_frame(self):
        """Generates an animated humanoid figure walking back and forth"""
        h, w = Config.FRAME_HEIGHT, Config.FRAME_WIDTH
        frame = np.zeros((h, w, 3), dtype=np.uint8)

        # Gradient background
        for y in range(h):
            val = int(15 + 25 * (y / h))
            frame[y, :] = (val, val + 5, val + 15)

        # Floor grid
        for y in range(h // 2, h, 25):
            cv2.line(frame, (0, y), (w, y), (35, 45, 65), 1)

        self.sim_tick += 1
        t = self.sim_tick * 0.05
        
        # Horizontal movement
        pos_x = int(w / 2 + math.sin(t * 0.7) * 170)
        # Scale movement (distance changing 1.1m to 2.2m)
        scale = 0.95 + 0.35 * math.cos(t * 0.4)
        base_h = int(250 * scale)
        top_y = int(100 - (scale - 1.0) * 35)
        bot_y = top_y + base_h

        # Head
        head_radius = int(22 * scale)
        head_cy = top_y + head_radius
        cv2.circle(frame, (pos_x, head_cy), head_radius, (190, 220, 255), -1)
        cv2.circle(frame, (pos_x, head_cy), head_radius, (255, 255, 255), 2)

        # Torso
        torso_top = head_cy + head_radius
        torso_bot = torso_top + int(90 * scale)
        cv2.line(frame, (pos_x, torso_top), (pos_x, torso_bot), (150, 180, 220), int(16 * scale))

        # Shoulders & Arms
        shoulder_w = int(38 * scale)
        arm_swing = math.sin(t * 4) * 22 * scale
        cv2.line(frame, (pos_x - shoulder_w, torso_top + 10), 
                 (int(pos_x - shoulder_w - arm_swing), torso_bot), (120, 160, 200), int(8 * scale))
        cv2.line(frame, (pos_x + shoulder_w, torso_top + 10), 
                 (int(pos_x + shoulder_w + arm_swing), torso_bot), (120, 160, 200), int(8 * scale))

        # Legs
        leg_swing = math.sin(t * 4) * 30 * scale
        cv2.line(frame, (pos_x - 14, torso_bot), 
                 (int(pos_x - 20 + leg_swing), bot_y), (100, 140, 180), int(10 * scale))
        cv2.line(frame, (pos_x + 14, torso_bot), 
                 (int(pos_x + 20 - leg_swing), bot_y), (100, 140, 180), int(10 * scale))

        # Simulation badge
        cv2.putText(frame, "SIMULATION TEST FEED ACTIVE", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 180), 2)
        cv2.putText(frame, f"Simulated Distance: ~{round(1.8 / scale, 2)}m | Move X: {pos_x - w//2}px", (20, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        with self.lock:
            self.latest_frame = frame
            self.last_frame_time = time.time()
            self.is_connected = True
            self.status_message = "Simulation Mode Active"
            self.fps = Config.TARGET_FPS

    def stop(self):
        self.is_running = False
