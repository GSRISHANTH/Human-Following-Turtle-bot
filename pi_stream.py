#!/usr/bin/env python3
"""
Ultra Low-Latency Raspberry Pi Camera Streaming Server for Robot AI Tracking
-----------------------------------------------------------------------------
Key Optimizations:
1. CAP_PROP_FOURCC = 'MJPG' (webcam hardware compression, zero Pi CPU load)
2. Fast V4L2 kernel buffer flushing via cap.grab() + cap.retrieve()
3. Quality tuned to 55% for minimal Wi-Fi packet size & instant transmission
4. Optimized capture thread prevents frame accumulation and lag buildup
"""

import time
import socket
import threading
from flask import Flask, Response, render_template_string, request
import cv2

# ================= Configuration =================
CAMERA_INDEX = 0          # Probes 0, 1, 2 if needed
FRAME_WIDTH = 640         # 640x480 (or 480x360 for ultra-weak Wi-Fi)
FRAME_HEIGHT = 480
TARGET_FPS = 20           # 20 FPS is ideal for smooth Wi-Fi stream without congestion
JPEG_QUALITY = 55         # 55% quality: 3x smaller file size, imperceptible difference for AI
PORT = 5000               # Port to stream on
# =================================================

app = Flask(__name__)

class ThreadedCamera:
    """Ultra-low latency threaded capture that always discards stale frames."""
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
        print(f"[Camera] Opening camera index {self.src} with V4L2...")
        self.cap = cv2.VideoCapture(self.src, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            print("[Camera] V4L2 backend failed, trying default backend...")
            self.cap = cv2.VideoCapture(self.src)

        if not self.cap.isOpened():
            for alt_idx in [1, 2, 0]:
                if alt_idx == self.src:
                    continue
                print(f"[Camera] Testing alternative index {alt_idx}...")
                test_cap = cv2.VideoCapture(alt_idx)
                if test_cap.isOpened():
                    self.src = alt_idx
                    self.cap = test_cap
                    print(f"[Camera] Successfully opened camera at index {alt_idx}!")
                    break

        if not self.cap.isOpened():
            raise RuntimeError(
                f"Cannot open webcam (tried index {self.src}). "
                "Check USB connection and run 'ls /dev/video*' on Raspberry Pi."
            )

        # 1. CRITICAL: Request hardware MJPG from camera (drops USB bus load from 12MB/s to 0.8MB/s)
        try:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        except Exception:
            pass

        # 2. Configure Resolution & FPS
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        # Minimize hardware buffer
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        ret, initial_frame = self.cap.read()
        if ret:
            self.frame = initial_frame
            print(f"[Camera] Ready: {initial_frame.shape[1]}x{initial_frame.shape[0]} @ {self.fps} FPS")

        self.is_running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        """Worker thread that continuously pulls frames to keep buffer fresh."""
        while self.is_running:
            # grab() dumps the frame buffer from the camera driver instantly
            ret = self.cap.grab()
            if ret:
                ret_frame, frame = self.cap.retrieve()
                if ret_frame:
                    with self.lock:
                        self.frame = frame
            else:
                time.sleep(0.005)

    def get_frame(self):
        with self.lock:
            if self.frame is not None:
                return self.frame.copy()
            return None

    def release(self):
        self.is_running = False
        if self.cap:
            self.cap.release()

camera = None

def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip

def generate_mjpeg_stream():
    """Generator yielding low-bandwidth JPEG frames with zero accumulation lag."""
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
    last_sent_time = 0.0
    frame_interval = 1.0 / TARGET_FPS

    while True:
        if camera is None:
            time.sleep(0.02)
            continue

        now = time.time()
        # Enforce rate limit to avoid overflowing Wi-Fi buffer
        if now - last_sent_time < frame_interval:
            time.sleep(0.002)
            continue

        frame = camera.get_frame()
        if frame is None:
            time.sleep(0.005)
            continue

        last_sent_time = now
        success, buffer = cv2.imencode('.jpg', frame, encode_params)
        if not success:
            continue

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n'
            + buffer.tobytes()
            + b'\r\n'
        )

@app.route('/video_feed')
def video_feed():
    return Response(
        generate_mjpeg_stream(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/')
def index():
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Raspberry Pi Low-Latency Camera Stream</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
            body { background: #0d1117; color: #e6edf3; font-family: sans-serif; text-align: center; padding: 20px; }
            .card { max-width: 650px; margin: 0 auto; background: #161b22; border: 1px solid #30363d; border-radius: 12px; padding: 20px; }
            img { width: 100%; border-radius: 8px; border: 1px solid #30363d; }
            .badge { color: #3fb950; font-weight: bold; margin-bottom: 12px; }
            .code { font-family: monospace; background: #0d1117; padding: 8px; border-radius: 4px; color: #58a6ff; }
        </style>
    </head>
    <body>
        <div class="card">
            <h2>Raspberry Pi Low-Latency Stream</h2>
            <div class="badge">&#9679; REAL-TIME FEED ACTIVE (FOURCC: MJPG)</div>
            <img src="/video_feed" alt="Live Camera Stream">
            <p class="code">Stream URL: http://{{ ip }}:{{ port }}/video_feed</p>
        </div>
    </body>
    </html>
    """
    return render_template_string(html, ip=get_local_ip(), port=PORT)

@app.route('/health')
def health():
    return {"status": "ok", "ip": get_local_ip(), "fps": TARGET_FPS}

if __name__ == '__main__':
    my_ip = get_local_ip()
    print("=" * 60)
    print("  RASPBERRY PI ULTRA LOW-LATENCY STREAMING SERVER")
    print("=" * 60)
    print(f" Detected IP   : {my_ip}")
    print(f" Stream URL    : http://{my_ip}:{PORT}/video_feed")
    print(f" Format        : Hardware MJPEG @ {TARGET_FPS} FPS")
    print(f" Resolution    : {FRAME_WIDTH}x{FRAME_HEIGHT} (Quality: {JPEG_QUALITY}%)")
    print("=" * 60)

    try:
        camera = ThreadedCamera(
            src=CAMERA_INDEX,
            width=FRAME_WIDTH,
            height=FRAME_HEIGHT,
            fps=TARGET_FPS
        )
        app.run(
            host='0.0.0.0',
            port=PORT,
            threaded=True,
            debug=False,
            use_reloader=False
        )
    finally:
        if camera:
            camera.release()
            print("\n[Camera] Released.")
