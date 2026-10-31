"""
Pose Tracker Module for Human Following Robot (Phase 1):
Handles MediaPipe Pose detection (with OpenCV HOG backup),
33-landmark skeleton rendering, target centroid tracking,
horizontal alignment calculation, and distance estimation.
"""

import math
import time
import os
import cv2
import numpy as np
import mediapipe as mp
from config import Config

class PoseTracker:
    def __init__(self, model_path=None):
        self.model_path = model_path or Config.MODEL_PATH
        self.landmarker = None
        self.hog = None
        
        # Skeleton connections for MediaPipe Pose (33 landmarks)
        self.POSE_CONNECTIONS = [
            # Torso
            (11, 12),  # Left shoulder to Right shoulder
            (11, 23),  # Left shoulder to Left hip
            (12, 24),  # Right shoulder to Right hip
            (23, 24),  # Left hip to Right hip
            # Left arm
            (11, 13), (13, 15),
            # Right arm
            (12, 14), (14, 16),
            # Left leg
            (23, 25), (25, 27),
            # Right leg
            (24, 26), (26, 28),
            # Face
            (0, 1), (1, 2), (2, 3), (3, 7),  # Left eye & ear
            (0, 4), (4, 5), (5, 6), (6, 8),  # Right eye & ear
            (9, 10),                          # Mouth
        ]

        # Calibration factor for distance estimation (Focal_length * Real_height)
        # Default ~220 gives: 160px torso at 640x480 -> ~1.37m
        self.distance_calibration = 210.0
        
        # Smoothing filters (Exponential Moving Average)
        self.smooth_distance = None
        self.smooth_offset_x = None
        self.last_seen_time = 0
        self.target_locked = False
        
        self._init_mediapipe()
        self._init_hog()

    def _init_mediapipe(self):
        try:
            if not os.path.exists(self.model_path):
                print(f"[PoseTracker] Model file {self.model_path} not found.")
                return

            BaseOptions = mp.tasks.BaseOptions
            PoseLandmarker = mp.tasks.vision.PoseLandmarker
            PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
            VisionRunningMode = mp.tasks.vision.RunningMode

            options = PoseLandmarkerOptions(
                base_options=BaseOptions(model_asset_path=self.model_path),
                running_mode=VisionRunningMode.IMAGE,
                num_poses=1,
                min_pose_detection_confidence=0.5,
                min_pose_presence_confidence=0.5,
                min_tracking_confidence=0.5
            )
            self.landmarker = PoseLandmarker.create_from_options(options)
            print("[PoseTracker] MediaPipe PoseLandmarker initialized successfully.")
        except Exception as e:
            print(f"[PoseTracker] MediaPipe initialization error: {e}")
            self.landmarker = None

    def _init_hog(self):
        try:
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            print("[PoseTracker] OpenCV HOG Human Detector ready as auxiliary detector.")
        except Exception as e:
            print(f"[PoseTracker] HOG init error: {e}")

    def update_calibration(self, new_val):
        """Update distance estimation calibration constant"""
        self.distance_calibration = float(new_val)

    def process_frame(self, frame):
        """
        Processes a raw frame:
        - Runs MediaPipe Pose
        - Falls back to HOG if no landmarks
        - Calculates distance and horizontal offset
        - Draws rich skeletal visualization and HUD
        Returns: (telemetry_dict, annotated_bgr_frame)
        """
        h, w = frame.shape[:2]
        annotated = frame.copy()
        now = time.time()

        telemetry = {
            "detected": False,
            "method": "none",
            "centroid_x": w // 2,
            "centroid_y": h // 2,
            "offset_x_norm": 0.0,
            "offset_deg": 0.0,
            "distance_m": 0.0,
            "distance_status": "SEARCHING",
            "confidence": 0.0,
            "bbox": None,
            "phase2_cmd_vel": {"linear_x": 0.0, "angular_z": 0.0}
        }

        mp_detected = False

        # 1. MediaPipe Pose Detection
        if self.landmarker is not None:
            try:
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
                detection_result = self.landmarker.detect(mp_image)

                if detection_result.pose_landmarks and len(detection_result.pose_landmarks) > 0:
                    landmarks = detection_result.pose_landmarks[0]
                    mp_detected = True
                    telemetry = self._extract_mediapipe_data(landmarks, w, h, annotated)
            except Exception as e:
                pass

        # 2. Auxiliary HOG Detection (if MediaPipe did not catch)
        if not mp_detected and self.hog is not None:
            hog_data = self._extract_hog_data(frame, w, h, annotated)
            if hog_data["detected"]:
                telemetry = hog_data

        # 3. Temporal Smoothing (EMA filter for stable distance and offset)
        if telemetry["detected"]:
            self.last_seen_time = now
            self.target_locked = True
            raw_dist = telemetry["distance_m"]
            raw_off = telemetry["offset_x_norm"]

            if self.smooth_distance is None:
                self.smooth_distance = raw_dist
                self.smooth_offset_x = raw_off
            else:
                alpha = 0.30  # EMA factor: responsive yet smooth
                self.smooth_distance = alpha * raw_dist + (1.0 - alpha) * self.smooth_distance
                self.smooth_offset_x = alpha * raw_off + (1.0 - alpha) * self.smooth_offset_x

            telemetry["distance_m"] = round(float(self.smooth_distance), 2)
            telemetry["offset_x_norm"] = round(float(self.smooth_offset_x), 3)
            # Approximate camera half-horizontal FOV is ~32 degrees
            telemetry["offset_deg"] = round(telemetry["offset_x_norm"] * 32.0, 1)

            # Categorize distance
            if telemetry["distance_m"] < 0.7:
                telemetry["distance_status"] = "TOO CLOSE"
            elif telemetry["distance_m"] > 2.5:
                telemetry["distance_status"] = "FAR"
            elif abs(telemetry["distance_m"] - Config.TARGET_DISTANCE) <= Config.DISTANCE_DEADZONE:
                telemetry["distance_status"] = "OPTIMAL"
            else:
                telemetry["distance_status"] = "TRACKING"

            # Phase 2 preview calculations (for motor control simulation)
            dist_err = telemetry["distance_m"] - Config.TARGET_DISTANCE
            if abs(dist_err) < Config.DISTANCE_DEADZONE:
                v_x = 0.0
            else:
                v_x = max(-0.10, min(Config.MAX_LINEAR_VEL, dist_err * Config.KP_LINEAR))

            if abs(telemetry["offset_x_norm"]) < Config.ANGULAR_DEADZONE:
                w_z = 0.0
            else:
                w_z = max(-Config.MAX_ANGULAR_VEL, min(Config.MAX_ANGULAR_VEL, -telemetry["offset_x_norm"] * Config.KP_ANGULAR * 2.0))

            telemetry["phase2_cmd_vel"] = {
                "linear_x": round(v_x, 2),
                "angular_z": round(w_z, 2)
            }
        else:
            if now - self.last_seen_time > Config.TARGET_LOSS_TIMEOUT:
                self.smooth_distance = None
                self.smooth_offset_x = None
                self.target_locked = False
                telemetry["distance_status"] = "SEARCHING"

        # 4. Render Tactical Robotics HUD
        self._render_hud(annotated, telemetry, w, h)

        return telemetry, annotated

    def _extract_mediapipe_data(self, landmarks, w, h, annotated):
        pts = {}
        xs = []
        ys = []

        for i, lm in enumerate(landmarks):
            px = int(lm.x * w)
            py = int(lm.y * h)
            pts[i] = (px, py, lm.visibility)
            if lm.visibility > 0.3:
                xs.append(px)
                ys.append(py)

        # Draw Skeleton bones (glowing neon lines)
        for p1, p2 in self.POSE_CONNECTIONS:
            if p1 in pts and p2 in pts:
                v1, v2 = pts[p1][2], pts[p2][2]
                if v1 > 0.35 and v2 > 0.35:
                    pt1 = (pts[p1][0], pts[p1][1])
                    pt2 = (pts[p2][0], pts[p2][1])
                    # Outer cyan glow
                    cv2.line(annotated, pt1, pt2, (255, 200, 0), 3)
                    # Inner white core
                    cv2.line(annotated, pt1, pt2, (255, 255, 255), 1)

        # Draw Joints
        for i, (px, py, vis) in pts.items():
            if vis > 0.35:
                # Key joints get distinct styling
                if i in [11, 12, 23, 24]:  # Shoulders and hips
                    cv2.circle(annotated, (px, py), 6, (0, 255, 255), -1)
                    cv2.circle(annotated, (px, py), 9, (0, 200, 255), 2)
                elif i in [15, 16, 27, 28]:  # Wrists and ankles
                    cv2.circle(annotated, (px, py), 5, (0, 165, 255), -1)
                else:
                    cv2.circle(annotated, (px, py), 4, (0, 255, 180), -1)

        # Calculate Shoulders and Torso
        sh_l = pts.get(11)
        sh_r = pts.get(12)
        hip_l = pts.get(23)
        hip_r = pts.get(24)

        if sh_l and sh_r and sh_l[2] > 0.3 and sh_r[2] > 0.3:
            sh_mid_x = (sh_l[0] + sh_r[0]) // 2
            sh_mid_y = (sh_l[1] + sh_r[1]) // 2
            sh_dist_px = math.hypot(sh_r[0] - sh_l[0], sh_r[1] - sh_l[1])
        else:
            sh_mid_x, sh_mid_y, sh_dist_px = w // 2, h // 2, 80

        if hip_l and hip_r and hip_l[2] > 0.3 and hip_r[2] > 0.3:
            hip_mid_x = (hip_l[0] + hip_r[0]) // 2
            hip_mid_y = (hip_l[1] + hip_r[1]) // 2
            torso_h_px = abs(hip_mid_y - sh_mid_y)
        else:
            hip_mid_x, hip_mid_y = sh_mid_x, sh_mid_y + 110
            torso_h_px = 110

        # Body centroid is mid-torso
        cx = (sh_mid_x + hip_mid_x) // 2
        cy = (sh_mid_y + hip_mid_y) // 2

        # Bounding box
        if xs and ys:
            x_min = max(0, min(xs) - 25)
            x_max = min(w, max(xs) + 25)
            y_min = max(0, min(ys) - 25)
            y_max = min(h, max(ys) + 25)
        else:
            x_min, x_max = max(0, cx - 70), min(w, cx + 70)
            y_min, y_max = max(0, cy - 120), min(h, cy + 120)

        # Distance estimation:
        # Pinhole model: Distance = Calibration / Feature_Size_in_pixels
        # Using a weighted blend of torso height and shoulder width for high stability
        feature_size = max(25.0, (torso_h_px * 0.65 + sh_dist_px * 0.70))
        est_distance = self.distance_calibration / feature_size
        est_distance = max(0.40, min(4.50, est_distance))

        # Horizontal offset from image center (-1.0 to +1.0)
        offset_x_norm = (cx - (w / 2)) / (w / 2)

        # Draw tech corner brackets on bounding box
        self._draw_corner_brackets(annotated, x_min, y_min, x_max, y_max, (0, 255, 120), "HUMAN POSE")

        return {
            "detected": True,
            "method": "mediapipe",
            "centroid_x": cx,
            "centroid_y": cy,
            "offset_x_norm": offset_x_norm,
            "distance_m": round(est_distance, 2),
            "bbox": [x_min, y_min, x_max, y_max],
            "confidence": 0.94
        }

    def _extract_hog_data(self, frame, w, h, annotated):
        small = cv2.resize(frame, (320, 240))
        boxes, _ = self.hog.detectMultiScale(small, winStride=(8, 8), padding=(8, 8), scale=1.06)

        if len(boxes) > 0:
            best_box = max(boxes, key=lambda b: b[2] * b[3])
            bx, by, bw, bh = best_box
            scale_x, scale_y = w / 320.0, h / 240.0
            x1, y1 = int(bx * scale_x), int(by * scale_y)
            x2, y2 = int((bx + bw) * scale_x), int((by + bh) * scale_y)

            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            offset_x_norm = (cx - (w / 2)) / (w / 2)

            # Distance via full-body box height
            box_h = max(30, y2 - y1)
            est_distance = (self.distance_calibration * 2.2) / box_h
            est_distance = max(0.40, min(4.50, est_distance))

            self._draw_corner_brackets(annotated, x1, y1, x2, y2, (0, 200, 255), "HUMAN (HOG)")

            return {
                "detected": True,
                "method": "hog",
                "centroid_x": cx,
                "centroid_y": cy,
                "offset_x_norm": offset_x_norm,
                "distance_m": round(est_distance, 2),
                "bbox": [x1, y1, x2, y2],
                "confidence": 0.76
            }

        return {
            "detected": False,
            "method": "none",
            "centroid_x": w // 2,
            "centroid_y": h // 2,
            "offset_x_norm": 0.0,
            "distance_m": 0.0,
            "bbox": None,
            "confidence": 0.0
        }

    def _draw_corner_brackets(self, img, x1, y1, x2, y2, color, label):
        """Draws cyberpunk/tactical corner brackets around target"""
        bw = x2 - x1
        bh = y2 - y1
        line_len = min(24, int(min(bw, bh) * 0.22))
        thick = 2

        # Top-left
        cv2.line(img, (x1, y1), (x1 + line_len, y1), color, thick)
        cv2.line(img, (x1, y1), (x1, y1 + line_len), color, thick)
        # Top-right
        cv2.line(img, (x2, y1), (x2 - line_len, y1), color, thick)
        cv2.line(img, (x2, y1), (x2, y1 + line_len), color, thick)
        # Bottom-left
        cv2.line(img, (x1, y2), (x1 + line_len, y2), color, thick)
        cv2.line(img, (x1, y2), (x1, y2 - line_len), color, thick)
        # Bottom-right
        cv2.line(img, (x2, y2), (x2 - line_len, y2), color, thick)
        cv2.line(img, (x2, y2), (x2, y2 - line_len), color, thick)

        # Label tag
        cv2.rectangle(img, (x1, max(0, y1 - 22)), (x1 + 120, y1), (15, 20, 30), -1)
        cv2.putText(img, label, (x1 + 4, max(12, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)

    def _render_hud(self, img, tel, w, h):
        """Renders robotics HUD overlays, deadzone corridor, tracking vector, and distance meter"""
        cx_mid = w // 2
        cy_mid = h // 2

        # 1. Deadzone vertical corridor lines (reference guides for center alignment)
        dz_px = int((w / 2) * Config.ANGULAR_DEADZONE)
        cv2.line(img, (cx_mid - dz_px, 35), (cx_mid - dz_px, h - 35), (60, 70, 95), 1)
        cv2.line(img, (cx_mid + dz_px, 35), (cx_mid + dz_px, h - 35), (60, 70, 95), 1)
        
        # Center reference crosshair
        cv2.line(img, (cx_mid, cy_mid - 10), (cx_mid, cy_mid + 10), (100, 110, 140), 1)
        cv2.line(img, (cx_mid - 10, cy_mid), (cx_mid + 10, cy_mid), (100, 110, 140), 1)

        # 2. Tracking vector if target detected
        if tel["detected"]:
            tx = tel["centroid_x"]
            ty = tel["centroid_y"]

            # Tracking vector line from camera center to human center
            cv2.line(img, (cx_mid, cy_mid), (tx, ty), (0, 255, 255), 2)
            cv2.circle(img, (tx, ty), 6, (0, 255, 255), -1)
            cv2.circle(img, (tx, ty), 12, (0, 180, 255), 2)

            # Distance pill badge floating directly above human centroid
            badge_y = max(45, ty - 35)
            badge_text = f"DIST: {tel['distance_m']:.2f} m"
            (tw, th), _ = cv2.getTextSize(badge_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            cv2.rectangle(img, (tx - tw//2 - 6, badge_y - th - 6), (tx + tw//2 + 6, badge_y + 4), (15, 25, 40), -1)
            cv2.rectangle(img, (tx - tw//2 - 6, badge_y - th - 6), (tx + tw//2 + 6, badge_y + 4), (0, 255, 200), 1)
            cv2.putText(img, badge_text, (tx - tw//2, badge_y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 200), 1)

            # Steering horizon indicator at bottom
            off_norm = tel["offset_x_norm"]
            arrow_end_x = int(cx_mid + off_norm * (w // 2 - 30))
            color_turn = (0, 255, 150) if abs(off_norm) < Config.ANGULAR_DEADZONE else (0, 180, 255)
            cv2.arrowedLine(img, (cx_mid, h - 25), (arrow_end_x, h - 25), color_turn, 2, tipLength=0.25)

        # 3. Top translucent status bar
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (w, 36), (10, 15, 25), -1)
        cv2.addWeighted(overlay, 0.70, img, 0.30, 0, img)

        if tel["detected"]:
            status_txt = f"AI TARGET LOCKED [{tel['method'].upper()}]"
            status_col = (0, 255, 120)
            dist_str = f"EST DIST: {tel['distance_m']:.2f}m"
            angle_str = f"OFFSET: {tel['offset_deg']:+.1f} deg"
        else:
            status_txt = "SEARCHING FOR HUMAN..."
            status_col = (0, 180, 255)
            dist_str = "EST DIST: --"
            angle_str = "OFFSET: 0.0 deg"

        cv2.putText(img, status_txt, (14, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.52, status_col, 2)
        cv2.putText(img, dist_str, (270, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
        cv2.putText(img, angle_str, (470, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 240, 255), 1)
