"""
Comprehensive Computer Vision Pipeline for Human Following Robot:
Integrates:
1. Multi-Person Detection & Pose Estimation (MediaPipe + HOG)
2. Person Identification & Multi-Person Disambiguation (HSV Appearance Model)
3. Dynamic Following Distance Estimation & Tracking Confidence
4. Lost-Human Recovery Mechanism (Kalman Motion Projection & Re-Identification)
5. Gesture-Based Commands (FOLLOW_ME, STOP_HALT, RELEASE)
6. Visual Obstacle & Proximity Safety Layer
7. Real-Time Event Logging & Tactical HUD Overlay
"""

import math
import time
import os
import cv2
import numpy as np
import mediapipe as mp

from config import Config
from appearance_model import AppearanceModel
from kalman_tracker import PersonKalmanTracker
from gesture_recognizer import GestureRecognizer
from safety_monitor import SafetyMonitor
from robot_controller import RobotController
from obstacle_avoidance import ObstacleAvoidance
import lidar as lidar_lib
from ekf_person import PersonEKF
from follow_controller import FollowController

class CVPipeline:
    def __init__(self, model_path=None):
        self.model_path = model_path or Config.MODEL_PATH
        
        # Subsystems
        self.appearance = AppearanceModel(h_bins=18, s_bins=16)
        self.gestures = GestureRecognizer(debounce_frames=4)
        self.safety = SafetyMonitor(min_safe_dist=0.55, caution_dist=0.85)
        self.robot = RobotController()
        self.nav = ObstacleAvoidance()

        # Sensor fusion: camera pinhole + LiDAR range -> EKF -> PID
        self.ekf = PersonEKF()
        self.follow = FollowController()
        self.lidar_avoid = lidar_lib.LidarAvoidance()
        self.lidar_ranges = None
        self.lidar_time = 0.0
        self.vision_scale = 1.0       # LiDAR-calibrated correction of the camera distance
        self.fusion_mode = "CAM"
        self.last_ctl_t = None
        self.lidar_clusters = []
        self.plan = None
        self.plan_time = 0.0
        # identity gate
        self.own_score = 0.6          # running average of the owner's own appearance score while tracked
        self.reacq_count = 0
        self.reacq_x = None
        self.reid = {"state": "IDLE", "own": 0.0, "best": 0.0, "thr": 0.0, "reacq": 0, "confirm": Config.REACQ_CONFIRM_FRAMES}
        self._last_reid_log = 0.0
        
        # Target Tracking State
        self.target_id = None
        self.target_signature = None
        self.target_state = Config.STATE_IDLE
        self.following_enabled = True
        self.resume_counter = 0
        self.target_lost_time = 0.0
        self.max_recovery_seconds = Config.OCCLUSION_RECOVERY_SEC
        
        # Target Kalman filter
        self.target_kalman = None

        # Owner Profile & Biometric Recognition (Saved to Disk)
        self.owner_profile_file = Config.OWNER_PROFILE_PATH
        self.owner_profile = self.appearance.load_profile(self.owner_profile_file)
        self.owner_enrolled = (self.owner_profile is not None)
        self.owner_match_score = 1.0 if self.owner_enrolled else 0.0
        self.last_candidates = []
        if self.owner_enrolled:
            self.target_state = Config.STATE_OWNER_LOCKED
            print("[CVPipeline] Loaded designated Owner biometric profile.")
        
        # Dynamic distance parameters
        self.base_target_distance = Config.DESIRED_DISTANCE
        self.dynamic_target_distance = self.base_target_distance
        self.distance_calibration = 210.0
        
        # Smoothing filters for target
        self.smooth_distance = None
        self.smooth_offset_x = None

        # Search & target loss tracking
        self.last_exit_direction = 0      # -1: left, +1: right
        self.last_exit_offset_x = 0.0
        self.target_lost_time = None

        # Event Log (keeps last 50 events)
        self.event_log = []
        self._log_event("SYSTEM_INIT", "CV Pipeline initialized with Multi-Person Disambiguation & Re-ID")

        # Initialize MediaPipe PoseLandmarker and OpenCV HOG
        self.landmarker = None
        self.hog = None
        self._pose_ts_ms = 0
        self._init_detectors()

        # Landmark skeleton connections
        self.POSE_CONNECTIONS = [
            (11, 12), (11, 23), (12, 24), (23, 24), # Torso
            (11, 13), (13, 15), # Left arm
            (12, 14), (14, 16), # Right arm
            (23, 25), (25, 27), # Left leg
            (24, 26), (26, 28), # Right leg
            (0, 1), (1, 2), (2, 3), (3, 7), # Face L
            (0, 4), (4, 5), (5, 6), (6, 8)  # Face R
        ]

    def _init_detectors(self):
        # MediaPipe
        try:
            if os.path.exists(self.model_path):
                BaseOptions = mp.tasks.BaseOptions
                PoseLandmarker = mp.tasks.vision.PoseLandmarker
                PoseLandmarkerOptions = mp.tasks.vision.PoseLandmarkerOptions
                VisionRunningMode = mp.tasks.vision.RunningMode

                options = PoseLandmarkerOptions(
                    base_options=BaseOptions(model_asset_path=self.model_path),
                    running_mode=VisionRunningMode.VIDEO,
                    num_poses=1,
                    min_pose_detection_confidence=0.5,
                    min_pose_presence_confidence=0.5,
                    min_tracking_confidence=0.5
                )
                self.landmarker = PoseLandmarker.create_from_options(options)
                print("[CVPipeline] MediaPipe Pose ready.")
        except Exception as e:
            print(f"[CVPipeline] MediaPipe error: {e}")

        # HOG for multi-person detection
        try:
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            print("[CVPipeline] OpenCV HOG multi-person detector ready.")
        except Exception as e:
            print(f"[CVPipeline] HOG error: {e}")

    def _log_event(self, event_type, message):
        """Appends a timestamped structured event to the log"""
        timestamp = time.strftime("%H:%M:%S")
        entry = {
            "time": timestamp,
            "type": event_type,
            "message": message
        }
        self.event_log.append(entry)
        if len(self.event_log) > 50:
            self.event_log.pop(0)

    def lock_target(self, person_id, signature, bbox):
        """Explicitly locks onto a designated individual"""
        self.target_id = person_id
        self.target_signature = signature
        self.target_state = "TRACKING"
        self.target_kalman = PersonKalmanTracker(bbox)
        self.target_lost_time = 0.0
        hex_code = signature.get("color_hex", "#ffffff") if signature else "#ffffff"
        self._log_event("TARGET_LOCKED", f"Designated Target locked: Person ID #{person_id} (Color: {hex_code})")

    def unlock_target(self):
        """Releases the designated target"""
        prev_id = self.target_id
        self.target_id = None
        self.target_signature = None
        self.target_state = "SEARCHING"
        self.target_kalman = None
        self.smooth_distance = None
        self.smooth_offset_x = None
        self._log_event("TARGET_RELEASED", f"Target ID #{prev_id} released. System in SEARCHING mode.")

    def enroll_owner(self, cand=None):
        """Enrolls the specified or primary candidate as the exclusive Owner to recognize & follow"""
        if cand is None:
            if hasattr(self, "last_candidates") and len(self.last_candidates) > 0:
                cand = self.last_candidates[0]
            else:
                return False

        if not cand or cand.get("signature") is None:
            return False

        # Create multi-template owner profile with immutable protected baseline
        self.owner_profile = self.appearance.create_owner_profile(cand["signature"], owner_id=cand.get("id", 1))
        self.owner_enrolled = True
        self.following_enabled = True
        self.owner_match_score = 1.0
        self.own_score = 0.7
        self.ekf.reset()
        self.appearance.save_profile(self.owner_profile, self.owner_profile_file)
        self.lock_target(cand["id"], self.owner_profile, cand["bbox"])
        cand["is_owner"] = True
        cand["is_target"] = True
        self.target_state = Config.STATE_OWNER_LOCKED
        self._log_event("OWNER_LOCKED", f"User #{cand['id']} locked as OWNER! Multi-template identity initialized.")
        return True

    def release_owner(self):
        """Explicitly releases the locked owner and halts robot"""
        self.owner_profile = None
        self.owner_enrolled = False
        self.following_enabled = False
        self.owner_match_score = 0.0
        self.target_id = None
        self.target_signature = None
        self.target_kalman = None
        self.target_state = Config.STATE_OWNER_RELEASED
        self.robot.update_auto_commands(0.0, 0.0, can_move=False)
        if os.path.exists(self.owner_profile_file):
            try:
                os.remove(self.owner_profile_file)
            except Exception:
                pass
        self._log_event("OWNER_RELEASED", "Owner released by explicit command. Robot stopped in IDLE state.")
        return True

    def clear_owner(self):
        """Alias for release_owner"""
        return self.release_owner()

    def start_following(self):
        """Starts / resumes autonomous following of the locked owner"""
        self.following_enabled = True
        if self.owner_enrolled:
            self.target_state = Config.STATE_FOLLOWING
        self._log_event("FOLLOWING_STARTED", "Autonomous following enabled by operator.")
        return True

    def pause_following(self):
        """Pauses following while keeping owner locked"""
        self.following_enabled = False
        if self.owner_enrolled:
            self.target_state = Config.STATE_WAIT_FOR_OWNER_MOVEMENT
        self.robot.update_auto_commands(0.0, 0.0, can_move=False)
        self._log_event("FOLLOWING_PAUSED", "Autonomous following paused. Robot stationary.")
        return True

    def set_lidar(self, bins_mm):
        self.lidar_ranges = lidar_lib.to_meters(bins_mm)
        self.lidar_clusters = lidar_lib.segment(self.lidar_ranges)
        self.lidar_time = time.time()

    def set_plan(self, plan):
        self.plan = plan
        self.plan_time = time.time()

    def _accept_owner(self, cand, total, n_cands, now, w):
        """Identity gate. Returns True only if this candidate may be treated as the owner.

        While tracking: appearance must stay near what the owner normally scores and the candidate must be
        where the EKF expects them. After any loss (left the view, occluded, or the program just started):
        the appearance score must clear a higher bar for REACQ_CONFIRM_FRAMES consecutive frames at a
        consistent place, so a stranger who walks in as the owner walks out is rejected."""
        app = cand.get("similarity_to_owner", 0.0)
        lost = (self.target_lost_time is not None
                or self.target_state in (Config.STATE_SEARCHING, Config.STATE_TARGET_TEMPORARILY_LOST)
                or not self.ekf.active or self.ekf.misses > 8)
        f_px = (w / 2.0) / math.tan(math.radians(Config.CAMERA_HFOV_DEG) / 2.0)
        bearing = -math.atan((cand["centroid"][0] - w / 2.0) / f_px)
        if not lost:
            thr = max(Config.IDENTITY_MATCH_THRESHOLD, Config.TRACK_RELATIVE * self.own_score)
            # a person cannot jump: the candidate must be within a few sigma of where the EKF has them
            px, py = self.ekf.s[0], self.ekf.s[1]
            r2 = max(px * px + py * py, 0.04)
            sigma_b = math.sqrt(max(0.0, (px * px * self.ekf.P[1, 1] + py * py * self.ekf.P[0, 0]) / (r2 * r2)))
            gate = max(math.radians(12.0), 4.0 * sigma_b)
            in_gate = abs(bearing - self.ekf.range_bearing()[1]) < gate
            ok = app >= thr and in_gate
            if ok:
                self.own_score = 0.9 * self.own_score + 0.1 * app
            self.reid = {"state": "TRACKING" if ok else "REJECTED", "own": self.own_score, "best": app, "thr": thr,
                         "reacq": 0, "confirm": Config.REACQ_CONFIRM_FRAMES}
            self.reacq_count, self.reacq_x = 0, None
            return ok

        thr = max(Config.REACQ_THRESHOLD, Config.REACQ_RELATIVE * self.own_score)
        x = cand["centroid"][0]
        if app >= thr:
            if self.reacq_x is not None and abs(x - self.reacq_x) > 0.25 * w:
                self.reacq_count = 0
            self.reacq_count += 1
            self.reacq_x = x
        else:
            self.reacq_count, self.reacq_x = 0, None
        ok = self.reacq_count >= Config.REACQ_CONFIRM_FRAMES or app >= 0.85  # near-identical to the enrolment view
        self.reid = {"state": "REACQUIRED" if ok else "REACQUIRING", "own": self.own_score, "best": app, "thr": thr,
                     "reacq": self.reacq_count, "confirm": Config.REACQ_CONFIRM_FRAMES}
        if not ok and app < thr and now - self._last_reid_log > 3.0:
            self._last_reid_log = now
            self._log_event("REID_REJECTED", f"Someone else is in view (match {app:.2f} < {thr:.2f}). Not the owner; ignoring them.")
        if ok:
            self.reacq_count, self.reacq_x = 0, None
        return ok

    def _lidar_fresh(self):
        return (Config.LIDAR_FUSION and self.lidar_ranges is not None
                and time.time() - self.lidar_time < Config.LIDAR_STALE_SEC)

    def _fuse(self, cand, w, now):
        """Camera pinhole bearing + range, cross-checked with the LiDAR, filtered by the EKF.
        Returns (range_m, bearing_rad). Bearing is positive to the left."""
        f_px = (w / 2.0) / math.tan(math.radians(Config.CAMERA_HFOV_DEG) / 2.0)
        cx = cand["centroid"][0]
        bearing_cam = -math.atan((cx - w / 2.0) / f_px)
        vis_d = cand["distance_m"] * self.vision_scale
        v_r, w_r = self.robot.current_linear, self.robot.current_angular

        lidar_r = lidar_b = None
        if self._lidar_fresh():
            near = self.ekf.range_bearing()[0] if self.ekf.active else vis_d
            pc = lidar_lib.person_cluster(self.lidar_clusters, bearing_cam, near)
            if pc is not None:
                lidar_r, lidar_b = pc["range"], pc["bearing"]
                if abs(lidar_r - vis_d) > Config.LIDAR_AGREE_FRAC * max(vis_d, lidar_r) and not self.ekf.active:
                    lidar_r = None
        self.fusion_mode = "LIDAR+CAM" if lidar_r is not None else "CAM"

        if not self.ekf.active or self.ekf.rejects >= 6:
            self.ekf.init(lidar_r if lidar_r is not None else vis_d, bearing_cam, now)
            self.follow.reset()
        else:
            self.ekf.predict(now, v_r, w_r)
            self.ekf.update(vis_d, bearing_cam, 0.12 * vis_d, math.radians(1.5))
            if lidar_r is not None:
                if self.ekf.update(lidar_r, lidar_b, 0.05, math.radians(1.0)):
                    # the LiDAR is the better ruler: slowly correct the camera's size-based distance
                    ratio = lidar_r / max(cand["distance_m"], 0.1)
                    self.vision_scale = max(0.6, min(1.6, 0.98 * self.vision_scale + 0.02 * ratio))
        r, b = self.ekf.range_bearing()
        return r, b, f_px

    def _follow_command(self, cand, bearing, rng, f_px, dist_for_dt, obs_dists, now):
        """EKF-fed PID, then the LiDAR (or camera-corridor) avoidance layer."""
        dt = 0.05 if self.last_ctl_t is None else now - self.last_ctl_t
        self.last_ctl_t = now
        lead = Config.EKF_LEAD_SEC
        lx, ly = self.ekf.predicted(lead)
        b_ctl = math.atan2(ly, lx) if lx > 0.1 else bearing
        plan = self.plan if self.plan is not None and time.time() - self.plan_time < 0.8 else None
        detour = plan is not None and plan.get("blocked")
        if detour and plan.get("wp_bearing") is not None:
            b_ctl = plan["wp_bearing"]          # steer at the waypoint round the obstacle, not through it
        ux, uy = self.ekf.s[2], self.ekf.s[3]
        los = (self.ekf.s[0] * ux + self.ekf.s[1] * uy) / max(rng, 0.1)
        v, w_cmd = self.follow.command(rng, b_ctl, self.ekf.range_rate(),
                                       self.ekf.bearing_rate(self.robot.current_angular), los, dt,
                                       desired=self.base_target_distance)
        if self._lidar_fresh():
            x1, _, x2, _ = cand["bbox"]
            half = math.atan(0.5 * (x2 - x1) / f_px) + 0.06
            sec = lidar_lib.sectors(self.lidar_ranges, exclude=(bearing, half, rng))
            self.robot.set_front_clearance(sec["front_raw"], self.lidar_time)
            v, w_cmd, status = self.lidar_avoid.apply(v, w_cmd, sec)
            self._lidar_sectors = sec
            if detour and status == "LIDAR_CLEAR":
                status = "DETOUR" if plan.get("wp_bearing") is not None else "NO_PATH"
                if plan.get("wp_bearing") is None:
                    v = 0.0
            return v, w_cmd, status
        safe_w, scale, status = self.nav.compute_safe_steering(math.tan(-bearing) / math.tan(math.radians(Config.CAMERA_HFOV_DEG) / 2.0),
                                                               obstacle_distances=obs_dists)
        if status != "PATH_CLEAR":
            return v * scale, safe_w, status
        return v, w_cmd, status

    def process_frame(self, frame):
        """
        Main pipeline execution on a single video frame.
        Returns: (telemetry_dict, annotated_frame)
        """
        h, w = frame.shape[:2]
        if w > 480:
            new_h = max(1, int(round(h * (480.0 / w))))
            frame = cv2.resize(frame, (480, new_h), interpolation=cv2.INTER_LINEAR)
            h, w = frame.shape[:2]
        annotated = frame.copy()
        now = time.time()

        # Step 1: Detect all person candidates in the scene
        candidates = self._detect_all_persons(frame, annotated)
        self.last_candidates = candidates

        # Step 2: Pose & Gesture Analysis across detected candidates
        active_gesture, is_new_gesture = "NONE", False
        gesture_person = None
        for c in candidates:
            lm = c.get("landmarks")
            if lm is not None:
                g, is_new = self.gestures.analyze_pose(lm)
                if g != "NONE":
                    active_gesture = g
                    is_new_gesture = is_new
                    gesture_person = c
                    break

        # Handle Gesture commands (HRI - FollowMe Framework)
        manual_stop_signal = False
        if is_new_gesture:
            self._log_event("GESTURE_DETECTED", f"Recognized user gesture: '{active_gesture}'")
            if active_gesture == "FOLLOW_ME":
                # If no owner is locked, enroll the person performing the gesture
                if not self.owner_enrolled:
                    target_to_enroll = gesture_person if gesture_person is not None else (candidates[0] if len(candidates) > 0 else None)
                    if target_to_enroll is not None:
                        self.enroll_owner(target_to_enroll)
                else:
                    # Owner is already locked -> Start/resume following
                    self.start_following()
            elif active_gesture == "WAIT":
                # Pause following -> enter WAIT_FOR_OWNER_MOVEMENT
                self.pause_following()
            elif active_gesture == "RELEASE":
                # Explicit release
                self.release_owner()
            elif active_gesture == "STOP_HALT":
                manual_stop_signal = True
                self.target_state = Config.STATE_SAFETY_STOP
                self._log_event("EMERGENCY_STOP", "Robot halted by human STOP gesture command.")

        if active_gesture == "STOP_HALT":
            manual_stop_signal = True

        # Step 3: Multi-Person Disambiguation & Owner Recognition Matching
        matched_target = None
        if self.owner_enrolled and self.owner_profile is not None:
            cand_scores = []
            for cand in candidates:
                # Match against multi-template owner profile
                sim_owner = self.appearance.compute_owner_match(self.owner_profile, cand["signature"])
                
                # Spatial continuity if we were tracking near this position
                spatial_sim = 1.0
                if self.smooth_offset_x is not None:
                    cx = cand["centroid"][0]
                    cand_offset = (cx - (w / 2.0)) / (w / 2.0)
                    spatial_sim = max(0.0, 1.0 - abs(cand_offset - self.smooth_offset_x))

                if len(candidates) == 1:
                    total_score = max(sim_owner, 0.65 * sim_owner + 0.35 * spatial_sim)
                else:
                    total_score = 0.70 * sim_owner + 0.30 * spatial_sim

                cand["similarity_to_owner"] = round(sim_owner, 2)
                cand["similarity_to_target"] = round(total_score, 2)
                cand_scores.append(total_score)

            # Ambiguity check: if multiple candidates look almost identical, do NOT guess
            is_ambig, _ = self.appearance.check_ambiguity(
                cand_scores,
                match_thresh=Config.IDENTITY_MATCH_THRESHOLD,
                delta_thresh=Config.IDENTITY_AMBIGUITY_DELTA
            )

            if is_ambig and len(candidates) > 1:
                self.target_state = Config.STATE_TARGET_AMBIGUOUS
                matched_target = None
                self.owner_match_score = 0.0
                self._log_event("TARGET_AMBIGUOUS", "Ambiguous identity match detected among candidates. Robot stopped safely.")
            else:
                best_score = -1.0
                best_cand = None
                for cand in candidates:
                    if cand["similarity_to_target"] > best_score:
                        best_score = cand["similarity_to_target"]
                        best_cand = cand

                if best_cand is not None and self._accept_owner(best_cand, best_score, len(candidates), now, w):
                    if self.target_state in [Config.STATE_SEARCHING, Config.STATE_TARGET_TEMPORARILY_LOST]:
                        self._log_event("TARGET_REACQUIRED", f"Owner re-acquired! Re-ID confidence: {best_score:.2f}")
                    self.target_lost_time = None

                    matched_target = best_cand
                    matched_target["is_target"] = True
                    matched_target["is_owner"] = True
                    self.target_id = matched_target["id"]
                    self.owner_match_score = round(best_score, 2)

                    # Add new template view gently if match is high confidence
                    self.appearance.add_owner_template(
                        self.owner_profile,
                        matched_target["signature"],
                        best_score,
                        min_conf=Config.TEMPLATE_UPDATE_MIN_CONF,
                        max_templates=Config.MAX_OWNER_TEMPLATES
                    )

                    # Mark all other candidates as bystanders
                    for c in candidates:
                        if c != matched_target:
                            c["is_target"] = False
                            c["is_owner"] = False

                    # Update Kalman Filter
                    if self.target_kalman is not None:
                        corr_bbox, (vx, vy) = self.target_kalman.update(matched_target["bbox"])
                        matched_target["velocity"] = (vx, vy)
                    else:
                        self.target_kalman = PersonKalmanTracker(matched_target["bbox"])
                else:
                    # Bystanders in frame, but owner is not matched
                    for c in candidates:
                        c["is_target"] = False
                        c["is_owner"] = False
                    matched_target = None
                    self.owner_match_score = 0.0

                    if self.target_state in [Config.STATE_FOLLOWING, Config.STATE_WAIT_FOR_OWNER_MOVEMENT]:
                        self.target_state = Config.STATE_TARGET_TEMPORARILY_LOST
                        self.target_lost_time = now
                        self._log_event("TARGET_LOST", "Owner occluded or out of view. Robot stopped; holding position.")
        else:
            # No owner enrolled
            if self.target_state not in [Config.STATE_OWNER_RELEASED, Config.STATE_MANUAL_CONTROL, Config.STATE_SAFETY_STOP]:
                self.target_state = Config.STATE_IDLE
            if len(candidates) > 0:
                # Potential candidate ready for enrollment
                matched_target = candidates[0]

        # Step 4: Lost-Human Trajectory Projection (Conservative stop, no blind rotation)
        projected_bbox = None
        if self.target_state == Config.STATE_TARGET_TEMPORARILY_LOST:
            if self.target_lost_time is None:
                self.target_lost_time = now
            occlusion_duration = now - self.target_lost_time
            if occlusion_duration <= self.max_recovery_seconds and self.target_kalman is not None:
                projected_bbox, (vx, vy) = self.target_kalman.predict()
                self._draw_projected_search_box(annotated, projected_bbox, occlusion_duration)

        # Step 5: Distance & Following Control (State Machine Evaluation)
        active_color = "#00f0ff"
        if self.owner_enrolled and self.owner_profile:
            active_color = self.owner_profile.get("color_hex", "#00f0ff")

        telemetry = {
            "target_state": self.target_state,
            "target_id": self.target_id,
            "detected": (matched_target is not None),
            "distance_m": 0.0,
            "dynamic_target_distance": Config.DESIRED_DISTANCE,
            "offset_deg": 0.0,
            "offset_x_norm": 0.0,
            "active_gesture": active_gesture,
            "target_color": active_color,
            "owner_enrolled": self.owner_enrolled,
            "is_owner_matched": (matched_target is not None and matched_target.get("is_owner", False)),
            "owner_match_score": getattr(self, "owner_match_score", 0.0),
            "people_count": len(candidates),
            "safety_status": "SAFE",
            "safety_alerts": [],
            "nav_status": "CLEAR",
            "phase2_cmd_vel": {"linear_x": 0.0, "angular_z": 0.0},
            "robot_telemetry": self.robot.get_telemetry()
        }

        # Always scan floor-plane driving corridors once per frame (masking all detected humans)
        all_human_boxes = [c["bbox"] for c in candidates] if len(candidates) > 0 else (
            [matched_target["bbox"]] if matched_target is not None else None
        )
        obs_dists, corridor_boxes = self.nav.scan_visual_corridors(frame, human_bboxes=all_human_boxes)
        telemetry["obstacle_distances"] = obs_dists

        v_x, w_z = 0.0, 0.0
        if matched_target is not None:
            fused_r, fused_b, f_px = self._fuse(matched_target, w, now)
            tan_half = math.tan(math.radians(Config.CAMERA_HFOV_DEG) / 2.0)
            offset_x_norm = math.tan(-fused_b) / tan_half   # positive = person on the right of the image

            # Record last known offset & direction for search re-acquisition
            self.last_exit_offset_x = offset_x_norm
            self.last_exit_direction = 1 if offset_x_norm >= 0 else -1
            self.target_lost_time = None

            self.smooth_distance = fused_r
            self.smooth_offset_x = offset_x_norm

            telemetry["distance_m"] = round(float(fused_r), 2)
            telemetry["offset_x_norm"] = round(float(offset_x_norm), 3)
            telemetry["offset_deg"] = round(math.degrees(-fused_b), 1)

            dist = telemetry["distance_m"]
            offset_x = telemetry["offset_x_norm"]

            # Evaluate State Machine Transitions
            if not self.owner_enrolled:
                # Idle state
                v_x, w_z = 0.0, 0.0
            elif not self.following_enabled:
                # Paused / locked state
                self.target_state = Config.STATE_WAIT_FOR_OWNER_MOVEMENT
                v_x, w_z = 0.0, 0.0
            else:
                # Following is enabled for enrolled owner
                # Required transition: FOLLOWING -> WAIT_FOR_OWNER_MOVEMENT
                if dist <= Config.STOP_DISTANCE:
                    self.target_state = Config.STATE_WAIT_FOR_OWNER_MOVEMENT
                    self.resume_counter = 0
                    v_x = 0.0
                    # Gentle camera re-centering if owner steps laterally while standing near
                    if abs(offset_x) > Config.STEERING_DEADZONE:
                        w_z = max(-0.35, min(0.35, -0.70 * offset_x))
                    else:
                        w_z = 0.0
                elif dist >= Config.RESUME_DISTANCE or self.target_state == Config.STATE_FOLLOWING:
                    if self.target_state == Config.STATE_WAIT_FOR_OWNER_MOVEMENT:
                        self.resume_counter += 1
                        if self.resume_counter >= Config.RESUME_HYSTERESIS_FRAMES:
                            self.target_state = Config.STATE_FOLLOWING
                            self._log_event("STATE_RESUME", f"Owner moved away ({dist:.2f}m >= {Config.RESUME_DISTANCE}m). Resuming FOLLOWING.")
                    else:
                        self.target_state = Config.STATE_FOLLOWING

                    if self.target_state == Config.STATE_FOLLOWING:
                        # EKF-fed PID, then LiDAR avoidance
                        v_x, w_z, nav_status = self._follow_command(matched_target, fused_b, fused_r, f_px, dist, obs_dists, now)
                        telemetry["nav_status"] = nav_status
                    else:
                        v_x = 0.0
                        w_z = 0.0
                else:
                    # In hysteresis band (1.10m - 1.35m)
                    if self.target_state == Config.STATE_WAIT_FOR_OWNER_MOVEMENT:
                        v_x = 0.0
                        if abs(offset_x) > Config.STEERING_DEADZONE:
                            w_z = max(-0.35, min(0.35, -0.70 * offset_x))
                        else:
                            w_z = 0.0
                    else:
                        # EKF-fed PID, then LiDAR avoidance
                        v_x, w_z, nav_status = self._follow_command(matched_target, fused_b, fused_r, f_px, dist, obs_dists, now)
                        telemetry["nav_status"] = nav_status

            telemetry["phase2_cmd_vel"] = {"linear_x": round(v_x, 2), "angular_z": round(w_z, 2)}
        else:
            # No owner target matched in frame (owner exited frame or is occluded)
            self.follow.reset()
            self.last_ctl_t = None
            if self.ekf.active:
                self.ekf.predict(now, self.robot.current_linear, self.robot.current_angular)
                if self.ekf.misses > 40 or (self.target_lost_time is not None and now - self.target_lost_time > self.max_recovery_seconds):
                    self.ekf.reset()
                else:
                    self.last_exit_direction = 1 if self.ekf.s[1] < 0 else -1  # predicted to be on the right
            if self.owner_enrolled and self.following_enabled:
                if self.target_lost_time is None:
                    self.target_lost_time = now
                    self.target_state = Config.STATE_TARGET_TEMPORARILY_LOST
                    dir_str = "RIGHT" if self.last_exit_direction >= 0 else "LEFT"
                    self._log_event("TARGET_LOST", f"Owner exited frame towards {dir_str}. Initiating identity-locked search.")

                elapsed_lost = now - self.target_lost_time
                if elapsed_lost < Config.SEARCH_START_DELAY_SEC:
                    # Initial hesitation / Kalman projection (stay stopped)
                    self.target_state = Config.STATE_TARGET_TEMPORARILY_LOST
                    v_x, w_z = 0.0, 0.0
                    telemetry["nav_status"] = "HOLD_POSITION"
                elif elapsed_lost < (Config.SEARCH_START_DELAY_SEC + Config.SEARCH_MAX_DURATION_SEC):
                    # Active Identity-Locked Search Rotation!
                    self.target_state = Config.STATE_SEARCHING
                    v_x = 0.0
                    # If owner exited to the right -> Rotate right (clockwise: w_z < 0)
                    # If owner exited to the left -> Rotate left (counter-clockwise: w_z > 0)
                    search_w = -Config.SEARCH_ROTATION_SPEED if self.last_exit_direction >= 0 else Config.SEARCH_ROTATION_SPEED
                    side_check = "right" if self.last_exit_direction >= 0 else "left"

                    # Obstacle check during search rotation
                    if obs_dists.get(side_check, 3.0) < 0.45 or obs_dists.get("front", 3.0) < 0.35:
                        w_z = 0.0 # Pause rotation to prevent obstacle collision
                        telemetry["nav_status"] = f"SEARCH_BLOCKED_{side_check.upper()}"
                    else:
                        w_z = search_w
                        telemetry["nav_status"] = f"SEARCHING_{side_check.upper()}"
                else:
                    # Search sweep timeout: halt and hold position safely
                    self.target_state = Config.STATE_TARGET_TEMPORARILY_LOST
                    v_x, w_z = 0.0, 0.0
                    telemetry["nav_status"] = "SEARCH_TIMEOUT_HALT"
            else:
                v_x, w_z = 0.0, 0.0
                telemetry["nav_status"] = "IDLE"

            telemetry["phase2_cmd_vel"] = {"linear_x": round(v_x, 2), "angular_z": round(w_z, 2)}

        # Step 6: Safety Layer Evaluation
        safety_eval = self.safety.evaluate_safety(
            frame,
            telemetry["distance_m"],
            telemetry["target_state"],
            manual_stop_gesture=manual_stop_signal
        )
        telemetry["safety_status"] = safety_eval["status"]
        telemetry["safety_alerts"] = safety_eval["alerts"]

        # Forward verified velocity commands to physical robot motors
        if matched_target is not None:
            self.robot.update_auto_commands(v_x, w_z, can_move=safety_eval["can_move"])
        elif self.target_state == Config.STATE_SEARCHING and w_z != 0.0:
            self.robot.update_auto_commands(0.0, w_z, can_move=safety_eval["can_move"])
        else:
            self.robot.update_auto_commands(0.0, 0.0, can_move=False)

        telemetry["robot_telemetry"] = self.robot.get_telemetry()
        telemetry["person"] = self.ekf.as_dict() if self.ekf.active else None
        telemetry["reid"] = dict(self.reid) if self.owner_enrolled else None
        telemetry["fusion"] = {"mode": self.fusion_mode if matched_target is not None else "LOST",
                               "vision_scale": round(self.vision_scale, 3),
                               "lidar": self._lidar_fresh(),
                               "sectors": getattr(self, "_lidar_sectors", None) if self._lidar_fresh() else None}

        overlay = []
        for cand in candidates:
            x1, y1, x2, y2 = cand["bbox"]
            overlay.append({
                "bbox": [
                    round(x1 / float(w), 4),
                    round(y1 / float(h), 4),
                    round(x2 / float(w), 4),
                    round(y2 / float(h), 4),
                ],
                "target": bool(cand.get("is_target") or cand.get("is_owner")),
                "pose": cand.get("pose_norm") or [],
            })
        telemetry["overlay"] = overlay

        # Step 7: Render Tactical HUD with Obstacle Corridors
        self._render_tactical_hud(annotated, candidates, matched_target, telemetry, w, h, corridor_boxes)

        return telemetry, annotated

    def _detect_all_persons(self, frame, annotated):
        """Detects all people in frame, extracts appearance signatures and distances"""
        h, w = frame.shape[:2]
        candidates = []
        person_counter = 1

        # 1. MediaPipe Pose (High-precision single pose with 33 landmarks)
        if self.landmarker is not None:
            try:
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
                self._pose_ts_ms += 33
                res = self.landmarker.detect_for_video(mp_image, self._pose_ts_ms)

                if res.pose_landmarks and len(res.pose_landmarks) > 0:
                    for lm in res.pose_landmarks:
                        xs, ys, pts = [], [], {}
                        for idx, pt in enumerate(lm):
                            px, py = int(pt.x * w), int(pt.y * h)
                            pts[idx] = (px, py, pt.visibility)
                            if pt.visibility > 0.35:
                                xs.append(px)
                                ys.append(py)

                        # Strict Anatomical Human Validation (Rejects chairs, furniture, clutter)
                        if self._is_valid_human_pose(pts, w, h):
                            # Draw skeleton
                            for p1, p2 in self.POSE_CONNECTIONS:
                                if p1 in pts and p2 in pts:
                                    if pts[p1][2] > 0.35 and pts[p2][2] > 0.35:
                                        cv2.line(annotated, (pts[p1][0], pts[p1][1]), (pts[p2][0], pts[p2][1]), (0, 240, 255), 2)

                            if xs and ys:
                                x1 = max(0, min(xs) - 20)
                                x2 = min(w, max(xs) + 20)
                                y1 = max(0, min(ys) - 20)
                                y2 = min(h, max(ys) + 20)
                            else:
                                x1, y1, x2, y2 = w//4, h//4, 3*w//4, 3*h//4

                            bbox = [x1, y1, x2, y2]
                            cx = (x1 + x2) // 2
                            cy = (y1 + y2) // 2
                            sig = self.appearance.extract_signature(frame, bbox, pts)

                            # Robust multi-anchor distance estimation (shoulder span, pelvic width, torso height)
                            sh_l, sh_r = pts.get(11), pts.get(12)
                            hip_l, hip_r = pts.get(23), pts.get(24)

                            dist_estimates = []
                            if sh_l and sh_r and sh_l[2] > 0.35 and sh_r[2] > 0.35:
                                sh_w = math.hypot(sh_r[0] - sh_l[0], sh_r[1] - sh_l[1])
                                if sh_w > 15:
                                    dist_estimates.append((235.0 / sh_w) * (w / 640.0))

                            if hip_l and hip_r and hip_l[2] > 0.35 and hip_r[2] > 0.35:
                                hip_w = math.hypot(hip_r[0] - hip_l[0], hip_r[1] - hip_l[1])
                                if hip_w > 15:
                                    # Pelvic bi-iliac width (~30cm) vs shoulder (~44cm)
                                    dist_estimates.append((160.0 / hip_w) * (w / 640.0))

                            if sh_l and sh_r and hip_l and hip_r:
                                th1 = abs(hip_l[1] - sh_l[1])
                                th2 = abs(hip_r[1] - sh_r[1])
                                torso_h = max(th1, th2)
                                if torso_h > 35:
                                    dist_estimates.append((230.0 / float(torso_h)) * (w / 640.0))

                            if dist_estimates:
                                dist_m = sum(dist_estimates) / len(dist_estimates)
                            else:
                                bbox_h = max(40, y2 - y1)
                                dist_m = (320.0 / float(bbox_h)) * (w / 640.0)

                            dist_m = max(0.35, min(4.50, dist_m))

                            pose_norm = [
                                [int(i), round(pts[i][0] / float(w), 4), round(pts[i][1] / float(h), 4), round(float(pts[i][2]), 3)]
                                for i in pts
                            ]

                            candidates.append({
                                "id": person_counter,
                                "bbox": bbox,
                                "centroid": (cx, cy),
                                "signature": sig,
                                "distance_m": round(dist_m, 2),
                                "landmarks": lm,
                                "pose_norm": pose_norm,
                                "is_target": False,
                                "confidence": 0.95
                            })
                            person_counter += 1
            except Exception as e:
                print(f"[Pose Candidate Error] {e}")

        return candidates

    def _is_valid_human_pose(self, pts, w, h):
        """
        Strict anatomical human verification: rejects chairs, coats, furniture, and clutter.
        Verifies:
        1. Shoulders or hips detected with anatomical integrity.
        2. Facial landmarks: if visible, must be above shoulders. If person is far (shoulders > 35% down), face must be present.
        3. If hips present, hips must be below shoulders (anatomically sound).
        4. Torso or leg structure biologically valid.
        5. At least 6 key body landmarks must be confident.
        """
        sh_l = pts.get(11) # Left shoulder
        sh_r = pts.get(12) # Right shoulder
        nose = pts.get(0)  # Nose
        hip_l = pts.get(23) # Left hip
        hip_r = pts.get(24) # Right hip
        knee_l = pts.get(25) # Left knee
        knee_r = pts.get(26) # Right knee

        # Must have at least 6 confident landmarks
        confident_count = sum(1 for p in pts.values() if p[2] > 0.35)
        if confident_count < 6:
            return False

        has_both_shoulders = bool(sh_l and sh_r and sh_l[2] > 0.35 and sh_r[2] > 0.35)
        has_both_hips = bool(hip_l and hip_r and hip_l[2] > 0.35 and hip_r[2] > 0.35)

        # Must have either both shoulders or both hips
        if not (has_both_shoulders or has_both_hips):
            return False

        # If shoulders are present
        if has_both_shoulders:
            avg_sh_y = (sh_l[1] + sh_r[1]) / 2.0
            sh_w = math.hypot(sh_r[0] - sh_l[0], sh_r[1] - sh_l[1])
            if sh_w < 20 or sh_w > (w * 0.95):
                return False

            has_face = any(pts.get(f, (0, 0, 0))[2] > 0.35 for f in [0, 1, 2, 3, 4, 5, 6])
            # If face is detected, nose MUST be above shoulders (upright orientation)
            if has_face and nose and nose[2] > 0.35:
                if nose[1] >= avg_sh_y:
                    return False
            elif avg_sh_y > (h * 0.35):
                # When person is far (>1.5m), shoulders are low, face MUST be present to rule out chairs
                if not has_face:
                    return False

            # If hips are also present, hips must be below shoulders
            if has_both_hips:
                avg_hip_y = (hip_l[1] + hip_r[1]) / 2.0
                if avg_hip_y <= avg_sh_y + 10:
                    return False

        # If shoulders are cut off at the top (close proximity) but hips are present:
        elif has_both_hips:
            avg_hip_y = (hip_l[1] + hip_r[1]) / 2.0
            # If knees are present, knees must be below hips
            if knee_l and knee_r and knee_l[2] > 0.35 and knee_r[2] > 0.35:
                avg_knee_y = (knee_l[1] + knee_r[1]) / 2.0
                if avg_knee_y <= avg_hip_y + 10:
                    return False

        return True

    def _draw_projected_search_box(self, img, bbox, duration):
        """Draws Kalman projected trajectory box during occlusion"""
        x1, y1, x2, y2 = bbox
        color = (0, 165, 255) # Orange
        # Dashed bounding box
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        cv2.putText(img, f"PROJECTING TARGET TRAJECTORY ({duration:.1f}s)",
                    (x1, max(20, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

    def _render_tactical_hud(self, img, candidates, target, tel, w, h, corridor_boxes=None):
        """Renders cyberpunk/robotics overlay on processed stream"""
        cx_mid = w // 2
        cy_mid = h // 2

        # Draw Visual Obstacle Driving Corridors
        if corridor_boxes:
            for (x1, y1, x2, y2, col, lbl) in corridor_boxes:
                # Corridor dividing lines
                cv2.line(img, (x1, y1), (x1, y2), col, 1)
                cv2.line(img, (x2, y1), (x2, y2), col, 1)
                cv2.putText(img, lbl, (x1 + 8, y2 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.36, col, 1)

        # Deadzone corridor
        dz_px = int((w / 2) * Config.ANGULAR_DEADZONE)
        cv2.line(img, (cx_mid - dz_px, 35), (cx_mid - dz_px, h - 35), (60, 70, 95), 1)
        cv2.line(img, (cx_mid + dz_px, 35), (cx_mid + dz_px, h - 35), (60, 70, 95), 1)

        # Draw all candidate people
        for c in candidates:
            x1, y1, x2, y2 = c["bbox"]
            is_tgt = (target is not None and c["id"] == target["id"])
            is_owner = c.get("is_owner", False)

            if is_tgt:
                if is_owner:
                    color = (0, 255, 120)  # Neon Emerald
                    match_pct = int(tel.get("owner_match_score", 1.0) * 100)
                    label = f"[★ OWNER] MATCH: {match_pct}% | {c['distance_m']:.2f}m"
                else:
                    color = (0, 230, 255)  # Cyan
                    label = f"[TARGET #{c['id']}] DIST: {c['distance_m']:.2f}m"

                thick = 2
                self._draw_target_brackets(img, x1, y1, x2, y2, color)

                # Vector line from camera center
                tcx, tcy = c["centroid"]
                cv2.line(img, (cx_mid, cy_mid), (tcx, tcy), (0, 255, 255), 2)
                cv2.circle(img, (tcx, tcy), 6, (0, 255, 255), -1)
            else:
                # Other people (Disambiguated bystanders / non-targets)
                color = (135, 140, 150)
                if tel.get("owner_enrolled"):
                    sim_str = f"Sim: {int(c.get('similarity_to_owner', 0.0) * 100)}%"
                    label = f"BYSTANDER #{c['id']} ({sim_str})"
                else:
                    sim_str = f"Sim: {c.get('similarity_to_target', 0.0):.2f}"
                    label = f"Person #{c['id']} ({sim_str})"
                thick = 1
                cv2.rectangle(img, (x1, y1), (x2, y2), color, thick)

            # Label tag
            tag_w = 230 if is_owner else 180
            cv2.rectangle(img, (x1, max(0, y1 - 22)), (x1 + tag_w, y1), (15, 20, 30), -1)
            cv2.putText(img, label, (x1 + 4, max(12, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.40, color, 1)

        # Top Status Banner
        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (w, 38), (10, 15, 25), -1)
        cv2.addWeighted(overlay, 0.75, img, 0.25, 0, img)

        if tel.get("owner_enrolled"):
            if tel.get("is_owner_matched"):
                state_color = (0, 255, 120)
                pct = int(tel.get("owner_match_score", 1.0) * 100)
                if tel['target_state'] == Config.STATE_WAIT_FOR_OWNER_MOVEMENT:
                    status_txt = f"★ OWNER NEARBY ({pct}%) | WAITING FOR OWNER TO MOVE"
                elif tel['target_state'] == Config.STATE_FOLLOWING:
                    status_txt = f"★ OWNER MATCHED ({pct}%) | FOLLOWING"
                else:
                    status_txt = f"★ OWNER MATCHED ({pct}%) | STATE: {tel['target_state']}"
            else:
                state_color = (0, 180, 255)
                if tel['target_state'] == Config.STATE_SEARCHING:
                    state_color = (0, 220, 255)
                    status_txt = f"★ OWNER OUT OF VIEW | SEARCH SWEEP ({tel.get('nav_status', 'ROTATING')})"
                elif tel['target_state'] == Config.STATE_TARGET_TEMPORARILY_LOST:
                    status_txt = f"★ OWNER LOST | HOLDING POSITION ({tel.get('nav_status', 'STANDBY')})"
                elif tel['target_state'] == Config.STATE_TARGET_AMBIGUOUS:
                    status_txt = f"⚠ TARGET AMBIGUOUS | MULTIPLE MATCHES (STOPPED)"
                else:
                    status_txt = f"★ OWNER MODE | STATE: {tel['target_state']}"
        else:
            state_color = (0, 220, 255)
            status_txt = f"IDLE: NO OWNER LOCKED | RAISE HAND TO ENROLL"

        cv2.putText(img, status_txt, (14, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.44, state_color, 2)

        if tel["active_gesture"] != "NONE":
            g_txt = f"GESTURE: {tel['active_gesture']}"
            cv2.putText(img, g_txt, (340, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 220), 2)

        # Safety Banner at bottom
        safety_col = (0, 255, 120) if tel["safety_status"] == "SAFE" else (0, 50, 255)
        cv2.rectangle(img, (14, h - 28), (w - 14, h - 6), (15, 20, 30), -1)
        safe_str = f"SAFETY LAYER: {tel['safety_status']} | FOLLOWING DIST: {tel['dynamic_target_distance']}m"
        cv2.putText(img, safe_str, (20, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.44, safety_col, 1)

    def _draw_target_brackets(self, img, x1, y1, x2, y2, color):
        """Tactical bracket targeting box"""
        bw = x2 - x1
        bh = y2 - y1
        l = min(26, int(min(bw, bh) * 0.25))
        t = 2

        cv2.line(img, (x1, y1), (x1 + l, y1), color, t)
        cv2.line(img, (x1, y1), (x1, y1 + l), color, t)
        cv2.line(img, (x2, y1), (x2 - l, y1), color, t)
        cv2.line(img, (x2, y1), (x2, y1 + l), color, t)
        cv2.line(img, (x1, y2), (x1 + l, y2), color, t)
        cv2.line(img, (x1, y2), (x1, y2 - l), color, t)
        cv2.line(img, (x2, y2), (x2 - l, y2), color, t)
        cv2.line(img, (x2, y2), (x2, y2 - l), color, t)
