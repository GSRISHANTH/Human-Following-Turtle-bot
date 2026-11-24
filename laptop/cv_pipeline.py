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
import fusion
from ekf_person import PersonEKF
from follow_controller import FollowController

class CVPipeline:
    def __init__(self, model_path=None):
        self.model_path = model_path or Config.MODEL_PATH
        
        # Subsystems
        self.appearance = AppearanceModel(h_bins=18, s_bins=16)
        self.gestures = GestureRecognizer(debounce_frames=Config.GESTURE_CONFIRM_FRAMES)
        self.any_stop = GestureRecognizer(debounce_frames=Config.GESTURE_CONFIRM_FRAMES)  # STOP_HALT from anyone
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
        self.cam_bearing_bias = 0.0   # camera yaw misalignment vs the LiDAR, estimated while both see the owner
        self.last_cam_seen = None
        self.lidar_only = False
        self.last_template_t = 0.0
        self.template_count = 0
        self._search_t = None
        self.search_scan_rot = 0.0
        self.search_dir = 1.0
        self.memory_done = False      # the drive to the last-seen position is over (arrived, blocked, or timed out)
        self.safety_latched = False   # STOP_HALT seen: stays stopped until FOLLOW_ME (owner) or START from the dashboard
        self.ambiguous = False        # two look-alikes in view: hold still, never guess and never search
        self.render_hud = True        # the dashboard draws in the browser; the worker turns the OpenCV HUD off
        self.lidar_cam = []           # LiDAR beams projected into the image, for the co-mapped overlay
        self.hand_raised = False      # hold-to-follow: owner's hand is up (debounced)
        self._backing = False         # backing off because the owner came closer than the follow distance
        self._raise_n = 0
        self._last_raise_t = 0.0
        self._raised_at_loss = False  # was the hand up when the camera lost the owner (then keep up by LiDAR)
        self.search_t0 = None         # when the scan search began; it starts after the memory pursuit, not at the loss
        
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
            # A profile from an earlier run is a convenience, not a command: never start driving on boot. With
            # hold-to-follow nothing moves until the owner raises a hand, so following can stay armed.
            self.following_enabled = bool(Config.FOLLOW_REQUIRES_RAISED_HAND)
            self.target_state = Config.STATE_OWNER_LOCKED
            print("[CVPipeline] Loaded designated Owner biometric profile.")
        
        # Dynamic distance parameters
        self.base_target_distance = Config.DESIRED_DISTANCE
        self.dynamic_target_distance = self.base_target_distance
        self._dist_extra = 0.0        # smoothed margin on top of the set distance (speed headway + low confidence)
        self.distance_calibration = Config.DISTANCE_SCALE   # kept for the dashboard; the value used is Config.DISTANCE_SCALE
        
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
        if self.owner_enrolled:
            self._log_event("PROFILE_LOADED", "Saved owner profile restored. Robot is stationary: raise a hand (FOLLOW ME) or press START.")

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
                    num_poses=max(1, int(Config.MAX_POSES)),
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
        self.target_state = Config.STATE_OWNER_LOCKED
        self.target_kalman = PersonKalmanTracker(bbox)
        self.target_lost_time = 0.0
        hex_code = signature.get("color_hex", "#ffffff") if signature else "#ffffff"
        self._log_event("TARGET_LOCKED", f"Designated Target locked: Person ID #{person_id} (Color: {hex_code})")

    def unlock_target(self):
        """Releases the designated target"""
        prev_id = self.target_id
        self.target_id = None
        self.target_signature = None
        self.target_state = Config.STATE_OWNER_LOCKED if self.owner_enrolled else Config.STATE_IDLE
        self.target_kalman = None
        self.smooth_distance = None
        self.smooth_offset_x = None
        self._log_event("TARGET_RELEASED", f"Target ID #{prev_id} released.")

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
        self.safety_latched = False
        self.ambiguous = False
        self.owner_match_score = 1.0
        self.own_score = 0.7
        self.last_cam_seen = time.time()
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
        self.ekf.reset()
        self.target_lost_time = None
        self.lidar_only = False
        self.target_state = Config.STATE_OWNER_RELEASED
        self._released_t = time.time()
        self.last_cam_seen = None
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
        """Starts / resumes autonomous following of the locked owner. Also the only way out of a STOP latch."""
        if self.safety_latched:
            self._log_event("SAFETY_CLEARED", "Stop latch cleared. Following may resume.")
        self.safety_latched = False
        self.following_enabled = True
        if self.robot.mode == "MANUAL" and not self.robot.emergency_stop:
            self.robot.set_mode("AUTO")
            self._log_event("MODE_AUTO", "Teleop mode left: autonomous following is driving again.")
        if self.owner_enrolled:
            self.target_state = Config.STATE_FOLLOWING
        self._log_event("FOLLOWING_STARTED", "Autonomous following enabled by operator.")
        return True

    def trigger_safety_stop(self, reason="Safety stop."):
        """Latched stop: following is disabled until start_following() (FOLLOW_ME by the owner, or START)."""
        if not self.safety_latched:
            self._log_event("EMERGENCY_STOP", reason)
        self.safety_latched = True
        self.following_enabled = False
        self.target_state = Config.STATE_SAFETY_STOP
        self.robot.update_auto_commands(0.0, 0.0, can_move=False)
        return True

    def pause_following(self):
        """Pauses following while keeping owner locked"""
        self.following_enabled = False
        if self.owner_enrolled:
            self.target_state = Config.STATE_PAUSED
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

    def _accept_owner(self, cand, total, n_cands, now, w, lidar_alive=False):
        """Identity gate. Returns True only if this candidate may be treated as the owner.

        Track alive (camera or LiDAR has followed the owner without a break): position continuity is the
        main evidence. The candidate must be where the EKF expects the owner and look at least vaguely like
        them, which survives turning around, lighting and distance changes.
        Track lost: appearance must clear a bar relative to how the owner normally scores, for a few
        consecutive frames. Near where the owner was heading the bar is lower and the wait shorter.
        A near-identical match is accepted at once. A stranger who walks in as the owner walks out must
        beat all of that, and is logged as REID_REJECTED."""
        app = cand.get("similarity_to_owner", 0.0)
        lost = (self.target_lost_time is not None
                or self.target_state in (Config.STATE_SEARCHING, Config.STATE_TARGET_TEMPORARILY_LOST, Config.STATE_OWNER_LOST,
                                         Config.STATE_RECOVERING)
                or not self.ekf.active) and not lidar_alive
        f_px = (w / 2.0) / math.tan(math.radians(Config.CAMERA_HFOV_DEG) / 2.0)
        bearing = -math.atan((cand["centroid"][0] - w / 2.0) / f_px) + self.cam_bearing_bias
        ref = min(self.own_score, 0.75)

        if not lost:
            thr = max(0.25, Config.TRACK_RELATIVE * ref)
            px, py = self.ekf.s[0], self.ekf.s[1]
            r2 = max(px * px + py * py, 0.04)
            sigma_b = math.sqrt(max(0.0, (px * px * self.ekf.P[1, 1] + py * py * self.ekf.P[0, 0]) / (r2 * r2)))
            gate = max(math.radians(12.0), 4.0 * sigma_b) + math.radians(2.0) * min(self.ekf.misses, 10)
            in_gate = abs(bearing - self.ekf.range_bearing()[1]) < gate
            ok = app >= thr and in_gate
            if ok:
                self.own_score = 0.9 * self.own_score + 0.1 * min(app, 0.9)
            self.reid = {"state": "TRACKING" if ok else "REJECTED", "own": self.own_score, "best": app, "thr": thr,
                         "reacq": 0, "confirm": Config.REACQ_CONFIRM_FRAMES}
            self.reacq_count, self.reacq_x = 0, None
            return ok

        t_lost = now - self.target_lost_time if self.target_lost_time else 0.0
        thr = max(Config.REACQ_THRESHOLD, Config.REACQ_RELATIVE * ref)
        need = Config.REACQ_CONFIRM_FRAMES
        if self.ekf.active:
            widen = min(60.0, 25.0 + 10.0 * t_lost)
            if abs(bearing - self.ekf.range_bearing()[1]) < math.radians(widen):
                thr -= 0.08
                need = max(3, need - 2)
        x = cand["centroid"][0]
        if app >= thr:
            if self.reacq_x is not None and abs(x - self.reacq_x) > 0.25 * w:
                self.reacq_count = 0
            self.reacq_count += 1
            self.reacq_x = x
        else:
            self.reacq_count, self.reacq_x = 0, None
        ok = self.reacq_count >= need or app >= 0.80
        self.reid = {"state": "REACQUIRED" if ok else "REACQUIRING", "own": self.own_score, "best": app, "thr": thr,
                     "reacq": self.reacq_count, "confirm": need}
        if not ok and app < thr and now - self._last_reid_log > 3.0:
            self._last_reid_log = now
            self._log_event("REID_REJECTED", f"Someone else is in view (match {app:.2f} < {thr:.2f}). Not the owner; ignoring them.")
        if ok:
            self.reacq_count, self.reacq_x = 0, None
        return ok

    def _learn_view(self, cand, candidates, now, w):
        """While the track is alive, keep adding what the owner looks like from other angles (their back, side,
        different distance). Only when they are alone in the picture, so a bystander can never get into the profile."""
        if now - self.last_template_t < 1.5 or self.reid.get("state") not in ("TRACKING", "REACQUIRED"):
            return
        sig = cand.get("signature")
        if sig is None:
            return
        cx = cand["centroid"][0]
        if any(abs(c["centroid"][0] - cx) < 0.3 * w for c in candidates if c is not cand):
            return
        if cand.get("similarity_to_owner", 0.0) < (Config.TEMPLATE_UPDATE_MIN_CONF if len(candidates) > 1 else 0.30):
            return
        templates = self.owner_profile.setdefault("templates", [])
        if len(templates) < Config.MAX_OWNER_TEMPLATES:
            templates.append(sig.copy())
        else:
            self.template_count += 1
            templates[1 + self.template_count % (len(templates) - 1)] = sig.copy()
        self.last_template_t = now

    def _desired(self):
        """Current follow distance: the operator's set distance (always taken as is, immediately) + the smoothed
        margin. Only the margin is smoothed, so lowering the set distance never leaves a stale larger target."""
        return max(Config.MIN_FOLLOW_DISTANCE, self.base_target_distance) + self._dist_extra

    def _stop_resume(self):
        """Wait distance and resume distance, derived from the current (dynamic) follow distance."""
        d = self._desired()
        return max(0.4, d - 0.10), d + 0.15

    def _update_dynamic_distance(self):
        """Following distance = operator's set distance + a time headway on the owner's speed + a margin that grows
        as tracking confidence falls (EKF position sigma, camera-only or LiDAR-only ranging, weak Re-ID)."""
        base = max(Config.MIN_FOLLOW_DISTANCE, self.base_target_distance)
        extra, conf = 0.0, 1.0
        if self.ekf.active:
            speed = math.hypot(self.ekf.s[2], self.ekf.s[3])
            sigma = math.sqrt(max(self.ekf.P[0, 0], self.ekf.P[1, 1]))
            mode_pen = {"LIDAR+CAM": 0.0, "CAM": 0.25, "LIDAR": 0.35}.get(self.fusion_mode, 0.5)
            reid_pen = max(0.0, min(1.0, (0.55 - self.own_score) / 0.3)) if self.owner_enrolled else 0.0
            unc = max(0.0, min(1.0, (sigma - 0.05) / 0.25))
            conf = 1.0 - max(unc, mode_pen, reid_pen)
            extra = Config.DIST_SPEED_GAIN * speed + Config.DIST_CONF_GAIN * (1.0 - conf)
        target = max(0.0, min(Config.DIST_MAX_EXTRA, extra))
        # the margin grows quickly (safety) and shrinks slowly (no chasing the set point)
        a = 0.3 if target > self._dist_extra else 0.05
        self._dist_extra = (1 - a) * self._dist_extra + a * target
        self.dynamic_target_distance = base + self._dist_extra
        self.tracking_confidence = round(conf, 2)
        return self.dynamic_target_distance

    def _face_person(self, bearing):
        """Stationary but the person shifts sideways: turn to keep them centred."""
        if abs(bearing) < math.radians(3.0):
            return 0.0
        return max(-Config.FACE_TURN_MAX, min(Config.FACE_TURN_MAX, Config.FACE_TURN_GAIN * bearing))

    def _predict_turn(self):
        """Owner just lost: rotate (never drive) toward where the EKF says they are now, i.e. their last
        LiDAR/camera position carried on along their velocity. Returns (v, w, status)."""
        if not self.ekf.active:
            return 0.0, 0.0, "HOLD_POSITION"
        px, py = self.ekf.predicted(0.5)
        b = math.atan2(py, px)
        if abs(b) < math.radians(4.0):
            return 0.0, 0.0, "HOLD_POSITION"
        w = max(-Config.PREDICT_TURN_MAX, min(Config.PREDICT_TURN_MAX, Config.PREDICT_TURN_GAIN * b))
        if self._lidar_fresh():
            sec = lidar_lib.sectors(self.lidar_ranges)
            if (w > 0 and sec["left"] < 0.22) or (w < 0 and sec["right"] < 0.22):
                return 0.0, 0.0, "PREDICT_BLOCKED"
        return 0.0, w, "PREDICT_TURN"

    def _pursue_prediction(self, since):
        """No map plan, but the owner was walking when lost: drive toward their extrapolated position (EKF coast,
        velocity decaying) with LiDAR avoidance. None when there is no motion to follow, it is reached, or too old."""
        if (not self.ekf.active or getattr(self, "_lost_speed", 0.0) < Config.PREDICT_MIN_SPEED
                or since > Config.PREDICT_PURSUIT_SEC):
            return None
        r, b = self.ekf.range_bearing()
        if r <= self.base_target_distance + 0.2:
            return None
        w = 0.0 if abs(b) < math.radians(3.0) else max(-1.5, min(1.5, 0.9 * Config.PID_TURN_KP * b))
        v = Config.MEMORY_SPEED * max(0.0, math.cos(min(abs(b), math.pi / 2))) ** 2
        if abs(b) > math.radians(60.0):
            v = 0.0
        elif 0.0 < v < Config.MIN_ROLLING_VEL:
            v = Config.MIN_ROLLING_VEL
        status = "PREDICT_PURSUIT"
        if self._lidar_fresh():
            sec = lidar_lib.sectors(self.lidar_ranges)
            self.robot.set_front_clearance(sec["front_raw"], self.lidar_time)
            v, w, avoid = self.lidar_avoid.apply(v, w, sec)
            self._lidar_sectors = sec
            if avoid in ("LIDAR_STOP", "LIDAR_AVOIDING"):
                status = avoid
        self.target_state = Config.STATE_RECOVERING
        return v, w, status

    def _lidar_peek(self, now):
        """The camera cannot see the owner (legs only when close, behind something, off to the side) but a
        person-sized LiDAR cluster is where the EKF expects them: the track is still alive."""
        if not (self.owner_enrolled and self.ekf.active and self._lidar_fresh()):
            return None
        recalling = self.target_state == Config.STATE_RECOVERING
        if self.last_cam_seen is None or (now - self.last_cam_seen > Config.LIDAR_TRACK_MAX_SEC and not recalling):
            return None
        lost_for = (now - self.target_lost_time) if self.target_lost_time else 0.0
        gate = Config.LIDAR_TRACK_GATE_M + min(1.0, 0.25 * lost_for)   # more position doubt the longer they are unseen
        self.ekf.predict(now, self.robot.current_linear, self.robot.current_angular)
        r, b = self.ekf.range_bearing()
        window = math.degrees(math.atan2(0.5, max(r, 0.3))) + 6.0
        pc = lidar_lib.person_cluster(self.lidar_clusters, b, near=r, window_deg=window)
        if pc is None:
            return None
        if math.hypot(pc["cx"] - self.ekf.s[0], pc["cy"] - self.ekf.s[1]) > gate:
            return None
        return pc

    def _pursue_memory(self, now, since):
        """Owner out of sight (low camera, an obstacle in the way) and not found by the LiDAR where the EKF expects.
        Drive to where the map says they were last seen, round obstacles, slowly, with the LiDAR avoidance on top.
        The LiDAR peek and the camera keep looking the whole way and take over the moment they find the owner.
        Returns (v, w, status), or None when the memory has nothing more to offer (arrived, no path, timed out)."""
        if since >= Config.MEMORY_PURSUIT_SEC:
            return None
        t_now = time.time()
        plan = self.plan if self.plan is not None and t_now - self.plan_time < 1.0 else None
        if plan is None or not plan.get("memory"):
            if since < Config.SEARCH_START_DELAY_SEC + Config.MEMORY_PLAN_GRACE_SEC:
                self.target_state = Config.STATE_TARGET_TEMPORARILY_LOST
                v, w, st = self._predict_turn()          # face where they went while the planner catches up
                return v, w, ("MEMORY_PLANNING" if st == "HOLD_POSITION" else st)
            return self._pursue_prediction(since)       # no map: follow their motion; None -> search
        if plan.get("status") == "OFF_MAP":
            return self._pursue_prediction(since)
        if plan.get("status") not in ("DIRECT", "DETOUR"):
            return None                      # AT_GOAL: nobody here after all. NO_PATH: cannot get there.
        blocked = plan.get("blocked") and plan.get("wp_bearing") is not None
        b = plan["wp_bearing"] if blocked else plan["mem_bearing"]
        goal_d = plan.get("goal_dist", 1.0)
        w = 0.0 if abs(b) < math.radians(3.0) else max(-1.2, min(1.2, 0.9 * Config.PID_TURN_KP * b))
        v = min(Config.MEMORY_SPEED, 0.6 * goal_d + 0.03) * max(0.0, math.cos(min(abs(b), math.pi / 2))) ** 2
        if abs(b) > math.radians(60.0):
            v = 0.0
        elif 0.0 < v < Config.MIN_ROLLING_VEL:
            v = Config.MIN_ROLLING_VEL
        status = "MEMORY_DETOUR" if blocked else "MEMORY_PURSUIT"
        if self._lidar_fresh():
            sec = lidar_lib.sectors(self.lidar_ranges)       # nobody to exclude: the owner is not in view
            self.robot.set_front_clearance(sec["front_raw"], self.lidar_time)
            v, w, avoid = self.lidar_avoid.apply(v, w, sec)
            self._lidar_sectors = sec
            if avoid in ("LIDAR_STOP", "LIDAR_AVOIDING"):       # a side-clearance nudge does not change what we are doing
                status = avoid
        self.target_state = Config.STATE_RECOVERING
        return v, w, status

    def _recover_or_search(self, now):
        """Lost owner: first recall where they were and go there round any obstacle; the scan search is the last resort."""
        since = now - self.target_lost_time
        if not self.memory_done and since < Config.SEARCH_START_DELAY_SEC:
            self.target_state = Config.STATE_TARGET_TEMPORARILY_LOST
            return self._predict_turn()
        if not Config.RECOVERY_DRIVE_TO_LAST_SEEN:
            return self._search_command(now)          # rotate toward where they went, sweep, full 360, stop
        if not self.memory_done and since >= Config.SEARCH_START_DELAY_SEC:
            result = self._pursue_memory(now, since)
            if result is not None:
                return result
            self.memory_done = True
            self.search_t0 = now - Config.SEARCH_START_DELAY_SEC    # the scan starts fresh, no second hold
            self._search_t = None
            self.search_scan_rot = 0.0
            self._log_event("MEMORY_DONE", "Reached the last-seen position without finding the owner. Starting the 360 degree search.")
        return self._search_command(now)

    def _search_command(self, now):
        """Owner lost. Three phases, and together they cover the whole circle:
          1 PREDICTED  turn toward where the EKF says they went (last bearing carried on at their last velocity)
          2 SWEEP      swing wider and wider around that direction, up to +/-110 degrees
          3 SCAN       keep turning the same way until a full 360 degrees has been covered, then stop
        Returns (v, w, status)."""
        elapsed = now - (self.search_t0 if self.search_t0 is not None else self.target_lost_time)
        delay = Config.SEARCH_START_DELAY_SEC
        first_call = self._search_t is None
        dt = 0.0 if first_call else min(0.2, now - self._search_t)
        self._search_t = now
        if first_call:
            # the scan will go the way the owner left, decided now and kept for the whole search
            if self.ekf.active:
                self.search_dir = -1.0 if self.ekf.s[1] < 0 else 1.0
            else:
                self.search_dir = -1.0 if self.last_exit_direction >= 0 else 1.0
        if elapsed < delay:
            self.target_state = Config.STATE_TARGET_TEMPORARILY_LOST
            return 0.0, 0.0, "HOLD_POSITION"
        t = elapsed - delay
        if t >= Config.SEARCH_MAX_DURATION_SEC:
            self._owner_lost()
            return 0.0, 0.0, "SEARCH_TIMEOUT_HALT"
        sweep_end = 1.2 + Config.SEARCH_SWEEP_SEC
        if t >= sweep_end:
            self.search_scan_rot += abs(self.robot.current_angular) * dt
            if self.search_scan_rot >= 2 * math.pi:
                self._owner_lost()
                return 0.0, 0.0, "SEARCH_COMPLETE_360"
        self.target_state = Config.STATE_SEARCHING
        if t < sweep_end and self.ekf.active:
            b_pred = math.atan2(self.ekf.s[1], self.ekf.s[0])
            sweeping = t > 1.2
            amp = min(math.radians(110), math.radians(25) + math.radians(22) * (t - 1.2)) if sweeping else 0.0
            off = amp * math.sin(1.7 * (t - 1.2)) if sweeping else 0.0
            e = b_pred + off
            w = max(-1.0, min(1.0, 1.8 * e))
            if abs(e) > 0.05 and abs(w) < 0.25:
                w = math.copysign(0.25, e)
            status = "SEARCH_SWEEP" if sweeping else "SEARCH_PREDICTED"
        else:
            w = self.search_dir * Config.SEARCH_SCAN_SPEED
            status = f"SEARCH_SCAN_{int(math.degrees(self.search_scan_rot))}"
        if self._lidar_fresh():
            sec = lidar_lib.sectors(self.lidar_ranges)
            if (w > 0 and sec["left"] < 0.22) or (w < 0 and sec["right"] < 0.22):
                return 0.0, 0.0, "SEARCH_BLOCKED"
        return 0.0, w, status

    def _back_off(self, dist, bearing, half, rng):
        """Owner closer than the follow distance (they walked up to the robot): reverse until the gap is restored,
        turning to keep them centred. Only with a fresh LiDAR and a clear rear sector; the controller adds a hard
        rear stop. Returns (v, w, status) while backing off, else None."""
        if not Config.BACK_OFF_ENABLED:
            return None
        desired = self._desired()
        if getattr(self, "_backing", False):
            if dist >= desired - Config.BACK_OFF_RELEASE:
                self._backing = False
        elif dist < desired - Config.BACK_OFF_MARGIN:
            self._backing = True
            self._log_event("BACKING_OFF", f"Owner came within {dist:.2f} m: reversing to keep {desired:.1f} m.")
        if not self._backing:
            return None
        w = self._face_person(bearing)
        if not self._lidar_fresh():
            return 0.0, w, "BACK_OFF_NO_LIDAR"          # cannot see behind: never reverse blind
        sec = lidar_lib.sectors(self.lidar_ranges, exclude=(bearing, half, rng))
        self.robot.set_rear_clearance(sec["rear_raw"], self.lidar_time)
        self._lidar_sectors = sec
        if sec["rear"] < Config.REVERSE_CLEARANCE_M:
            return 0.0, w, "BACK_OFF_BLOCKED"
        v = Config.BACK_OFF_GAIN * (dist - desired)    # negative: too close
        v = max(-Config.MAX_REVERSE_VEL, min(-Config.PID_MIN_V, v))
        # slow down as the rear gets tight
        v *= max(0.3, min(1.0, (sec["rear"] - Config.REVERSE_CLEARANCE_M) / 0.4 + 0.3))
        return v, w, "BACKING_OFF"

    def _owner_lost(self):
        if self.target_state != Config.STATE_OWNER_LOST:
            self._log_event("OWNER_LOST", "Search finished without finding the owner. Stopped; waiting for them to reappear.")
        self.target_state = Config.STATE_OWNER_LOST

    def _paused_turn(self, bearing):
        """Paused by the operator: stationary. Still face the owner, but only in hold-to-follow (where facing is the
        normal idle behaviour); a classic pause means do not move at all."""
        return (0.0, self._face_person(bearing)) if Config.FOLLOW_REQUIRES_RAISED_HAND else (0.0, 0.0)

    def _lidar_fresh(self):
        return (Config.LIDAR_FUSION and self.lidar_ranges is not None
                and time.time() - self.lidar_time < Config.LIDAR_STALE_SEC)

    def _update_hand(self, up, now):
        """Debounce the hold-to-follow hand: RAISE_CONFIRM_FRAMES frames up to start, FOLLOW_RELEASE_SEC down to stop."""
        if up:
            self._raise_n += 1
            if self._raise_n >= Config.RAISE_CONFIRM_FRAMES:
                if not self.hand_raised:
                    self._log_event("FOLLOW_HAND_UP", "Hand raised: following.")
                self.hand_raised = True
                self._last_raise_t = now
        else:
            self._raise_n = 0
            if self.hand_raised and now - self._last_raise_t > Config.FOLLOW_RELEASE_SEC:
                self.hand_raised = False
                self._log_event("FOLLOW_HAND_DOWN", "Hand lowered: stopping here and keeping you centred.")

    def _vote_mirror(self, raw_bearing, box_half, cam_range, found_here):
        """Orientation self-check while the owner is tracked: is the LiDAR person at the camera's bearing, or at the
        mirror image of it? Only off-centre sightings count (near the middle both sides coincide)."""
        if abs(raw_bearing) < box_half + math.radians(4.0):
            return
        mirrored = lidar_lib.box_person_cluster(self.lidar_clusters, -raw_bearing, box_half, cam_range) is not None
        if found_here == mirrored:
            return                                        # both or neither: no evidence either way
        votes = self.__dict__.setdefault("_mirror_votes", [])
        votes.append(1 if mirrored else 0)
        del votes[:-40]
        self.mirror_suspect = len(votes) >= 25 and sum(votes) >= 0.8 * len(votes)

    def _fuse(self, cand, w, now, t_meas=None):
        """Camera pinhole bearing + range, cross-checked with the LiDAR, filtered by the EKF.
        Returns (range_m, bearing_rad). Bearing is positive to the left."""
        f_px = (w / 2.0) / math.tan(math.radians(Config.CAMERA_HFOV_DEG) / 2.0)
        cx = cand["centroid"][0]
        t_m = max(t_meas if t_meas else now, self.ekf.last_t or 0.0)
        raw_bearing = -math.atan((cx - w / 2.0) / f_px)
        bearing_cam = raw_bearing + self.cam_bearing_bias
        vis_d = cand["distance_m"] * self.vision_scale
        v_r, w_r = self.robot.current_linear, self.robot.current_angular

        lidar_r = lidar_b = None
        if self._lidar_fresh():
            # co-mapping: the person is the nearest person-sized cluster inside their own box, at a range the
            # camera roughly agrees with (raw camera range, not the LiDAR-scaled one, so a wrong lock cannot feed itself)
            x1, _, x2, _ = cand["bbox"]
            box_half = math.atan(0.5 * (x2 - x1) / f_px)
            pc = lidar_lib.box_person_cluster(self.lidar_clusters, bearing_cam, box_half, cand["distance_m"])
            self._owner_beams = pc["idx"] if pc is not None else None
            self._vote_mirror(raw_bearing, box_half, cand["distance_m"], pc is not None)
            if pc is not None:
                lidar_r, lidar_b = pc["range"], pc["bearing"]
        self.fusion_mode = "LIDAR+CAM" if lidar_r is not None else "CAM"

        if not self.ekf.active or self.ekf.rejects >= 6:
            self.ekf.init(lidar_r if lidar_r is not None else vis_d, bearing_cam, t_m)
            self.follow.reset()
        else:
            self.ekf.predict(t_m, v_r, w_r)
            self.ekf.update(vis_d, bearing_cam, 0.12 * vis_d, math.radians(1.5))
            if lidar_r is not None:
                if self.ekf.update(lidar_r, lidar_b, 0.05, math.radians(1.0)):
                    # the LiDAR is the better ruler: slowly correct the camera's size-based distance, but only from
                    # a close agreement and within bounds, so one bad association cannot walk the scale away
                    ratio = lidar_r / max(cand["distance_m"], 0.1)
                    if 0.7 <= ratio <= 1.4:
                        self.vision_scale = max(0.75, min(1.35, 0.98 * self.vision_scale + 0.02 * ratio))
                    err = (lidar_b - raw_bearing + math.pi) % (2 * math.pi) - math.pi
                    self.cam_bearing_bias = max(-0.1, min(0.1, 0.97 * self.cam_bearing_bias + 0.03 * err))
        self.ekf.predict(now, v_r, w_r)   # the frame is ~100 ms old by now; bring the estimate up to date
        r, b = self.ekf.range_bearing()
        return r, b, f_px

    def _follow_command(self, bearing, rng, half, obs_dists, now):
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
        px, py = self.ekf.s[0], self.ekf.s[1]
        ang_rate = (px * uy - py * ux) / max(px * px + py * py, 0.09)   # the person's own motion across the view
        v, w_cmd = self.follow.command(rng, b_ctl, self.ekf.range_rate(),
                                       self.ekf.bearing_rate(self.robot.current_angular), los, dt,
                                       desired=self._desired(),
                                       person_ang_rate=0.0 if detour else ang_rate)
        if self._lidar_fresh():
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

    def process_frame(self, frame, t_frame=None):
        """
        Main pipeline execution on a single video frame.
        Returns: (telemetry_dict, annotated_frame)
        """
        h, w = frame.shape[:2]
        if w > 480:
            new_h = max(1, int(round(h * (480.0 / w))))
            frame = cv2.resize(frame, (480, new_h), interpolation=cv2.INTER_LINEAR)
            h, w = frame.shape[:2]
        annotated = frame.copy() if self.render_hud else frame
        now = time.time()
        t_meas = t_frame if t_frame else now   # when the frame reached the laptop, not when we finished with it

        # Step 1: Detect all person candidates in the scene
        candidates = self._detect_all_persons(frame, annotated)
        self.last_candidates = candidates

        # Step 3: Multi-Person Disambiguation & Owner Recognition Matching
        matched_target = None
        lidar_pc = self._lidar_peek(now)
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
                if not self.ambiguous:
                    self._log_event("TARGET_AMBIGUOUS", "Two people match the owner equally well. Holding still until they separate.")
                self.ambiguous = True
                self.target_state = Config.STATE_TARGET_AMBIGUOUS
                matched_target = None
                self.owner_match_score = 0.0
            else:
                if self.ambiguous:
                    self._log_event("AMBIGUITY_RESOLVED", "Only one candidate matches the owner again.")
                self.ambiguous = False
                best_score = -1.0
                best_cand = None
                for cand in candidates:
                    if cand["similarity_to_target"] > best_score:
                        best_score = cand["similarity_to_target"]
                        best_cand = cand

                if best_cand is not None and self._accept_owner(best_cand, best_score, len(candidates), now, w, lidar_alive=lidar_pc is not None):
                    if self.target_state in [Config.STATE_SEARCHING, Config.STATE_TARGET_TEMPORARILY_LOST, Config.STATE_OWNER_LOST,
                                             Config.STATE_RECOVERING]:
                        self._log_event("TARGET_REACQUIRED", f"Owner re-acquired! Re-ID confidence: {best_score:.2f}")
                    self.target_lost_time = None

                    matched_target = best_cand
                    matched_target["is_target"] = True
                    matched_target["is_owner"] = True
                    self.target_id = matched_target["id"]
                    self.owner_match_score = round(best_score, 2)

                    self._learn_view(matched_target, candidates, now, w)

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

                    if lidar_pc is None and self.target_state in [Config.STATE_FOLLOWING, Config.STATE_WAIT_FOR_OWNER_MOVEMENT,
                                                                  Config.STATE_AWAITING_GESTURE, Config.STATE_LIDAR_GUIDED]:
                        self.target_state = Config.STATE_TARGET_TEMPORARILY_LOST
                        self.target_lost_time = now
                        self._log_event("TARGET_LOST", "Owner occluded or out of view. Robot stopped; holding position.")
        else:
            # No owner enrolled
            if self.target_state == Config.STATE_OWNER_RELEASED:
                if now - getattr(self, "_released_t", 0.0) > Config.RELEASED_SHOW_SEC:
                    self.target_state = Config.STATE_IDLE
            elif not (self.safety_latched and self.target_state == Config.STATE_SAFETY_STOP):
                self.target_state = Config.STATE_IDLE
            if len(candidates) > 0:
                # Potential candidate ready for enrollment
                matched_target = candidates[0]

        # Step 3b: Gestures (FollowMe HRI). Once an owner is enrolled only the owner can command the robot:
        # a bystander's raised hand or crossed arms is ignored. STOP_HALT is honoured from anyone in view.
        labels = [self.gestures.classify(c.get("landmarks")) for c in candidates]
        if self.owner_enrolled:
            subject = matched_target if (matched_target is not None and matched_target.get("is_owner")) else None
        else:
            subject = next((c for c, g in zip(candidates, labels) if g == "FOLLOW_ME"), None)
            if subject is None and candidates:
                subject = max(candidates, key=lambda c: (c["bbox"][2] - c["bbox"][0]) * (c["bbox"][3] - c["bbox"][1]))
        subj_i = next((i for i, c in enumerate(candidates) if c is subject), None)
        active_gesture, is_new_gesture = self.gestures.feed(labels[subj_i] if subj_i is not None else "NONE")
        stop_g, stop_new = self.any_stop.feed("STOP_HALT" if "STOP_HALT" in labels else "NONE")
        if stop_new and not (is_new_gesture and active_gesture == "STOP_HALT"):
            active_gesture, is_new_gesture = "STOP_HALT", True
        elif stop_g == "STOP_HALT" and active_gesture == "NONE":
            active_gesture = "STOP_HALT"
        if self.owner_enrolled:
            ignored = [g for i, g in enumerate(labels) if i != subj_i and g not in ("NONE", "STOP_HALT")]
            if ignored and now - getattr(self, "_last_ignored_log", 0.0) > 3.0:
                self._last_ignored_log = now
                self._log_event("GESTURE_IGNORED", f"Bystander gesture '{ignored[0]}' ignored: only the owner can command the robot.")

        if is_new_gesture:
            self._log_event("GESTURE_DETECTED", f"Recognized user gesture: '{active_gesture}'")
            if active_gesture == "FOLLOW_ME":
                if not self.owner_enrolled:
                    if subject is not None:
                        self.enroll_owner(subject)
                        matched_target = subject
                else:
                    self.start_following()
            elif active_gesture == "WAIT" and not Config.FOLLOW_REQUIRES_RAISED_HAND:
                self.pause_following()
            elif active_gesture == "RELEASE":
                self.release_owner()
                matched_target = None
            elif active_gesture == "STOP_HALT":
                self.trigger_safety_stop("Robot halted by STOP gesture. Raise one hand (FOLLOW ME) or press START to resume.")
        manual_stop_signal = self.safety_latched or active_gesture == "STOP_HALT"
        if self.owner_enrolled and subj_i is not None:
            up = labels[subj_i] != "STOP_HALT" and self.gestures.hand_raised(candidates[subj_i].get("landmarks"))
            self._update_hand(up, now)

        # Step 4: Lost-Human Trajectory Projection (Conservative stop, no blind rotation)
        projected_bbox = None
        if self.target_state == Config.STATE_TARGET_TEMPORARILY_LOST:
            if self.target_lost_time is None:
                self.target_lost_time = now
            occlusion_duration = now - self.target_lost_time
            if occlusion_duration <= self.max_recovery_seconds and self.target_kalman is not None:
                projected_bbox, (vx, vy) = self.target_kalman.predict()
                if self.render_hud:
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
            "dynamic_target_distance": self.base_target_distance,
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
        tracking = (matched_target is not None or lidar_pc is not None) and not self.ambiguous
        if tracking:
            tan_half = math.tan(math.radians(Config.CAMERA_HFOV_DEG) / 2.0)
            f_px = (w / 2.0) / tan_half
            if matched_target is not None:
                fused_r, fused_b, f_px = self._fuse(matched_target, w, now, t_meas)
                x1, _, x2, _ = matched_target["bbox"]
                half = math.atan(0.5 * (x2 - x1) / f_px) + 0.06
                self.last_cam_seen = now
                self.lidar_only = False
            else:
                # camera lost the owner but the LiDAR still has them: keep following, do not search
                self.ekf.update(lidar_pc["range"], lidar_pc["bearing"], 0.06, math.radians(1.5))
                self._owner_beams = lidar_pc["idx"]
                fused_r, fused_b = self.ekf.range_bearing()
                half = math.atan2(lidar_pc["width"] / 2.0, max(fused_r, 0.3)) + 0.06
                self.fusion_mode = "LIDAR"
                if not self.lidar_only:
                    self._raised_at_loss = self.hand_raised
                    self._log_event("LIDAR_TRACKING", "Owner out of camera view. LiDAR has them: turning to bring them back into view.")
                if self.target_state == Config.STATE_RECOVERING:
                    # found again by the LiDAR while driving to the remembered spot: give the approach a fresh leash so the
                    # camera has time to see them again, and say so
                    self.last_cam_seen = now               # a fresh LiDAR leash; the camera must confirm within it
                    self._log_event("MEMORY_RELOCK", "LiDAR found the owner near the remembered position. Following again.")
                self.lidar_only = True
                if self.target_state in (Config.STATE_SEARCHING, Config.STATE_TARGET_TEMPORARILY_LOST,
                                         Config.STATE_RECOVERING, Config.STATE_IDLE, Config.STATE_OWNER_LOST):
                    self.target_state = Config.STATE_WAIT_FOR_OWNER_MOVEMENT
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

            self._update_dynamic_distance()
            stop_d, resume_d = self._stop_resume()
            # Evaluate State Machine Transitions
            if not self.owner_enrolled:
                # Idle state
                v_x, w_z = 0.0, 0.0
            elif not self.following_enabled:
                # Paused / locked state (or latched by STOP_HALT)
                self.target_state = Config.STATE_SAFETY_STOP if self.safety_latched else Config.STATE_PAUSED
                v_x, w_z = self._paused_turn(fused_b)
            elif Config.FOLLOW_REQUIRES_RAISED_HAND and self.lidar_only:
                # Lost from the camera, LiDAR still has them: LiDAR-guided search. Turn to bring them back into the
                # picture; keep up with them only if they were being followed (hand up) when the camera lost them.
                self.target_state = Config.STATE_LIDAR_GUIDED
                v_x, w_z = 0.0, self._face_person(fused_b)     # rotate to find them; never drive blind
                telemetry["nav_status"] = "LIDAR_GUIDED_TURN"
            elif Config.FOLLOW_REQUIRES_RAISED_HAND and not self.hand_raised:
                # Owner in view, hand down: stay put, keep them centred in the camera, never search.
                if self.target_state != Config.STATE_AWAITING_GESTURE:
                    self._log_event("AWAITING_GESTURE", "Owner in view. Raise a hand above your shoulder to make me follow.")
                self.target_state = Config.STATE_AWAITING_GESTURE
                self.follow.reset()
                self.last_ctl_t = None
                v_x, w_z = 0.0, self._face_person(fused_b)
                telemetry["nav_status"] = "AWAITING_GESTURE"
                bo = self._back_off(dist, fused_b, half, fused_r)
                if bo is not None:
                    v_x, w_z, telemetry["nav_status"] = bo
            else:
                # Following is enabled for enrolled owner
                # Required transition: FOLLOWING -> WAIT_FOR_OWNER_MOVEMENT
                if dist <= stop_d:
                    self.target_state = Config.STATE_WAIT_FOR_OWNER_MOVEMENT
                    self.resume_counter = 0
                    v_x = 0.0
                    # Gentle camera re-centering if owner steps laterally while standing near
                    w_z = self._face_person(fused_b)
                    bo = self._back_off(dist, fused_b, half, fused_r)
                    if bo is not None:
                        v_x, w_z, telemetry["nav_status"] = bo
                elif dist >= resume_d or self.target_state == Config.STATE_FOLLOWING:
                    self._backing = False
                    if self.target_state == Config.STATE_WAIT_FOR_OWNER_MOVEMENT:
                        self.resume_counter += 1
                        if self.resume_counter >= Config.RESUME_HYSTERESIS_FRAMES:
                            self.target_state = Config.STATE_FOLLOWING
                            self._log_event("STATE_RESUME", f"Owner moved away ({dist:.2f}m >= {resume_d:.2f}m). Resuming FOLLOWING.")
                    else:
                        self.target_state = Config.STATE_FOLLOWING

                    if self.target_state == Config.STATE_FOLLOWING:
                        # EKF-fed PID, then LiDAR avoidance
                        v_x, w_z, nav_status = self._follow_command(fused_b, fused_r, half, obs_dists, now)
                        telemetry["nav_status"] = nav_status
                    else:
                        v_x = 0.0
                        w_z = 0.0
                else:
                    # In hysteresis band (stop .. resume): not too close, so no back-off latch survives here
                    self._backing = False
                    if self.target_state == Config.STATE_WAIT_FOR_OWNER_MOVEMENT:
                        v_x = 0.0
                        w_z = self._face_person(fused_b)
                    else:
                        # EKF-fed PID, then LiDAR avoidance
                        v_x, w_z, nav_status = self._follow_command(fused_b, fused_r, half, obs_dists, now)
                        telemetry["nav_status"] = nav_status

            telemetry["phase2_cmd_vel"] = {"linear_x": round(v_x, 2), "angular_z": round(w_z, 2)}
        elif self.ambiguous:
            # Two equally good matches: hold still. No loss timer, so no memory pursuit or search either.
            self.follow.reset()
            self.last_ctl_t = None
            if self.ekf.active:
                self.ekf.coast(now, self.robot.current_linear, self.robot.current_angular)
            telemetry["nav_status"] = "AMBIGUOUS_HOLD"
            telemetry["phase2_cmd_vel"] = {"linear_x": 0.0, "angular_z": 0.0}
        else:
            # Owner is in neither the camera nor the LiDAR where we expect them
            self.follow.reset()
            self.last_ctl_t = None
            self.lidar_only = False
            if self.ekf.active:
                self.ekf.coast(now, self.robot.current_linear, self.robot.current_angular)
                if self.target_lost_time is not None and now - self.target_lost_time > Config.SEARCH_MEMORY_SEC:
                    self.ekf.reset()
                else:
                    self.last_exit_direction = 1 if self.ekf.s[1] < 0 else -1  # predicted to be on the right
            if self.owner_enrolled and self.following_enabled and self.last_cam_seen is None:
                # restored profile, owner not seen yet this session: wait for them, do not search for them
                if self.target_state not in (Config.STATE_SAFETY_STOP, Config.STATE_PAUSED):
                    self.target_state = Config.STATE_OWNER_LOCKED
                telemetry["nav_status"] = "WAITING_FOR_OWNER"
            elif self.owner_enrolled and self.following_enabled:
                if self.target_lost_time is None:
                    self.target_lost_time = now
                    self._lost_speed = math.hypot(self.ekf.s[2], self.ekf.s[3]) if self.ekf.active else 0.0
                    self._search_t = None
                    self.search_scan_rot = 0.0
                    self.memory_done = False
                    self.search_t0 = None
                    self.target_state = Config.STATE_TARGET_TEMPORARILY_LOST
                    where = ("RIGHT" if self.last_exit_direction >= 0 else "LEFT")
                    self._log_event("TARGET_LOST", f"Owner out of sight, last seen heading {where}. Recalling their last position from the map.")
                v_x, w_z, nav = self._recover_or_search(now)
                telemetry["nav_status"] = nav
            else:
                telemetry["nav_status"] = "IDLE"

            telemetry["phase2_cmd_vel"] = {"linear_x": round(v_x, 2), "angular_z": round(w_z, 2)}

        if self.safety_latched:
            self.target_state = Config.STATE_SAFETY_STOP
            v_x, w_z = 0.0, 0.0
        # Sensor abnormality: the LiDAR was streaming and has gone silent. Avoidance and the hard stop depend on it.
        lidar_fault = (Config.LIDAR_FUSION and self.lidar_ranges is not None
                       and now - self.lidar_time > Config.LIDAR_FAULT_SEC)
        if lidar_fault:
            if not getattr(self, "_lidar_fault", False):
                self._log_event("SENSOR_FAULT", "LiDAR stopped sending scans. Robot stopped until it returns.")
            v_x, w_z = 0.0, 0.0
            telemetry["nav_status"] = "SENSOR_FAULT_LIDAR"
        elif getattr(self, "_lidar_fault", False):
            self._log_event("SENSOR_OK", "LiDAR scans are back.")
        self._lidar_fault = lidar_fault
        telemetry["sensor_fault"] = "LIDAR_STALE" if lidar_fault else None
        if self.safety_latched or lidar_fault or self.robot.emergency_stop:
            telemetry["phase2_cmd_vel"] = {"linear_x": 0.0, "angular_z": 0.0}   # report what is actually sent
        # the state after this frame's transitions, not the one it started in
        telemetry["target_state"] = self.target_state
        telemetry["follow_state"] = self.target_state           # the behaviour state underneath any override
        # Overrides that stop or bypass autonomy are shown as the state, so the diagram never claims FOLLOWING while
        # the wheels are held. Priority: E-STOP / latch > sensor fault > manual teleop.
        if self.robot.emergency_stop or self.safety_latched:
            telemetry["target_state"] = Config.STATE_SAFETY_STOP
        elif lidar_fault:
            telemetry["target_state"] = Config.STATE_SENSOR_FAULT
        elif self.robot.mode == "MANUAL":
            telemetry["target_state"] = Config.STATE_MANUAL_CONTROL
        telemetry["safety_latched"] = self.safety_latched
        telemetry["hand_raised"] = self.hand_raised
        telemetry["hold_to_follow"] = bool(Config.FOLLOW_REQUIRES_RAISED_HAND)
        telemetry["ambiguous"] = self.ambiguous

        # Step 6: Safety Layer Evaluation
        safety_eval = self.safety.evaluate_safety(
            frame,
            telemetry["distance_m"],
            telemetry["target_state"],
            manual_stop_gesture=manual_stop_signal
        )
        if (safety_eval["status"] == "CRITICAL_PROXIMITY_STOP" and v_x < 0.0 and not self.safety_latched
                and not lidar_fault):
            safety_eval["can_move"] = True                # backing away from someone too close is the safe move
            safety_eval["alerts"] = list(safety_eval["alerts"]) + ["Backing away from the person"]
        if lidar_fault:
            safety_eval["status"], safety_eval["can_move"] = "SENSOR_FAULT", False
            safety_eval["alerts"] = list(safety_eval["alerts"]) + ["Sensor: LiDAR data stale (> %.1f s)" % Config.LIDAR_FAULT_SEC]
        telemetry["safety_status"] = safety_eval["status"]
        telemetry["safety_alerts"] = safety_eval["alerts"]

        # Forward verified velocity commands to physical robot motors
        if tracking:
            self.robot.update_auto_commands(v_x, w_z, can_move=safety_eval["can_move"])
        elif self.target_state in (Config.STATE_SEARCHING, Config.STATE_RECOVERING,
                                   Config.STATE_TARGET_TEMPORARILY_LOST) and (w_z != 0.0 or v_x != 0.0):
            self.robot.update_auto_commands(v_x, w_z, can_move=safety_eval["can_move"])
        else:
            self.robot.update_auto_commands(0.0, 0.0, can_move=False)

        telemetry["robot_telemetry"] = self.robot.get_telemetry()
        telemetry["person"] = self.ekf.as_dict() if self.ekf.active else None
        telemetry["reid"] = dict(self.reid) if self.owner_enrolled else None
        telemetry["recalling"] = self.target_state == Config.STATE_RECOVERING
        telemetry["fusion"] = {"mode": self.fusion_mode if tracking else "LOST",
                               "vision_scale": round(self.vision_scale, 3),
                               "bearing_bias_deg": round(math.degrees(self.cam_bearing_bias), 1),
                               "lidar_only": self.lidar_only,
                               "lidar": self._lidar_fresh(),
                               "sectors": getattr(self, "_lidar_sectors", None) if self._lidar_fresh() else None}

        # Co-mapping: the LiDAR scan projected into this image, and each person's LiDAR range from the beams
        # that fall inside their box. The browser draws both over the live video.
        proj = None
        if self._lidar_fresh():
            cam = fusion.CameraModel(w, h, self.cam_bearing_bias)
            proj = cam.project_scan(self.lidar_ranges)
        owner_beams = getattr(self, "_owner_beams", None) if tracking else None
        telemetry["lidar_cam"] = fusion.overlay_points(proj, owner_beams) if proj is not None else []
        overlay = []
        for cand in candidates:
            x1, y1, x2, y2 = cand["bbox"]
            lidar_m = None
            if proj is not None:
                lidar_m, _n = fusion.box_range(proj, cam.box_beams(proj, cand["bbox"]), near=cand["distance_m"])
            overlay.append({
                "bbox": [
                    round(x1 / float(w), 4),
                    round(y1 / float(h), 4),
                    round(x2 / float(w), 4),
                    round(y2 / float(h), 4),
                ],
                "target": bool(cand.get("is_target") or cand.get("is_owner")),
                "owner": bool(cand.get("is_owner")),
                "pose": cand.get("pose_norm") or [],
                "cam_m": round(float(cand["distance_m"] * self.vision_scale), 2),
                "lidar_m": None if lidar_m is None else round(lidar_m, 2),
                "match": cand.get("similarity_to_owner"),
            })
        telemetry["overlay"] = overlay
        telemetry["mirror_suspect"] = bool(getattr(self, "mirror_suspect", False))
        telemetry["camera"] = {"mirror": bool(Config.CAM_MIRROR), "hfov_deg": Config.CAMERA_HFOV_DEG, "yaw_deg": round(Config.CAM_YAW_DEG + math.degrees(self.cam_bearing_bias), 2)}
        telemetry["dynamic_target_distance"] = round(self._desired(), 2)
        telemetry["tracking_confidence"] = getattr(self, "tracking_confidence", None)
        telemetry["control"] = {"set": self.base_target_distance, "desired": round(self._desired(), 2), "stop": round(self._stop_resume()[0], 2),
                                "resume": round(self._stop_resume()[1], 2), "cmd_v": round(v_x, 3), "cmd_w": round(w_z, 3)}

        # Step 7: Render Tactical HUD with Obstacle Corridors
        if self.render_hud:
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
                            # Draw skeleton (only when an OpenCV HUD is wanted; the dashboard draws its own)
                            for p1, p2 in (self.POSE_CONNECTIONS if self.render_hud else ()):
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

                            # pinhole: range = f * real size / pixel size, sizes from Config (no magic constants)
                            f_px = (w / 2.0) / math.tan(math.radians(Config.CAMERA_HFOV_DEG) / 2.0)
                            dist_estimates = []
                            if sh_l and sh_r and sh_l[2] > 0.35 and sh_r[2] > 0.35:
                                sh_w = math.hypot(sh_r[0] - sh_l[0], sh_r[1] - sh_l[1])
                                if sh_w > 15:
                                    dist_estimates.append(f_px * Config.PERSON_SHOULDER_M / sh_w)

                            if hip_l and hip_r and hip_l[2] > 0.35 and hip_r[2] > 0.35:
                                hip_w = math.hypot(hip_r[0] - hip_l[0], hip_r[1] - hip_l[1])
                                if hip_w > 15:
                                    dist_estimates.append(f_px * Config.PERSON_HIP_M / hip_w)

                            if sh_l and sh_r and hip_l and hip_r:
                                th1 = abs(hip_l[1] - sh_l[1])
                                th2 = abs(hip_r[1] - sh_r[1])
                                torso_h = max(th1, th2)
                                if torso_h > 35:
                                    dist_estimates.append(f_px * Config.PERSON_TORSO_M / float(torso_h))

                            if dist_estimates:
                                dist_m = sum(dist_estimates) / len(dist_estimates)
                            else:
                                bbox_h = max(40, y2 - y1)
                                dist_m = f_px * Config.PERSON_BOX_M / float(bbox_h)
                            dist_m *= Config.DISTANCE_SCALE

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
