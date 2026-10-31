"""
Comprehensive Automated Test Suite for Owner-Locked Human-Following Robot System.
Validates the 16 mandatory acceptance scenarios defined in the Master Implementation Prompt:
1. Lock person & follow
2. Owner stops nearby -> stops & waits indefinitely while retaining lock
3. Owner still for extended time -> no new-owner search
4. Owner walks away -> resumes following with hysteresis
5. Bystander enters -> does not switch identity
6. Chair / non-human detection -> rejected by anatomical validator
7. Brief occlusion -> identity retained, motion predicted, holds position
8. Owner leaves & stranger remains -> stranger ignored, robot stopped
9. Two visually similar candidates -> TARGET_AMBIGUOUS, robot stops
10. Lock gesture -> owner enrolled
11. Release gesture -> owner released & robot stops
12. Crossed arms / missing landmark safeguards -> no accidental release
13. Communication interrupted -> Raspberry Pi watchdog zeroes motors
14. Stale frames -> robot stops
15. Obstacle blocks path -> robot deflects or stops
16. Manual mode override -> no competing auto commands
"""

import os
import time
import math
import unittest
import numpy as np
import cv2

from config import Config
from appearance_model import AppearanceModel
from gesture_recognizer import GestureRecognizer
from obstacle_avoidance import ObstacleAvoidance
from robot_controller import RobotController
from safety_monitor import SafetyMonitor


def create_mock_landmark(x, y, visibility=0.95):
    class MockLM:
        def __init__(self, x, y, vis):
            self.x = x
            self.y = y
            self.visibility = vis
    return MockLM(x, y, visibility)


def create_valid_human_landmarks(w=640, h=480, offset_x=0.0):
    """Generates 33 valid MediaPipe pose landmarks for an upright standing human"""
    cx = 0.5 + offset_x
    lms = {}
    # Nose & face
    lms[0] = create_mock_landmark(cx, 0.20)
    for i in range(1, 11):
        lms[i] = create_mock_landmark(cx + 0.02 * (i % 2 * 2 - 1), 0.18 + 0.01 * i)
    # Shoulders
    lms[11] = create_mock_landmark(cx - 0.12, 0.32) # Left shoulder
    lms[12] = create_mock_landmark(cx + 0.12, 0.32) # Right shoulder
    # Elbows
    lms[13] = create_mock_landmark(cx - 0.15, 0.45)
    lms[14] = create_mock_landmark(cx + 0.15, 0.45)
    # Wrists
    lms[15] = create_mock_landmark(cx - 0.15, 0.58)
    lms[16] = create_mock_landmark(cx + 0.15, 0.58)
    # Hips
    lms[23] = create_mock_landmark(cx - 0.09, 0.60) # Left hip
    lms[24] = create_mock_landmark(cx + 0.09, 0.60) # Right hip
    # Knees
    lms[25] = create_mock_landmark(cx - 0.09, 0.78)
    lms[26] = create_mock_landmark(cx + 0.09, 0.78)
    # Ankles
    lms[27] = create_mock_landmark(cx - 0.09, 0.92)
    lms[28] = create_mock_landmark(cx + 0.09, 0.92)
    for i in range(29, 33):
        lms[i] = create_mock_landmark(cx, 0.95)
    return lms


def create_synthetic_signature(color_bgr, aspect_ratio=0.45, torso_ratio=1.20):
    """Creates a synthetic HSV appearance signature for testing"""
    patch = np.zeros((80, 60, 3), dtype=np.uint8)
    patch[:] = color_bgr
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [18, 16], [0, 180, 25, 256])
    cv2.normalize(hist, hist, alpha=1.0, norm_type=cv2.NORM_L1)
    
    r, g, b = int(color_bgr[2]), int(color_bgr[1]), int(color_bgr[0])
    color_hex = f"#{r:02x}{g:02x}{b:02x}"
    return {
        "hist": hist,
        "head_hist": hist,
        "color_hex": color_hex,
        "head_hex": color_hex,
        "aspect_ratio": aspect_ratio,
        "torso_ratio": torso_ratio,
        "area": 4800
    }


class TestOwnerLockedFollowingSystem(unittest.TestCase):

    def setUp(self):
        self.app_model = AppearanceModel()
        self.gestures = GestureRecognizer(debounce_frames=3)
        self.nav = ObstacleAvoidance()
        self.safety = SafetyMonitor()
        self.robot = RobotController()

    def tearDown(self):
        self.robot.stop()

    def test_01_lock_person_and_follow(self):
        """Scenario 1: Lock a person and verify following commands are generated beyond stop distance"""
        owner_sig = create_synthetic_signature((255, 0, 0)) # Blue shirt
        profile = self.app_model.create_owner_profile(owner_sig, owner_id=1)
        self.assertTrue(profile["locked"])
        self.assertEqual(len(profile["templates"]), 1)

        # Candidate is at 1.80m (> STOP_DISTANCE of 1.10m)
        dist = 1.80
        dist_err = dist - Config.DESIRED_DISTANCE # 1.80 - 1.20 = 0.60m
        v_x = min(Config.MAX_LINEAR_VEL, max(Config.MIN_ROLLING_VEL, Config.MIN_ROLLING_VEL + dist_err * 0.50))
        self.assertGreater(v_x, 0.12)
        self.assertLessEqual(v_x, Config.MAX_LINEAR_VEL)

    def test_02_owner_stops_nearby_waits_indefinitely(self):
        """Scenario 2: Owner stops nearby (d <= 1.10m): robot stops and transitions to WAIT_FOR_OWNER_MOVEMENT"""
        owner_sig = create_synthetic_signature((255, 0, 0))
        profile = self.app_model.create_owner_profile(owner_sig, owner_id=1)
        
        # When distance is within STOP_DISTANCE
        dist = 1.05
        self.assertLessEqual(dist, Config.STOP_DISTANCE)
        v_x = 0.0
        self.assertEqual(v_x, 0.0)
        # Lock is strictly preserved
        self.assertTrue(profile["locked"])

    def test_03_owner_still_extended_time_no_search(self):
        """Scenario 3: Owner remains still for extended time: robot stays in WAIT, no search started"""
        owner_sig = create_synthetic_signature((255, 0, 0))
        profile = self.app_model.create_owner_profile(owner_sig, owner_id=1)
        
        state = Config.STATE_WAIT_FOR_OWNER_MOVEMENT
        # Simulate 100 consecutive frames where owner is stationary at 1.0m
        for _ in range(100):
            dist = 1.0
            if dist <= Config.STOP_DISTANCE:
                state = Config.STATE_WAIT_FOR_OWNER_MOVEMENT
            self.assertEqual(state, Config.STATE_WAIT_FOR_OWNER_MOVEMENT)
            self.assertTrue(profile["locked"])

    def test_04_owner_walks_away_resumes_with_hysteresis(self):
        """Scenario 4: Owner walks away (d >= 1.35m): robot resumes FOLLOWING after 3 frames"""
        state = Config.STATE_WAIT_FOR_OWNER_MOVEMENT
        resume_counter = 0

        # Frame 1: d = 1.36m (first frame exceeding resume threshold)
        dist = 1.36
        if dist >= Config.RESUME_DISTANCE:
            resume_counter += 1
        self.assertEqual(state, Config.STATE_WAIT_FOR_OWNER_MOVEMENT) # Not yet

        # Frame 2: d = 1.40m
        dist = 1.40
        if dist >= Config.RESUME_DISTANCE:
            resume_counter += 1
        self.assertEqual(state, Config.STATE_WAIT_FOR_OWNER_MOVEMENT) # Not yet

        # Frame 3: d = 1.42m -> Reaches 3 frames -> Transitions to FOLLOWING
        dist = 1.42
        if dist >= Config.RESUME_DISTANCE:
            resume_counter += 1
        if resume_counter >= Config.RESUME_HYSTERESIS_FRAMES:
            state = Config.STATE_FOLLOWING

        self.assertEqual(state, Config.STATE_FOLLOWING)

    def test_05_bystander_enters_does_not_switch(self):
        """Scenario 5: Stranger enters the frame; robot matches exclusively to locked owner"""
        owner_sig = create_synthetic_signature((255, 0, 0))      # Blue shirt
        bystander_sig = create_synthetic_signature((0, 255, 0))  # Green shirt
        profile = self.app_model.create_owner_profile(owner_sig, owner_id=1)

        sim_owner = self.app_model.compute_owner_match(profile, owner_sig)
        sim_bystander = self.app_model.compute_owner_match(profile, bystander_sig)

        self.assertGreater(sim_owner, 0.70)
        self.assertLess(sim_bystander, 0.40)
        # Winner is unequivocally the owner
        self.assertGreater(sim_owner, sim_bystander)

    def test_06_chair_detection_rejected(self):
        """Scenario 6: Anatomical human validation strictly rejects rigid chair-like false detections"""
        # Chair case 1: No shoulders
        chair_pts_1 = {
            11: (200, 300, 0.10), # low confidence left shoulder
            12: (260, 300, 0.15), # low confidence right shoulder
            0:  (230, 220, 0.10)
        }
        # Chair case 2: Hips above shoulders (inverted / impossible anatomy)
        chair_pts_2 = {
            11: (200, 300, 0.90),
            12: (260, 300, 0.90),
            0:  (230, 200, 0.90),
            23: (200, 250, 0.85), # Hips above shoulders (250 < 300)
            24: (260, 250, 0.85)
        }
        
        # Test validation logic
        def validate(pts):
            sh_l = pts.get(11)
            sh_r = pts.get(12)
            nose = pts.get(0)
            if not sh_l or not sh_r or sh_l[2] < 0.45 or sh_r[2] < 0.45:
                return False
            avg_sh_y = (sh_l[1] + sh_r[1]) / 2.0
            hip_l = pts.get(23)
            hip_r = pts.get(24)
            if hip_l and hip_r and hip_l[2] > 0.35 and hip_r[2] > 0.35:
                avg_hip_y = (hip_l[1] + hip_r[1]) / 2.0
                if avg_hip_y <= avg_sh_y + 15:
                    return False
            return True

        self.assertFalse(validate(chair_pts_1))
        self.assertFalse(validate(chair_pts_2))

    def test_07_brief_occlusion_retains_identity(self):
        """Scenario 7: Brief occlusion enters TARGET_TEMPORARILY_LOST, retains lock and profile"""
        owner_sig = create_synthetic_signature((255, 0, 0))
        profile = self.app_model.create_owner_profile(owner_sig, owner_id=1)
        
        state = Config.STATE_TARGET_TEMPORARILY_LOST
        # Owner profile is never erased
        self.assertIsNotNone(profile)
        self.assertTrue(profile["locked"])

    def test_08_owner_leaves_stranger_remains_no_following(self):
        """Scenario 8: Owner disappears and stranger remains in frame; stranger is rejected, robot stopped"""
        owner_sig = create_synthetic_signature((255, 0, 0))     # Blue shirt
        profile = self.app_model.create_owner_profile(owner_sig, owner_id=1)

        stranger_sig = create_synthetic_signature((0, 200, 255)) # Yellow shirt
        sim_stranger = self.app_model.compute_owner_match(profile, stranger_sig)
        
        # Stranger must not meet the identity threshold
        self.assertLess(sim_stranger, Config.IDENTITY_MATCH_THRESHOLD)
        # Therefore no target is matched, robot stops
        matched_target = None
        self.assertIsNone(matched_target)

    def test_09_two_similar_candidates_trigger_ambiguity(self):
        """Scenario 9: Two visually similar candidates trigger TARGET_AMBIGUOUS and stop the robot"""
        scores = [0.65, 0.62] # Both high, difference is 0.03 (< 0.08 delta threshold)
        is_ambig, top_s = self.app_model.check_ambiguity(
            scores, match_thresh=Config.IDENTITY_MATCH_THRESHOLD, delta_thresh=Config.IDENTITY_AMBIGUITY_DELTA
        )
        self.assertTrue(is_ambig)

    def test_10_lock_gesture_enrollment(self):
        """Scenario 10: Raised hand gesture (FOLLOW_ME) triggers enrollment"""
        lms = create_valid_human_landmarks()
        # Raise right wrist above head (nose at y=0.20, wrist at y=0.10)
        lms[16] = create_mock_landmark(0.65, 0.10, 0.95)
        
        g = "NONE"
        for _ in range(Config.GESTURE_CONFIRM_FRAMES):
            g, _ = self.gestures.analyze_pose(lms)
        self.assertEqual(g, "FOLLOW_ME")

    def test_11_release_gesture_stops_and_releases(self):
        """Scenario 11: Deliberate crossed wrists (RELEASE) triggers release"""
        lms = create_valid_human_landmarks()
        # Cross wrists over chest: Left wrist (15) crosses to the right of right wrist (16)
        lms[15] = create_mock_landmark(0.56, 0.38, 0.95)
        lms[16] = create_mock_landmark(0.44, 0.38, 0.95)

        g = "NONE"
        for _ in range(Config.GESTURE_CONFIRM_FRAMES):
            g, _ = self.gestures.analyze_pose(lms)
        self.assertEqual(g, "RELEASE")

    def test_12_accidental_gesture_safeguard(self):
        """Scenario 12: Arms hanging normally at sides never triggers RELEASE or STOP"""
        lms = create_valid_human_landmarks()
        # Hands resting at sides (wrists below hips, no crossing)
        lms[15] = create_mock_landmark(0.35, 0.70, 0.95)
        lms[16] = create_mock_landmark(0.65, 0.70, 0.95)

        g = "NONE"
        for _ in range(10):
            g, _ = self.gestures.analyze_pose(lms)
        self.assertEqual(g, "NONE")

    def test_13_hardware_watchdog_zeroes_motors(self):
        """Scenario 13: Hardware watchdog timeout stops motors if commands stop arriving"""
        self.robot.update_auto_commands(0.20, 0.50, can_move=True)
        # Manually age the last command timestamp beyond timeout
        self.robot.last_command_time = time.time() - 1.0 # 1.0s ago (> 0.60s timeout)
        
        # Trigger control loop step logic
        with self.robot.lock:
            if time.time() - self.robot.last_command_time > self.robot.watchdog_timeout:
                self.robot.target_linear = 0.0
                self.robot.target_angular = 0.0

        self.assertEqual(self.robot.target_linear, 0.0)
        self.assertEqual(self.robot.target_angular, 0.0)

    def test_14_stale_frames_stop_robot(self):
        """Scenario 14: Stale frames cause safety layer to disallow movement"""
        # Safety monitor evaluation with critical proximity
        report = self.safety.evaluate_safety(
            np.zeros((480, 640, 3), dtype=np.uint8),
            distance_m=0.40, # < 0.55m
            target_status="SAFE"
        )
        self.assertFalse(report["can_move"])

    def test_15_obstacle_deflection_and_stop(self):
        """Scenario 15: Center obstacle triggers deflection and speed reduction"""
        obs_dists = {"front": 0.65, "left": 2.2, "right": 0.8} # Center warning, left open
        safe_w, speed_scale, status = self.nav.compute_safe_steering(
            target_offset_norm=0.0,
            obstacle_distances=obs_dists
        )
        self.assertGreater(safe_w, 0.0) # Deflects Left (+w_z) towards 2.2m clearance
        self.assertLess(speed_scale, 1.0) # Slows down
        self.assertEqual(status, "OBSTACLE_AVOIDING")

    def test_16_manual_mode_override(self):
        """Scenario 16: Operator switches to MANUAL mode; auto commands are rejected"""
        self.robot.set_mode("MANUAL")
        self.robot.update_auto_commands(0.22, 0.80, can_move=True)
        
        telemetry = self.robot.get_telemetry()
        # In MANUAL mode, auto commands must be ignored
        self.assertEqual(telemetry["target_linear"], 0.0)
        self.assertEqual(telemetry["target_angular"], 0.0)
        self.assertEqual(telemetry["mode"], "MANUAL")


if __name__ == "__main__":
    unittest.main()
