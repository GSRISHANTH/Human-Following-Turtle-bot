"""State machine and HRI through the real CVPipeline.process_frame, with people injected in place of MediaPipe.

The owner wears red, the bystander blue. Each test drives whole frames, so it checks what the robot would really do:
who may command it, that STOP latches, that an ambiguous scene holds still instead of searching, that a restored
profile does not drive on boot, and that the reported state is the one after this frame's transitions.
"""

import math
import os
import tempfile
import time
import unittest

import numpy as np

from config import Config
from cv_pipeline import CVPipeline
from tests.test_lidar import bins_with
from tests.test_system import create_mock_landmark, create_valid_human_landmarks

W, H = 640, 480


def scene(colors):
    """Frame with one shirt-coloured block per person: colors = [(x_centre_px, (b, g, r)), ...]."""
    img = np.full((H, W, 3), 40, dtype=np.uint8)
    for cx, col in colors:
        img[120:400, cx - 60:cx + 60] = col
    return img


def gesture_landmarks(kind, offset_x=0.0):
    lms = create_valid_human_landmarks(offset_x=offset_x)
    cx = 0.5 + offset_x
    if kind == "RELEASE":
        lms[15] = create_mock_landmark(cx + 0.06, 0.38)
        lms[16] = create_mock_landmark(cx - 0.06, 0.38)
    elif kind == "STOP_HALT":
        lms[15] = create_mock_landmark(cx - 0.14, 0.10)
        lms[16] = create_mock_landmark(cx + 0.14, 0.10)
    elif kind == "FOLLOW_ME":
        lms[16] = create_mock_landmark(cx + 0.15, 0.10)
    elif kind == "RAISE":                               # hand above the shoulder, below the head
        lms[16] = create_mock_landmark(cx + 0.15, 0.26)
        lms[14] = create_mock_landmark(cx + 0.16, 0.36)
    return lms


class _Harness(unittest.TestCase):
    """Shared pipeline + injected-people harness (no tests of its own)."""
    @classmethod
    def setUpClass(cls):
        cls.pipe = CVPipeline()
        cls.pipe.render_hud = False

    def setUp(self):
        p = self.pipe
        self.owner_img = scene([(200, (30, 30, 220))])
        self.owner_sig = p.appearance.extract_signature(self.owner_img, [140, 120, 260, 400])
        p.owner_profile = p.appearance.create_owner_profile(self.owner_sig)
        p.owner_enrolled = True
        p.following_enabled = True
        p.safety_latched = False
        p.ambiguous = False
        p.base_target_distance = 1.2
        p.robot.set_mode("AUTO")
        p.target_state = Config.STATE_FOLLOWING
        p.target_lost_time = None
        p.lidar_only = False
        p.lidar_ranges = None
        p.ekf.reset()
        p.follow.reset()
        p.last_ctl_t = None
        p.gestures.feed("NONE")
        p.any_stop.feed("NONE")
        p.event_log.clear()
        p.smooth_offset_x = None          # no previous image position: identity alone decides
        self._orig_detect = p._detect_all_persons

    def tearDown(self):
        self.pipe._detect_all_persons = self._orig_detect
        self.pipe.robot.set_mode("AUTO")

    def person(self, cx, color, gesture=None, dist=2.0):
        img = scene([(cx, color)])
        bbox = [cx - 60, 120, cx + 60, 400]
        return {"id": cx, "bbox": bbox, "centroid": (cx, 260), "distance_m": dist,
                "signature": self.pipe.appearance.extract_signature(img, bbox),
                "landmarks": gesture_landmarks(gesture or "NONE", offset_x=(cx - W / 2) / W),
                "pose_norm": [], "is_target": False, "confidence": 0.95}

    def run_frames(self, make_people, n):
        p = self.pipe
        tels = []
        for _ in range(n):
            people = make_people()
            p._detect_all_persons = lambda frame, annotated, people=people: people
            tel, _ = p.process_frame(np.zeros((H, W, 3), dtype=np.uint8))
            tels.append(tel)
        return tels

    RED, BLUE = (30, 30, 220), (220, 60, 30)



class PipelineStates(_Harness):
    def test_bystander_cannot_release_the_owner(self):
        tels = self.run_frames(lambda: [self.person(200, self.RED), self.person(460, self.BLUE, "RELEASE")], 10)
        self.assertTrue(self.pipe.owner_enrolled, "a bystander's crossed arms must not release the owner")
        self.assertTrue(any(e["type"] == "GESTURE_IGNORED" for e in self.pipe.event_log))
        self.assertNotEqual(tels[-1]["target_state"], Config.STATE_OWNER_RELEASED)

    def test_owner_can_release(self):
        self.run_frames(lambda: [self.person(200, self.RED, "RELEASE"), self.person(460, self.BLUE)], 10)
        self.assertFalse(self.pipe.owner_enrolled)

    def test_stop_from_anyone_latches_until_owner_says_follow(self):
        tels = self.run_frames(lambda: [self.person(200, self.RED), self.person(460, self.BLUE, "STOP_HALT")], 8)
        self.assertTrue(self.pipe.safety_latched)
        self.assertEqual(tels[-1]["target_state"], Config.STATE_SAFETY_STOP)
        # hands come down: still stopped (latched), and nothing moves
        tels = self.run_frames(lambda: [self.person(200, self.RED), self.person(460, self.BLUE)], 10)
        self.assertEqual(tels[-1]["target_state"], Config.STATE_SAFETY_STOP)
        self.assertEqual(tels[-1]["phase2_cmd_vel"], {"linear_x": 0.0, "angular_z": 0.0})
        self.assertEqual(self.pipe.robot.target_linear, 0.0)
        # a bystander's FOLLOW_ME does not clear it
        self.run_frames(lambda: [self.person(200, self.RED), self.person(460, self.BLUE, "FOLLOW_ME")], 8)
        self.assertTrue(self.pipe.safety_latched)
        # the owner's does
        self.run_frames(lambda: [self.person(200, self.RED, "FOLLOW_ME"), self.person(460, self.BLUE)], 8)
        self.assertFalse(self.pipe.safety_latched)
        tels = self.run_frames(lambda: [self.person(200, self.RED), self.person(460, self.BLUE)], 4)
        self.assertEqual(tels[-1]["target_state"], Config.STATE_FOLLOWING)

    def test_two_lookalikes_hold_still_and_never_search(self):
        tels = self.run_frames(lambda: [self.person(200, self.RED), self.person(460, self.RED)], 40)
        states = {t["target_state"] for t in tels}
        self.assertEqual(states, {Config.STATE_TARGET_AMBIGUOUS}, states)
        for bad in (Config.STATE_SEARCHING, Config.STATE_RECOVERING, Config.STATE_TARGET_TEMPORARILY_LOST):
            self.assertNotIn(bad, states)
        self.assertTrue(all(t["phase2_cmd_vel"]["angular_z"] == 0.0 and t["phase2_cmd_vel"]["linear_x"] == 0.0 for t in tels))
        self.assertEqual(sum(1 for e in self.pipe.event_log if e["type"] == "TARGET_AMBIGUOUS"), 1, "log once, not per frame")
        # the look-alike walks off: following resumes on the owner
        tels = self.run_frames(lambda: [self.person(200, self.RED)], 6)
        self.assertEqual(tels[-1]["target_state"], Config.STATE_FOLLOWING)

    def test_lookalike_next_to_a_tracked_owner_does_not_steal_the_track(self):
        """With a live track, position continuity separates two look-alikes: the one where the owner was wins."""
        self.run_frames(lambda: [self.person(200, self.RED)], 4)
        tels = self.run_frames(lambda: [self.person(200, self.RED), self.person(460, self.RED)], 10)
        self.assertEqual(tels[-1]["target_state"], Config.STATE_FOLLOWING)
        owner = [o for o in tels[-1]["overlay"] if o["owner"]]
        self.assertEqual(len(owner), 1)
        self.assertLess(owner[0]["bbox"][0], 0.5, "must stay on the person on the left, where the owner was")

    def test_reported_state_is_after_this_frames_transition(self):
        p = self.pipe
        p.set_lidar(bins_with({0: 0.9}))
        now = time.time()
        p.ekf.init(0.9, 0.0, now)
        p.last_cam_seen = now
        p.target_state = Config.STATE_FOLLOWING
        tel, _ = p.process_frame(np.zeros((H, W, 3), dtype=np.uint8))
        self.assertEqual(tel["target_state"], p.target_state)
        self.assertEqual(tel["target_state"], Config.STATE_WAIT_FOR_OWNER_MOVEMENT)

    def test_co_mapped_lidar_range_per_person(self):
        """The LiDAR beams inside the owner's box give their range, shown next to the camera's estimate."""
        p = self.pipe
        bearing = math.degrees(-math.atan((200 - W / 2) / ((W / 2) / math.tan(math.radians(Config.CAMERA_HFOV_DEG) / 2))))
        p.set_lidar(bins_with({round(bearing): 1.8}))
        tels = self.run_frames(lambda: [self.person(200, self.RED, dist=1.9)], 3)
        ov = tels[-1]["overlay"][0]
        self.assertIsNotNone(ov["lidar_m"])
        self.assertAlmostEqual(ov["lidar_m"], 1.8, delta=0.05)
        self.assertTrue(tels[-1]["lidar_cam"], "projected beams must be sent for the overlay")
        self.assertTrue(all(0.0 <= pt[0] <= 1.0 for pt in tels[-1]["lidar_cam"]))


class HoldToFollow(_Harness):
    """Drive only while the owner's hand is up; hand down = stay and keep them centred; search only when the camera
    loses them (LiDAR-guided first); found again = stop searching, centre, wait for the hand."""

    def setUp(self):
        super().setUp()
        self._saved = (Config.FOLLOW_REQUIRES_RAISED_HAND, Config.FOLLOW_RELEASE_SEC)
        Config.FOLLOW_REQUIRES_RAISED_HAND, Config.FOLLOW_RELEASE_SEC = True, 0.0
        p = self.pipe
        p.hand_raised, p._raise_n, p._raised_at_loss = False, 0, False
        p.memory_done, p.search_t0, p.plan = False, None, None

    def tearDown(self):
        Config.FOLLOW_REQUIRES_RAISED_HAND, Config.FOLLOW_RELEASE_SEC = self._saved
        super().tearDown()

    def test_hand_down_stays_and_centres_never_searches(self):
        tels = self.run_frames(lambda: [self.person(200, self.RED)], 12)
        self.assertEqual({t["target_state"] for t in tels[2:]}, {Config.STATE_AWAITING_GESTURE})
        self.assertTrue(all(t["phase2_cmd_vel"]["linear_x"] == 0.0 for t in tels))
        self.assertGreater(tels[-1]["phase2_cmd_vel"]["angular_z"], 0.0, "owner left of centre: turn left to centre them")

    def test_hand_up_follows_hand_down_stops(self):
        tels = self.run_frames(lambda: [self.person(320, self.RED, "RAISE")], 6)
        self.assertEqual(tels[-1]["target_state"], Config.STATE_FOLLOWING)
        self.assertGreater(tels[-1]["phase2_cmd_vel"]["linear_x"], 0.0)
        tels = self.run_frames(lambda: [self.person(320, self.RED)], 3)
        self.assertEqual(tels[-1]["target_state"], Config.STATE_AWAITING_GESTURE)
        self.assertEqual(tels[-1]["phase2_cmd_vel"]["linear_x"], 0.0)
        self.assertTrue(self.pipe.owner_enrolled and self.pipe.following_enabled, "lowering the hand is not a pause")

    def test_a_bystanders_raised_hand_does_not_drive(self):
        tels = self.run_frames(lambda: [self.person(200, self.RED), self.person(460, self.BLUE, "RAISE")], 8)
        self.assertEqual(tels[-1]["target_state"], Config.STATE_AWAITING_GESTURE)
        self.assertEqual(tels[-1]["phase2_cmd_vel"]["linear_x"], 0.0)

    def test_out_of_camera_lidar_guides_the_turn_and_never_drives_blind(self):
        p = self.pipe
        for raised, drives in ((False, False), (True, False)):
            p.hand_raised, p.lidar_only = raised, False
            p.target_state = Config.STATE_FOLLOWING if raised else Config.STATE_AWAITING_GESTURE
            p.set_lidar(bins_with({-40: 2.4}))             # owner walked off to the right, out of the 57 deg view
            now = time.time()
            p.ekf.init(2.4, math.radians(-40), now)
            p.last_cam_seen = now
            tels = self.run_frames(lambda: [], 3)
            self.assertEqual(tels[-1]["target_state"], Config.STATE_LIDAR_GUIDED, tels[-1]["nav_status"])
            self.assertLess(tels[-1]["phase2_cmd_vel"]["angular_z"], 0.0, "turn right, toward the LiDAR position")
            self.assertEqual(tels[-1]["phase2_cmd_vel"]["linear_x"] > 0.0, drives)

    def test_found_again_during_search_stops_and_waits_for_the_hand(self):
        p = self.pipe
        p.hand_raised = False
        p.target_state = Config.STATE_SEARCHING
        p.target_lost_time = time.time() - 5.0
        p.memory_done = True
        tels = self.run_frames(lambda: [self.person(150, self.RED)], 4)      # left of centre (frames are 480 wide)
        self.assertEqual(tels[-1]["target_state"], Config.STATE_AWAITING_GESTURE)
        self.assertEqual(tels[-1]["phase2_cmd_vel"]["linear_x"], 0.0)
        self.assertGreater(tels[-1]["phase2_cmd_vel"]["angular_z"], 0.0, "centres the owner instead of sweeping on")


class RestoredProfile(unittest.TestCase):
    def test_saved_profile_never_drives_on_boot(self):
        from appearance_model import AppearanceModel
        tmp = tempfile.mkdtemp()
        path = os.path.join(tmp, "owner.pkl")
        am = AppearanceModel()
        img = scene([(320, (30, 30, 220))])
        am.save_profile(am.create_owner_profile(am.extract_signature(img, [260, 120, 380, 400])), path)
        old = Config.OWNER_PROFILE_PATH
        Config.OWNER_PROFILE_PATH = path
        try:
            p = CVPipeline()
        finally:
            Config.OWNER_PROFILE_PATH = old
        self.assertTrue(p.owner_enrolled)
        self.assertFalse(p.following_enabled, "an old profile must wait for FOLLOW_ME / START")
        p.robot.set_mode("STOP")


if __name__ == "__main__":
    unittest.main()
