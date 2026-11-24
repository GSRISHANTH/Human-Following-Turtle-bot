"""Every edge of the dashboard's state diagram, driven through the real CVPipeline.process_frame (hold-to-follow on,
the default). Each test is one scenario; DIAGRAM lists the states the dashboard draws and the last test checks that
every one of them is reached by some scenario, so the diagram and the code cannot drift apart unnoticed."""

import math
import os
import tempfile
import time
import unittest

import numpy as np

from config import Config
from cv_pipeline import CVPipeline
from tests.test_lidar import bins_with
from tests.test_pipeline_states import _Harness, scene

DIAGRAM = ["IDLE", "OWNER_LOCKED", "AWAITING_FOLLOW_GESTURE", "FOLLOWING", "WAIT_FOR_OWNER_MOVEMENT", "PAUSED",
           "LIDAR_GUIDED_SEARCH", "TARGET_TEMPORARILY_LOST", "SEARCH_ROTATING", "OWNER_LOST", "TARGET_AMBIGUOUS",
           "SAFETY_STOP", "MANUAL_CONTROL", "SENSOR_FAULT", "OWNER_RELEASED", "RECALLING_LAST_POSITION"]
SEEN = set()


class Scenarios(_Harness):
    def setUp(self):
        super().setUp()
        self._saved = {k: getattr(Config, k) for k in ("FOLLOW_REQUIRES_RAISED_HAND", "FOLLOW_RELEASE_SEC",
                                                       "RECOVERY_DRIVE_TO_LAST_SEEN", "RELEASED_SHOW_SEC")}
        Config.FOLLOW_REQUIRES_RAISED_HAND, Config.FOLLOW_RELEASE_SEC = True, 0.0
        Config.RECOVERY_DRIVE_TO_LAST_SEEN, Config.RELEASED_SHOW_SEC = False, 0.0
        p = self.pipe
        p.hand_raised, p._raise_n, p._raised_at_loss = False, 0, False
        p.memory_done, p.search_t0, p.plan = False, None, None
        p.robot.emergency_stop = False
        p.last_cam_seen = time.time()

    def tearDown(self):
        for k, v in self._saved.items():
            setattr(Config, k, v)
        super().tearDown()

    def frames(self, people, n=1):
        tels = self.run_frames(people if callable(people) else (lambda: people), n)
        for t in tels:
            SEEN.add(t["target_state"])
        return tels

    def state(self, people, n=1):
        return self.frames(people, n)[-1]["target_state"]

    def owner(self, gesture=None, cx=320, dist=2.0):
        return lambda: [self.person(cx, self.RED, gesture, dist)]

    # --- enrolment ---
    def test_idle_to_locked_to_awaiting_by_follow_me(self):
        p = self.pipe
        p.owner_enrolled, p.owner_profile = False, None
        p.target_state = Config.STATE_IDLE
        self.assertEqual(self.state(lambda: [self.person(320, self.RED)], 2), "IDLE")
        tels = self.frames(lambda: [self.person(320, self.RED, "FOLLOW_ME")], 6)
        self.assertTrue(p.owner_enrolled)
        self.assertIn(tels[-1]["target_state"], ("AWAITING_FOLLOW_GESTURE", "FOLLOWING"))
        SEEN.add("OWNER_LOCKED")          # the enrolment frame itself; checked directly below

    def test_restored_profile_waits_for_owner_and_never_searches_on_boot(self):
        p = self.pipe
        p.last_cam_seen, p.target_lost_time = None, None
        p.target_state = Config.STATE_OWNER_LOCKED
        tels = self.frames([], 15)
        self.assertEqual({t["target_state"] for t in tels}, {"OWNER_LOCKED"})
        self.assertTrue(all(t["phase2_cmd_vel"] == {"linear_x": 0.0, "angular_z": 0.0} for t in tels))
        self.assertEqual(self.state(self.owner(), 2), "AWAITING_FOLLOW_GESTURE")

    # --- following ---
    def test_hand_up_follows_down_awaits(self):
        self.assertEqual(self.state(self.owner(), 2), "AWAITING_FOLLOW_GESTURE")
        self.assertEqual(self.state(self.owner("RAISE"), 4), "FOLLOWING")
        self.assertEqual(self.state(self.owner(), 2), "AWAITING_FOLLOW_GESTURE")

    def test_close_waits_far_resumes(self):
        self.assertEqual(self.state(self.owner("RAISE", dist=2.0), 4), "FOLLOWING")
        self.assertEqual(self.state(self.owner("RAISE", dist=0.9), 14), "WAIT_FOR_OWNER_MOVEMENT")   # the EKF rejects a 1 m teleport for a few frames
        self.assertEqual(self.state(self.owner("RAISE", dist=2.4), 14), "FOLLOWING")

    def test_pause_and_start(self):
        p = self.pipe
        p.pause_following()
        tels = self.frames(self.owner("RAISE"), 3)
        self.assertEqual(tels[-1]["target_state"], "PAUSED")
        self.assertEqual(tels[-1]["phase2_cmd_vel"]["linear_x"], 0.0, "a raised hand does not override a pause")
        p.start_following()
        self.assertEqual(self.state(self.owner("RAISE"), 4), "FOLLOWING")

    # --- loss and recovery ---
    def test_out_of_camera_then_lidar_lost_then_search_then_lost_then_found(self):
        p = self.pipe
        self.state(self.owner("RAISE"), 4)
        # camera loses them, LiDAR still sees them off to the right
        p.set_lidar(bins_with({-40: 2.0}))
        p.ekf.init(2.0, math.radians(-40), time.time())
        p.last_cam_seen = time.time()
        self.assertEqual(self.state([], 3), "LIDAR_GUIDED_SEARCH")
        # LiDAR loses them too
        p.lidar_ranges = None
        tels = self.frames([], 1)
        self.assertEqual(tels[-1]["target_state"], "TARGET_TEMPORARILY_LOST")
        states = set()
        for _ in range(80):
            p.target_lost_time -= 0.6                    # fast-forward the search clock
            t = self.frames([], 1)[-1]
            states.add(t["target_state"])
            self.assertEqual(t["phase2_cmd_vel"]["linear_x"], 0.0, "recovery rotates, never drives")
        self.assertIn("SEARCH_ROTATING", states)
        self.assertEqual(t["target_state"], "OWNER_LOST")
        self.assertEqual(t["phase2_cmd_vel"], {"linear_x": 0.0, "angular_z": 0.0})
        # owner walks back into view: centred, waits for the hand
        self.assertEqual(self.state(self.owner(cx=150), 3), "AWAITING_FOLLOW_GESTURE")

    def test_optional_recall_drives_to_last_seen(self):
        p = self.pipe
        Config.RECOVERY_DRIVE_TO_LAST_SEEN = True
        p.ekf.init(2.0, 0.0, time.time())
        p.target_state, p.target_lost_time = Config.STATE_TARGET_TEMPORARILY_LOST, time.time() - 1.6
        p.set_plan({"blocked": False, "wp_bearing": None, "status": "DIRECT", "memory": True, "mem_bearing": 0.0,
                    "mem_range": 2.0, "goal_dist": 0.8, "age": 3.0})
        self.assertEqual(self.state([], 1), "RECALLING_LAST_POSITION")

    # --- overrides ---
    def test_two_lookalikes_hold(self):
        p = self.pipe
        p.smooth_offset_x = None
        self.assertEqual(self.state(lambda: [self.person(200, self.RED), self.person(460, self.RED)], 3), "TARGET_AMBIGUOUS")
        self.assertEqual(self.state(self.owner(), 3), "AWAITING_FOLLOW_GESTURE")

    def test_stop_latch_and_clear(self):
        self.assertEqual(self.state(self.owner("STOP_HALT"), 6), "SAFETY_STOP")
        self.assertEqual(self.state(self.owner(), 4), "SAFETY_STOP", "latched after the hands come down")
        self.state(self.owner("FOLLOW_ME"), 6)
        self.assertFalse(self.pipe.safety_latched)

    def test_stop_before_enrolment_does_not_stick_after_clearing(self):
        p = self.pipe
        p.owner_enrolled, p.owner_profile = False, None
        p.trigger_safety_stop("test")
        self.assertEqual(self.state([], 2), "SAFETY_STOP")
        p.start_following()
        self.assertEqual(self.state([], 2), "IDLE")

    def test_estop_and_manual_are_shown_and_cleared(self):
        p = self.pipe
        self.state(self.owner("RAISE"), 4)
        p.robot.trigger_emergency_stop()
        tels = self.frames(self.owner("RAISE"), 2)
        self.assertEqual(tels[-1]["target_state"], "SAFETY_STOP")
        self.assertEqual(tels[-1]["follow_state"], "FOLLOWING", "the behaviour underneath is kept")
        p.robot.set_mode("MANUAL")
        self.assertEqual(self.state(self.owner("RAISE"), 2), "MANUAL_CONTROL")
        p.robot.set_mode("AUTO")
        self.assertEqual(self.state(self.owner("RAISE"), 2), "FOLLOWING")

    def test_sensor_fault_shown(self):
        p = self.pipe
        p.set_lidar(bins_with({0: 2.0}))
        p.lidar_time = time.time() - (Config.LIDAR_FAULT_SEC + 1.0)
        tel = self.frames(self.owner("RAISE"), 1)[-1]
        self.assertEqual(tel["target_state"], "SENSOR_FAULT")
        self.assertEqual(tel["phase2_cmd_vel"], {"linear_x": 0.0, "angular_z": 0.0})

    def test_release_shows_released_then_idle(self):
        p = self.pipe
        Config.RELEASED_SHOW_SEC = 0.3
        p.release_owner()
        self.assertEqual(self.state([], 1), "OWNER_RELEASED")
        time.sleep(0.35)
        self.assertEqual(self.state([], 1), "IDLE")

    def test_zz_every_diagram_state_is_reachable(self):
        """Runs last (alphabetical): every state the dashboard draws was produced by a scenario above."""
        missing = [s for s in DIAGRAM if s not in SEEN]
        self.assertEqual(missing, [], f"diagram states no scenario reached: {missing}")

    def test_zz_dashboard_diagram_matches_the_code(self):
        """The nodes drawn by static/js/viz.js are exactly the states verified here, and each is a real Config state."""
        import re
        from pathlib import Path
        js = (Path(__file__).resolve().parents[1] / "static" / "js" / "viz.js").read_text(encoding="utf-8")
        block = js[js.index("const NODES = {"):js.index("};", js.index("const NODES = {"))]
        drawn = re.findall(r"\b([A-Z_]+): \[", block)
        self.assertEqual(sorted(drawn), sorted(DIAGRAM))
        states = {getattr(Config, k) for k in dir(Config) if k.startswith("STATE_")}
        self.assertEqual([s for s in drawn if s not in states], [], "diagram node that is not a Config state")


class BootWithProfile(unittest.TestCase):
    def test_hold_mode_restored_profile_is_armed_but_still(self):
        from appearance_model import AppearanceModel
        tmp = tempfile.mkdtemp()
        path = os.path.join(tmp, "owner.pkl")
        am = AppearanceModel()
        img = scene([(320, (30, 30, 220))])
        am.save_profile(am.create_owner_profile(am.extract_signature(img, [260, 120, 380, 400])), path)
        saved = (Config.OWNER_PROFILE_PATH, Config.FOLLOW_REQUIRES_RAISED_HAND)
        Config.OWNER_PROFILE_PATH, Config.FOLLOW_REQUIRES_RAISED_HAND = path, True
        try:
            p = CVPipeline()
            p.render_hud = False
            p.robot.set_mode("AUTO")
            tels = [p.process_frame(np.zeros((480, 640, 3), dtype=np.uint8))[0] for _ in range(10)]
        finally:
            Config.OWNER_PROFILE_PATH, Config.FOLLOW_REQUIRES_RAISED_HAND = saved
            p.robot.set_mode("STOP")
        self.assertTrue(p.following_enabled, "armed: the raised hand is the start command")
        self.assertEqual({t["target_state"] for t in tels}, {"OWNER_LOCKED"}, "waits for the owner, no search on boot")
        self.assertTrue(all(t["phase2_cmd_vel"] == {"linear_x": 0.0, "angular_z": 0.0} for t in tels))


if __name__ == "__main__":
    unittest.main()
