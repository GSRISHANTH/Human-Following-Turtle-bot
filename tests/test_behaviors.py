"""Behaviours through the real CVPipeline.process_frame: close-range wait/resume, LiDAR-only tracking,
predictive search, view learning, and the safety overrides. Frames are blank, so the camera sees nobody;
the person exists only in the LiDAR scan, which is exactly the close-range blind-spot situation."""

import math
import sys
import time
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT, ROOT / "pi", ROOT / "laptop"):
    sys.path.insert(0, str(p))

from config import Config  # noqa: E402
from cv_pipeline import CVPipeline  # noqa: E402
from tests.test_lidar import bins_with  # noqa: E402

BLANK = np.zeros((480, 640, 3), dtype=np.uint8)


class Behaviours(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pipe = CVPipeline()

    def setUp(self):
        p = self.pipe
        p.owner_enrolled = True
        p.owner_profile = {"protected_template": {}, "templates": []}
        p.following_enabled = True
        p.base_target_distance = 1.2
        p.robot.set_mode("AUTO")
        p.robot.emergency_stop = False
        p.target_state = Config.STATE_FOLLOWING
        p.target_lost_time = None
        p.lidar_only = False
        p.ekf.reset()
        p.follow.reset()
        p.last_ctl_t = None
        p.plan, p.plan_time = None, 0.0
        p.memory_done, p.search_t0 = False, None
        p.lidar_ranges = None

    def track_at(self, rng, bearing_deg=0.0):
        p = self.pipe
        now = time.time()
        p.ekf.init(rng, math.radians(bearing_deg), now)
        for i in range(8):
            p.ekf.predict(now + 0.05 * (i + 1))
            p.ekf.update(rng, math.radians(bearing_deg), 0.05, math.radians(1.0))
        p.last_cam_seen = time.time()

    def frame(self, lidar_obstacles):
        p = self.pipe
        p.set_lidar(bins_with(lidar_obstacles))
        tel, _ = p.process_frame(BLANK.copy())
        return tel

    def test_close_range_waits_on_lidar_and_does_not_search(self):
        p = self.pipe
        self.track_at(0.9)
        for _ in range(12):
            tel = self.frame({0: 0.9})
        self.assertEqual(tel["target_state"], Config.STATE_WAIT_FOR_OWNER_MOVEMENT)
        self.assertTrue(tel["fusion"]["lidar_only"])
        self.assertEqual(tel["fusion"]["mode"], "LIDAR")
        self.assertNotEqual(tel["target_state"], Config.STATE_SEARCHING)
        self.assertEqual(tel["phase2_cmd_vel"]["linear_x"], 0.0, "must stand still at the wait distance")

    def test_resumes_following_the_same_person_when_they_walk_away(self):
        self.track_at(0.9)
        for _ in range(6):
            tel = self.frame({0: 0.9})
        self.assertEqual(tel["target_state"], Config.STATE_WAIT_FOR_OWNER_MOVEMENT)
        for i in range(30):                       # walks away at about 0.5 m/s of frame-to-frame motion
            tel = self.frame({0: 0.9 + 0.03 * (i + 1)})
        self.assertEqual(tel["target_state"], Config.STATE_FOLLOWING, tel["nav_status"])
        self.assertGreater(tel["phase2_cmd_vel"]["linear_x"], 0.0)

    def test_follow_distance_is_configurable(self):
        p = self.pipe
        p.base_target_distance = 2.0
        stop, resume = p._stop_resume()
        self.assertAlmostEqual(stop, 1.9)
        self.assertAlmostEqual(resume, 2.15)
        p.base_target_distance = 0.1
        self.assertGreaterEqual(p._stop_resume()[0], 0.4)  # never below the floor
        p.base_target_distance = 1.2

    def test_lost_person_is_searched_for_where_they_were_heading(self):
        p = self.pipe
        for bearing, expect_sign in ((-30.0, -1), (30.0, 1)):
            self.track_at(1.5, bearing)
            p.target_state = Config.STATE_FOLLOWING
            p.target_lost_time = None
            cmd = None
            for _ in range(10):
                tel = self.frame({})
                cmd = tel["phase2_cmd_vel"]["angular_z"]
                time.sleep(0.02)
                p.target_lost_time = (p.target_lost_time or time.time()) - 0.5  # fast-forward past the memory-plan grace
            self.assertEqual(tel["nav_status"].split("_")[0], "SEARCH", tel["nav_status"])
            self.assertEqual(np.sign(cmd), expect_sign, f"owner left toward {bearing} deg but robot turned {cmd:+.2f}")

    def test_search_sweeps_wider_over_time_then_halts(self):
        p = self.pipe
        self.track_at(1.5, 20.0)
        p.target_state = Config.STATE_FOLLOWING
        p.target_lost_time = time.time() - 0.5
        seen = set()
        for age in (0.6, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0):
            p.target_lost_time = time.time() - age
            p.ekf.s[:2] = [1.5, 0.0]       # keep the prediction fixed so only the sweep offset moves the command
            v, w, status = p._search_command(time.time())
            seen.add((round(w, 1), status))
        self.assertGreater(len({w for w, _ in seen}), 2, "sweep must change direction and amplitude")
        p.target_lost_time = time.time() - (Config.SEARCH_MAX_DURATION_SEC + 1.0)
        self.assertEqual(p._search_command(time.time())[2], "SEARCH_TIMEOUT_HALT")

    def run_search(self, bearing_deg):
        """Drive _search_command with a simulated robot yaw and return (covered 10-degree headings, statuses, scan signs)."""
        p = self.pipe
        self.track_at(1.5, bearing_deg)
        p.lidar_ranges = None
        p.target_lost_time = 100.0
        p._search_t = None
        p.search_scan_rot = 0.0
        yaw, covered, statuses, scan_signs = 0.0, set(), [], []
        dt, now = 0.05, 100.0
        p.ekf.last_t = now
        for _ in range(int(45 / dt)):
            now += dt
            p.ekf.coast(now, 0.0, p.robot.current_angular)
            v, w, status = p._search_command(now)
            p.robot.current_angular = w
            yaw += w * dt
            covered.add(int((math.degrees(yaw) % 360) // 10))
            statuses.append(status)
            if status.startswith("SEARCH_SCAN"):
                scan_signs.append(np.sign(w))
            if status in ("SEARCH_COMPLETE_360", "SEARCH_TIMEOUT_HALT"):
                break
        p.robot.current_angular = 0.0
        return covered, statuses, scan_signs

    def test_search_covers_a_full_360_and_then_stops(self):
        for bearing in (-40.0, 40.0, 0.0):
            covered, statuses, signs = self.run_search(bearing)
            self.assertEqual(statuses[-1], "SEARCH_COMPLETE_360", f"owner at {bearing} deg: {statuses[-1]}")
            self.assertEqual(len(covered), 36, f"owner at {bearing} deg: only {len(covered) * 10} of 360 degrees were looked at")
            self.assertEqual(len(set(signs)), 1, "the scan must keep turning one way, not reverse")
            order = [st.split("_")[1] for st in statuses if st != "HOLD_POSITION"]
            self.assertLess(order.index("PREDICTED"), order.index("SWEEP"))
            self.assertLess(order.index("SWEEP"), order.index("SCAN"))

    def test_search_scan_turns_toward_the_side_the_owner_left(self):
        _, _, signs = self.run_search(-35.0)
        self.assertEqual(set(signs), {-1.0})
        _, _, signs = self.run_search(35.0)
        self.assertEqual(set(signs), {1.0})

    # ---- memory pursuit: the owner is hidden (low camera / obstacle) and must be recalled from the map ----

    def lose(self, plan, ago=1.6):
        """Owner lost `ago` seconds ago, a plan from the main process is available."""
        p = self.pipe
        self.track_at(2.0, 0.0)
        p.target_state = Config.STATE_TARGET_TEMPORARILY_LOST     # the loss was already noticed `ago` seconds back
        p.target_lost_time = time.time() - ago
        p.last_cam_seen = time.time() - ago
        p.set_plan(plan)
        p.memory_done, p.search_t0 = False, None
        p.lidar_ranges = None

    def mem_plan(self, **kw):
        plan = {"blocked": False, "wp_bearing": None, "status": "DIRECT", "memory": True, "mem_bearing": 0.0,
                "mem_range": 2.0, "goal_dist": 0.8, "age": 3.0}
        plan.update(kw)
        return plan

    def test_hidden_owner_is_recalled_not_searched_for(self):
        p = self.pipe
        self.lose(self.mem_plan())
        tel = self.frame({})
        self.assertEqual(p.target_state, Config.STATE_RECOVERING)
        self.assertEqual(tel["nav_status"], "MEMORY_PURSUIT")
        self.assertGreater(tel["phase2_cmd_vel"]["linear_x"], 0.0, "must drive toward the remembered position")
        self.assertTrue(tel["recalling"])
        self.assertNotEqual(p.target_state, Config.STATE_SEARCHING)

    def test_recall_goes_round_an_obstacle_via_the_waypoint(self):
        p = self.pipe
        self.lose(self.mem_plan(blocked=True, status="DETOUR", wp_bearing=0.7, mem_bearing=0.0))
        tel = self.frame({})
        self.assertEqual(tel["nav_status"], "MEMORY_DETOUR")
        self.assertGreater(tel["phase2_cmd_vel"]["angular_z"], 0.0, "steers at the waypoint (left), not at the hidden owner (dead ahead)")
        self.lose(self.mem_plan(blocked=True, status="DETOUR", wp_bearing=-0.7))
        self.assertLess(self.frame({})["phase2_cmd_vel"]["angular_z"], 0.0)

    def test_recall_never_drives_into_something_close(self):
        p = self.pipe
        self.lose(self.mem_plan())
        tel = self.frame({0: 0.25})          # a box right in front of the robot
        self.assertEqual(tel["phase2_cmd_vel"]["linear_x"], 0.0)
        self.assertEqual(tel["nav_status"], "LIDAR_STOP")

    def test_recall_turns_before_driving_when_the_spot_is_off_to_the_side(self):
        p = self.pipe
        self.lose(self.mem_plan(mem_bearing=1.4))        # 80 degrees to the left
        tel = self.frame({})
        self.assertEqual(tel["phase2_cmd_vel"]["linear_x"], 0.0)
        self.assertGreater(tel["phase2_cmd_vel"]["angular_z"], 0.0)

    def test_arriving_without_finding_them_starts_the_360_search_fresh(self):
        p = self.pipe
        self.lose(self.mem_plan(status="AT_GOAL", goal_dist=0.0), ago=2.0)
        tel = self.frame({})
        self.assertNotEqual(p.target_state, Config.STATE_RECOVERING)
        self.assertTrue(p.memory_done)
        self.assertTrue(tel["nav_status"].startswith("SEARCH_") or tel["nav_status"] == "HOLD_POSITION", tel["nav_status"])
        self.assertLess(p.search_t0 - time.time(), 0.5)           # the search clock restarted just now, not at the loss

    def test_no_path_or_no_map_falls_back_to_search_after_a_short_grace(self):
        p = self.pipe
        self.lose(self.mem_plan(status="NO_PATH"), ago=2.0)
        tel = self.frame({})
        self.assertTrue(p.memory_done)
        self.lose(self.mem_plan(), ago=0.7)
        p.plan = None                                              # the planner has not produced anything yet
        tel = self.frame({})
        self.assertEqual(tel["nav_status"], "MEMORY_PLANNING")     # waits, does not start spinning
        self.assertEqual((tel["phase2_cmd_vel"]["linear_x"], tel["phase2_cmd_vel"]["angular_z"]), (0.0, 0.0))
        p.target_lost_time = time.time() - 3.0
        p.last_cam_seen = time.time() - 3.0
        tel = self.frame({})
        self.assertTrue(p.memory_done)

    def test_pursuit_times_out(self):
        p = self.pipe
        self.lose(self.mem_plan(), ago=Config.MEMORY_PURSUIT_SEC + 1.0)
        self.frame({})
        self.assertTrue(p.memory_done)

    def test_lidar_finds_them_near_the_remembered_spot_and_following_resumes(self):
        p = self.pipe
        self.lose(self.mem_plan())
        tel = self.frame({})
        self.assertEqual(p.target_state, Config.STATE_RECOVERING)
        for _ in range(3):                                         # the owner steps out from behind the obstacle
            tel = self.frame({0: 2.1})
        self.assertTrue(tel["fusion"]["lidar_only"])
        self.assertIn(p.target_state, (Config.STATE_FOLLOWING, Config.STATE_WAIT_FOR_OWNER_MOVEMENT))
        self.assertTrue(any(e["type"] == "MEMORY_RELOCK" for e in p.event_log))
        self.assertGreater(time.time() - p.last_cam_seen, 5.0)     # leash set so the approach can be confirmed by camera
        self.assertLess(time.time() - p.last_cam_seen, Config.LIDAR_TRACK_MAX_SEC)

    def test_a_far_away_person_is_not_taken_for_the_hidden_owner(self):
        p = self.pipe
        self.lose(self.mem_plan(), ago=1.6)
        tel = self.frame({40: 2.0})                                # someone 40 degrees away from where the owner should be
        self.assertFalse(tel["fusion"]["lidar_only"])
        self.assertEqual(p.target_state, Config.STATE_RECOVERING)

    def test_lidar_peek_rejects_a_different_person_elsewhere(self):
        p = self.pipe
        self.track_at(1.2, 0.0)
        p.set_lidar(bins_with({25: 1.2}))   # someone else, 25 degrees away
        self.assertIsNone(p._lidar_peek(time.time()))
        p.set_lidar(bins_with({0: 1.25}))
        self.assertIsNotNone(p._lidar_peek(time.time()))

    def test_lidar_tracking_expires_if_the_camera_never_confirms(self):
        p = self.pipe
        self.track_at(1.2, 0.0)
        p.set_lidar(bins_with({0: 1.2}))
        p.last_cam_seen = time.time() - Config.LIDAR_TRACK_MAX_SEC - 1.0
        self.assertIsNone(p._lidar_peek(time.time()))

    def test_view_learning_only_when_the_owner_is_alone(self):
        p = self.pipe
        p.owner_profile = {"protected_template": {"hist": 1}, "templates": [{"hist": 1}]}
        p.reid = {"state": "TRACKING"}
        sig = {"hist": np.ones(4, dtype=np.float32)}
        owner = {"signature": sig, "centroid": (200, 240), "similarity_to_owner": 0.35}
        other = {"signature": sig, "centroid": (260, 240), "similarity_to_owner": 0.1}
        p.last_template_t = 0.0
        p._learn_view(owner, [owner, other], time.time(), 480)   # bystander right next to them: nothing learned
        self.assertEqual(len(p.owner_profile["templates"]), 1)
        p._learn_view(owner, [owner], time.time(), 480)          # alone: their back/side view is kept
        self.assertEqual(len(p.owner_profile["templates"]), 2)

    # ---- safety overrides ----

    def test_lidar_hard_stop_beats_any_command(self):
        rc = self.pipe.robot
        rc.set_mode("AUTO")
        rc.update_auto_commands(0.2, 0.0, can_move=True)
        rc.set_front_clearance(0.12, time.time())
        time.sleep(0.2)
        self.assertEqual(rc.get_telemetry()["current_linear"], 0.0)
        rc.set_front_clearance(None, 0.0)

    def test_estop_zeroes_and_blocks_commands_until_released(self):
        rc = self.pipe.robot
        rc.set_mode("MANUAL")
        rc.update_manual_teleop(0.2, 0.0)
        rc.trigger_emergency_stop()
        rc.update_manual_teleop(0.2, 0.0)
        time.sleep(0.15)
        tel = rc.get_telemetry()
        self.assertTrue(tel["emergency_stop"])
        self.assertEqual((tel["current_linear"], tel["target_linear"]), (0.0, 0.0))
        rc.release_emergency_stop()
        self.assertFalse(rc.get_telemetry()["emergency_stop"])

    def test_commands_expire_by_themselves(self):
        rc = self.pipe.robot
        rc.set_mode("MANUAL")
        rc.update_manual_teleop(0.15, 0.0)
        time.sleep(rc.watchdog_timeout + 0.25)
        self.assertEqual(rc.get_telemetry()["target_linear"], 0.0)

    def test_tests_never_transmit_to_a_robot(self):
        """Guard for the test suite itself: nothing may leave the machine while tests drive the controller."""
        rc = self.pipe.robot
        sent = []

        class Spy:
            def sendto(self, *a, **k):
                sent.append(a)

        real = rc.udp_sock
        rc.udp_sock = Spy()
        try:
            self.assertEqual(rc.protocol, "mock")
            rc.set_mode("MANUAL")
            rc.update_manual_teleop(0.2, 1.0)
            time.sleep(0.2)
            rc.update_manual_teleop(0.0, 0.0)
        finally:
            rc.udp_sock = real
        self.assertEqual(sent, [], "a test sent UDP to the robot")

    def test_faster_limits_are_applied_and_bounded(self):
        rc = self.pipe.robot
        rc.set_mode("MANUAL")
        rc.update_manual_teleop(5.0, 9.0)
        self.assertLessEqual(rc.get_telemetry()["target_linear"], 0.22)       # hardware limit of the Burger
        self.assertLessEqual(rc.get_telemetry()["target_angular"], Config.MAX_ANGULAR_VEL)
        self.assertGreaterEqual(Config.MAX_ANGULAR_VEL, 2.0)
        rc.update_manual_teleop(0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
