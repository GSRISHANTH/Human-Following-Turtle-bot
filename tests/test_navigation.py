"""Localisation (scan matching), classification, planning, and the Re-ID gate."""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT, ROOT / "pi", ROOT / "laptop"):
    sys.path.insert(0, str(p))

import lidar  # noqa: E402
from navigation import Planner  # noqa: E402


def room_scan(pose, boxes=(), half=(2.5, 1.8), n=360):
    """Ray-cast a rectangular room (x in [-2.5, 2.5], y in [-1.8, 1.8]) plus axis-aligned boxes."""
    x0, y0, yaw = pose
    out = []
    for i in range(n):
        a = yaw + (i + 0.5) * 2 * math.pi / n
        dx, dy = math.cos(a), math.sin(a)
        best = 1e9
        for ax, hx, d, o, lim in ((0, half[0], dx, x0, half[1]), (1, half[1], dy, y0, half[0])):
            for wall in (-1, 1):
                if abs(d) > 1e-9:
                    t = (wall * (hx) - o) / d
                    if t > 0:
                        best = min(best, t)
        for (bx0, by0, bx1, by1) in boxes:
            tmin, tmax = 0.0, 1e9
            for o, d, lo, hi in ((x0, dx, bx0, bx1), (y0, dy, by0, by1)):
                if abs(d) < 1e-9:
                    if not lo <= o <= hi:
                        tmin, tmax = 1, 0
                else:
                    t1, t2 = (lo - o) / d, (hi - o) / d
                    tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
            if tmin <= tmax and tmin > 0:
                best = min(best, tmin)
        out.append(int(best * 1000) if best < 6 else 0)
    return out


class Localisation(unittest.TestCase):
    def test_scan_matching_pulls_drifted_odometry_back(self):
        m = lidar.LidarMap()
        box = [(0.8, -0.3, 1.1, 0.0)]
        for _ in range(40):
            m.update(room_scan((0.0, 0.0, 0.0), box), (0.0, 0.0, 0.0))
        true_pose = (0.3, 0.1, 0.05)
        drifted_odom = (0.42, -0.02, 0.0)  # 0.17 m / 3 degrees off
        for _ in range(12):
            m.update(room_scan(true_pose, box), drifted_odom)
        err_odom = math.hypot(drifted_odom[0] - true_pose[0], drifted_odom[1] - true_pose[1])
        err_loc = math.hypot(m.pose[0] - true_pose[0], m.pose[1] - true_pose[1])
        self.assertLess(err_loc, 0.6 * err_odom, f"localised error {err_loc:.3f} vs odometry {err_odom:.3f}")

    def test_perfect_odometry_is_left_alone(self):
        m = lidar.LidarMap()
        for _ in range(60):
            m.update(room_scan((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
        self.assertLess(math.hypot(m.pose[0], m.pose[1]), 0.04)

    def test_new_object_in_seen_free_space_is_moving_person_cluster_is_distinct(self):
        m = lidar.LidarMap()
        for _ in range(40):
            m.update(room_scan((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
        m.update(room_scan((0.0, 0.0, 0.0), [(1.0, -0.15, 1.3, 0.15)]), (0.0, 0.0, 0.0))
        labels = {tuple(np.round([c["cx"], c["cy"]], 1)): l for c, l in zip(m.clusters, m.labels)}
        self.assertIn("moving", labels.values())
        self.assertIn("wall", labels.values())

    def test_render_with_plan(self):
        m = lidar.LidarMap()
        for _ in range(30):
            m.update(room_scan((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
        plan = {"path": [(0.0, 0.0), (1.0, 0.5)], "goal": (1.0, 0.5)}
        img = m.render(person={"x": 1.5, "y": 0.0, "vx": 0.1, "vy": 0.0}, sec=lidar.sectors(m.bins), plan=plan)
        self.assertEqual(img.shape[2], 3)


class Planning(unittest.TestCase):
    def _map(self):
        m = lidar.LidarMap()
        wall = [(0.4, -1.8, 0.5, 0.45)]        # wall across the room with a gap on the left (y > 0.45)
        for _ in range(40):
            m.update(room_scan((0.0, 0.0, 0.0), wall), (0.0, 0.0, 0.0))
        return m

    def test_direct_when_clear(self):
        m = lidar.LidarMap()
        for _ in range(40):
            m.update(room_scan((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
        plan = Planner().plan(m.grid, m.origin, m.pose, (2.0, 0.0), 1.2)
        self.assertFalse(plan["blocked"])
        self.assertEqual(plan["status"], "DIRECT")

    def test_detours_round_a_wall_toward_the_gap(self):
        m = self._map()
        plan = Planner().plan(m.grid, m.origin, m.pose, (2.0, -0.5), 1.2)
        # person stands behind the wall, on the side without the gap; goal is behind it too
        self.assertTrue(plan["blocked"])
        self.assertEqual(plan["status"], "DETOUR")
        self.assertGreater(plan["wp_bearing"], 0.1, "should head left, toward the gap")
        self.assertTrue(all(math.hypot(a[0] - b[0], a[1] - b[1]) > 0 for a, b in zip(plan["path"], plan["path"][1:])))

    def test_no_target_and_at_goal(self):
        m = self._map()
        self.assertEqual(Planner().plan(m.grid, m.origin, m.pose, None, 1.2)["status"], "NO_TARGET")
        self.assertEqual(Planner().plan(m.grid, m.origin, m.pose, (1.0, 0.0), 1.2)["status"], "AT_GOAL")


class ReIdGate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from cv_pipeline import CVPipeline
        cls.pipe = CVPipeline()

    def setUp(self):
        from config import Config
        p = self.pipe
        self.cfg = Config
        p.ekf.reset()
        p.target_state = Config.STATE_FOLLOWING
        p.target_lost_time = None
        p.own_score = 0.7
        p.reacq_count, p.reacq_x = 0, None

    def cand(self, score, x=240):
        return {"similarity_to_owner": score, "centroid": (x, 240), "bbox": [x - 40, 60, x + 40, 420]}

    def track(self):
        p = self.pipe
        p.ekf.init(2.0, 0.0, 0.0)
        for i in range(25):  # a person who has been tracked for a couple of seconds
            p.ekf.predict((i + 1) * 0.08)
            p.ekf.update(2.0, 0.0, 0.05, math.radians(1.0))

    def test_stranger_after_owner_left_is_rejected(self):
        p = self.pipe
        self.track()
        p.target_lost_time = 1.0       # the owner left the view
        p.target_state = self.cfg.STATE_SEARCHING
        results = [p._accept_owner(self.cand(0.42), 0.6, 1, 10.0 + i, 480) for i in range(30)]
        self.assertFalse(any(results), "someone who scores 0.42 must never inherit the lock")

    def test_owner_returning_is_accepted_after_confirmation(self):
        p = self.pipe
        self.track()
        p.target_lost_time = 1.0
        p.target_state = self.cfg.STATE_SEARCHING
        results = [p._accept_owner(self.cand(0.66, 200), 0.7, 1, 10.0 + i, 480) for i in range(12)]
        self.assertFalse(results[0])
        self.assertTrue(results[self.cfg.REACQ_CONFIRM_FRAMES - 1])

    def test_confirmation_resets_if_candidate_jumps_or_score_dips(self):
        p = self.pipe
        self.track()
        p.target_lost_time = 1.0
        p.target_state = self.cfg.STATE_SEARCHING
        for i in range(5):
            p._accept_owner(self.cand(0.66, 100), 0.7, 1, 10.0 + i, 480)
        self.assertFalse(p._accept_owner(self.cand(0.2, 100), 0.7, 1, 16.0, 480))
        self.assertEqual(p.reacq_count, 0)
        for i in range(4):
            p._accept_owner(self.cand(0.66, 100), 0.7, 1, 17.0 + i, 480)
        self.assertFalse(p._accept_owner(self.cand(0.66, 400), 0.7, 1, 22.0, 480))  # different place
        self.assertEqual(p.reacq_count, 1)

    def test_while_tracking_a_much_lower_match_is_not_the_owner(self):
        p = self.pipe
        self.track()
        self.assertTrue(p._accept_owner(self.cand(0.72), 0.8, 2, 5.0, 480))
        self.assertFalse(p._accept_owner(self.cand(0.30), 0.5, 2, 5.1, 480))

    def test_while_tracking_wrong_place_is_rejected_even_if_similar(self):
        p = self.pipe
        self.track()
        self.assertFalse(p._accept_owner(self.cand(0.72, x=420), 0.8, 2, 5.0, 480))  # about 22 degrees away from where the EKF has them

    def test_exact_enrolment_view_is_accepted_immediately(self):
        p = self.pipe
        p.target_lost_time = 0.0
        self.assertTrue(p._accept_owner(self.cand(0.97), 1.0, 1, 5.0, 480))


if __name__ == "__main__":
    unittest.main()
