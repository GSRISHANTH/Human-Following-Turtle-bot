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

    def lose(self, ago=0.5):
        p = self.pipe
        p.target_lost_time = 1000.0
        p.target_state = self.cfg.STATE_SEARCHING
        return 1000.0 + ago

    def test_stranger_after_owner_left_is_rejected(self):
        p = self.pipe
        self.track()
        t = self.lose()
        results = [p._accept_owner(self.cand(0.28), 0.5, 1, t + i * 0.08, 480) for i in range(40)]
        self.assertFalse(any(results), "someone who looks nothing like the owner must never inherit the lock")

    def test_owner_returning_is_accepted_after_a_short_confirmation(self):
        p = self.pipe
        self.track()
        t = self.lose()
        results = [p._accept_owner(self.cand(0.52, 200), 0.6, 1, t + i * 0.08, 480) for i in range(8)]
        self.assertFalse(results[0])
        self.assertTrue(results[2], "near where the owner was heading, 3 frames are enough")

    def test_far_from_where_the_owner_was_heading_needs_more_evidence(self):
        p = self.pipe
        self.track()
        t = self.lose(0.0)
        # candidate at the frame edge, ~+28 degrees from the EKF, shortly after the loss: no relaxation
        results = [p._accept_owner(self.cand(0.45, 5), 0.5, 1, t + i * 0.08, 480) for i in range(8)]
        self.assertFalse(results[2], "3 frames are only enough near the predicted position")
        self.assertTrue(results[self.cfg.REACQ_CONFIRM_FRAMES - 1])

    def test_lidar_alive_means_the_track_is_not_lost(self):
        p = self.pipe
        self.track()
        self.lose()
        # the owner is invisible to the camera but the LiDAR still has them: a lookalike elsewhere is rejected
        self.assertFalse(p._accept_owner(self.cand(0.9, 20), 0.9, 2, 1001.0, 480, lidar_alive=True))
        self.assertTrue(p._accept_owner(self.cand(0.5, 245), 0.6, 2, 1001.1, 480, lidar_alive=True))

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


class Memory(unittest.TestCase):
    from navigation import PersonMemory, plan_with_memory  # noqa: E402

    @staticmethod
    def person(x, y, vx=0.0, vy=0.0, misses=0):
        return {"x": x, "y": y, "vx": vx, "vy": vy, "misses": misses}

    def test_remembers_in_the_map_frame_and_ignores_unmeasured_coasting(self):
        from navigation import PersonMemory
        mem = PersonMemory()
        mem.update(self.person(2.0, 0.0), (1.0, 1.0, math.pi / 2), 10.0)       # robot faces +y, person 2 m ahead
        self.assertAlmostEqual(mem.pos[0], 1.0, places=3)
        self.assertAlmostEqual(mem.pos[1], 3.0, places=3)
        mem.update(self.person(9.0, 9.0, misses=5), (1.0, 1.0, 0.0), 11.0)      # coasting EKF: not trusted
        self.assertAlmostEqual(mem.pos[1], 3.0, places=3)

    def test_extrapolation_decays_and_is_capped(self):
        from navigation import PersonMemory, MAX_EXTRAP_M
        mem = PersonMemory()
        mem.update(self.person(2.0, 0.0, vx=0.5), (0.0, 0.0, 0.0), 0.0)
        near = mem.estimate(1.0)[0] - 2.0
        far = mem.estimate(10.0)[0] - 2.0
        self.assertGreater(near, 0.0)
        self.assertGreater(far, near)
        self.assertLessEqual(far, MAX_EXTRAP_M + 1e-6)
        fast = PersonMemory()
        fast.update(self.person(2.0, 0.0, vx=2.0), (0.0, 0.0, 0.0), 0.0)
        self.assertLessEqual(fast.estimate(30.0)[0] - 2.0, MAX_EXTRAP_M + 1e-6)

    def test_expires(self):
        from navigation import PersonMemory, MEMORY_SEC
        mem = PersonMemory()
        mem.update(self.person(2.0, 0.0), (0.0, 0.0, 0.0), 0.0)
        self.assertTrue(mem.valid(MEMORY_SEC - 1))
        self.assertFalse(mem.valid(MEMORY_SEC + 1))

    def _scene(self):
        m = lidar.LidarMap()
        wall = [(0.4, -1.8, 0.5, 0.45)]          # wall across the room with the only gap on the left
        for _ in range(40):
            m.update(room_scan((0.0, 0.0, 0.0), wall), (0.0, 0.0, 0.0))
        return m

    def test_hidden_owner_behind_an_obstacle_is_planned_to_round_it(self):
        from navigation import PersonMemory, plan_with_memory
        m = self._scene()
        mem = PersonMemory()
        pl = Planner()
        # tracked just before they walked behind the wall, then no longer measured
        first = plan_with_memory(pl, m.grid, m.origin, m.pose, self.person(2.0, -0.5), mem, 0.0, 1.2)
        self.assertFalse(first["memory"])
        lost = plan_with_memory(pl, m.grid, m.origin, m.pose, None, mem, 3.0, 1.2)
        self.assertTrue(lost["memory"])
        self.assertEqual(lost["status"], "DETOUR")
        self.assertGreater(lost["wp_bearing"], 0.1, "toward the gap")
        self.assertGreater(lost["goal_dist"], 0.3)
        self.assertGreater(lost["age"], 2.9)

    def test_memory_stays_put_in_the_map_while_the_robot_moves(self):
        from navigation import PersonMemory, plan_with_memory
        m = self._scene()
        mem = PersonMemory()
        pl = Planner()
        plan_with_memory(pl, m.grid, m.origin, m.pose, self.person(2.0, 0.0), mem, 0.0, 1.2)
        world = tuple(mem.pos)
        moved = (0.0, 0.9, math.radians(40))                       # the robot drove round and turned
        plan = plan_with_memory(pl, m.grid, m.origin, moved, None, mem, 2.0, 1.2)
        self.assertEqual(tuple(mem.pos), world)
        # in the new robot frame the remembered spot is where the map says it is
        c, s = math.cos(-moved[2]), math.sin(-moved[2])
        dx, dy = world[0] - moved[0], world[1] - moved[1]
        self.assertAlmostEqual(plan["mem_bearing"], math.atan2(s * dx + c * dy, c * dx - s * dy), places=2)

    def test_nothing_to_go_to_when_never_seen_or_too_old(self):
        from navigation import PersonMemory, plan_with_memory, MEMORY_SEC
        m = self._scene()
        mem = PersonMemory()
        self.assertIsNone(plan_with_memory(Planner(), m.grid, m.origin, m.pose, None, mem, 0.0, 1.2))
        mem.update(self.person(2.0, 0.0), m.pose, 0.0)
        self.assertIsNone(plan_with_memory(Planner(), m.grid, m.origin, m.pose, None, mem, MEMORY_SEC + 1, 1.2))

    def test_replans_when_an_obstacle_appears_and_when_it_goes(self):
        from navigation import PersonMemory, plan_with_memory
        clear = lidar.LidarMap()
        for _ in range(40):
            clear.update(room_scan((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
        mem = PersonMemory()
        pl = Planner()
        a = plan_with_memory(pl, clear.grid, clear.origin, clear.pose, self.person(2.0, 0.0), mem, 0.0, 1.2)
        self.assertEqual(a["status"], "DIRECT")
        for _ in range(40):                                        # a box is put between the robot and the spot
            clear.update(room_scan((0.0, 0.0, 0.0), [(0.3, -0.3, 0.55, 0.3)]), (0.0, 0.0, 0.0))
        b = plan_with_memory(pl, clear.grid, clear.origin, clear.pose, None, mem, 1.0, 1.2)
        self.assertTrue(b["memory"])
        self.assertEqual(b["status"], "DETOUR", "must replan round the new box")
        for _ in range(60):                                        # and it is taken away again
            clear.update(room_scan((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
        c = plan_with_memory(pl, clear.grid, clear.origin, clear.pose, None, mem, 2.0, 1.2)
        self.assertEqual(c["status"], "DIRECT")
