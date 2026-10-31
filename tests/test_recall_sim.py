"""Closed loop, real pipeline, simulated room: the owner walks behind a wall and the camera loses them.

The robot must remember where they were (kept in the LiDAR map), plan round the wall through the gap, drive
there with LiDAR avoidance on, find them again with the LiDAR and resume following. It must never fall into the
spinning search on the way. Runs in real time (about 10 s) because the controller and watchdog are real-time.
Commands go to a mock controller, never to a robot (see conftest.py).
"""

import math
import time
import unittest

import numpy as np

from config import Config
from cv_pipeline import CVPipeline
import lidar
from navigation import Planner, PersonMemory, plan_with_memory
from tests.test_navigation import room_scan

BLANK = np.zeros((480, 640, 3), dtype=np.uint8)
WALL = (0.4, -1.8, 0.5, 0.45)            # across the room, the only gap is on the left (y > 0.45)
PERSON = (1.95, -0.65, 2.10, -0.35)      # standing behind the wall, 2 m ahead and 0.5 m to the right


class RecallSim(unittest.TestCase):
    def run_scene(self, extra_boxes=(), seconds=30.0):
        p = CVPipeline()
        p.owner_enrolled = True
        p.owner_profile = {"protected_template": {}, "templates": []}
        p.following_enabled = True
        p.base_target_distance = 1.2
        p.robot.set_mode("AUTO")
        boxes = [WALL, PERSON] + list(extra_boxes)

        pose = [0.0, 0.0, 0.0]
        lm = lidar.LidarMap()
        for _ in range(40):                                     # the map of the room, built before the owner hid
            lm.update(room_scan(tuple(pose), boxes), tuple(pose))
        mem, planner = PersonMemory(), Planner()
        mem.update({"x": 2.0, "y": -0.5, "vx": 0.0, "vy": 0.0, "misses": 0}, tuple(pose), time.time() - 0.5)   # last sighting

        now = time.time()
        p.ekf.init(math.hypot(2.0, 0.5), math.atan2(-0.5, 2.0), now)
        p.target_state = Config.STATE_TARGET_TEMPORARILY_LOST
        p.target_lost_time = now - 0.5
        p.last_cam_seen = now - 0.5

        states, statuses, plan_statuses, min_range = [], [], [], 9.9
        relocked_at = None
        last_plan, last = 0.0, time.time()
        t0 = time.time()
        while time.time() - t0 < seconds:
            t = time.time()
            dt = min(0.2, t - last)
            last = t
            bins = room_scan(tuple(pose), boxes)
            lm.update(bins, tuple(pose))
            p.set_lidar(bins)
            min_range = min(min_range, float(np.nanmin(lidar.to_meters(bins))))
            if t - last_plan >= 0.25:
                last_plan = t
                person = p.ekf.as_dict() if p.lidar_only else None
                plan = plan_with_memory(planner, lm.grid, lm.origin, lm.pose, person, mem, t, Config.TARGET_DISTANCE)
                if plan is not None:
                    plan_statuses.append(plan["status"])
                    p.set_plan({k: plan.get(k) for k in ("blocked", "wp_bearing", "status", "memory", "mem_bearing", "mem_range", "goal_dist", "age")})
            tel, _ = p.process_frame(BLANK.copy())
            states.append(p.target_state)
            statuses.append(tel["nav_status"])
            v, w = p.robot.current_linear, p.robot.current_angular
            pose[0] += v * math.cos(pose[2]) * dt
            pose[1] += v * math.sin(pose[2]) * dt
            pose[2] += w * dt
            if p.lidar_only and relocked_at is None:
                relocked_at = time.time() - t0
            if relocked_at is not None and time.time() - t0 > relocked_at + 1.5:
                break
            time.sleep(0.03)
        p.robot.set_mode("STOP")
        self.plan_statuses = plan_statuses
        return p, states, statuses, relocked_at, min_range, pose

    def test_goes_round_the_wall_finds_the_owner_and_follows_again(self):
        p, states, statuses, relocked_at, min_range, pose = self.run_scene()
        self.assertIsNotNone(relocked_at, f"never found the owner again; last statuses {statuses[-5:]}, pose {pose}")
        self.assertNotIn(Config.STATE_SEARCHING, states, "it must not fall into the spinning search while a path to the remembered spot exists")
        self.assertIn(Config.STATE_RECOVERING, states)
        self.assertIn("DETOUR", self.plan_statuses, "the wall is in the way, so the planner must route via the gap")
        self.assertTrue(any(st.startswith("MEMORY_") or st.startswith("LIDAR_") for st in statuses))
        self.assertGreater(min_range, 0.12, f"came within {min_range:.2f} m of something")
        self.assertIn(p.target_state, (Config.STATE_FOLLOWING, Config.STATE_WAIT_FOR_OWNER_MOVEMENT))
        self.assertGreater(pose[1], 0.3, "it must have gone through the gap on the left")
        self.assertTrue(any(e["type"] == "MEMORY_RELOCK" for e in p.event_log))


if __name__ == "__main__":
    unittest.main()
