"""CVPipeline fusion path with a synthetic person and a synthetic LiDAR scan (no camera needed)."""

import math
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "pi"))
sys.path.insert(0, str(ROOT / "laptop"))

from config import Config  # noqa: E402
from cv_pipeline import CVPipeline  # noqa: E402
from tests.test_lidar import bins_with  # noqa: E402

W = 480


def person(bearing_deg, vision_range):
    """Candidate as _detect_all_persons would produce it, centred at the given bearing."""
    f_px = (W / 2) / math.tan(math.radians(Config.CAMERA_HFOV_DEG) / 2)
    cx = W / 2 - math.tan(math.radians(bearing_deg)) * f_px
    return {"centroid": (cx, 240), "bbox": [cx - 40, 60, cx + 40, 420], "distance_m": vision_range}


class FusionPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pipe = CVPipeline()
        cls.pipe.robot.set_mode("AUTO")

    def setUp(self):
        self.pipe.ekf.reset()
        self.pipe.vision_scale = 1.0
        self.pipe.follow.reset()

    def test_lidar_overrides_wrong_camera_range_and_calibrates(self):
        p = self.pipe
        p.set_lidar(bins_with({0: 1.8}))
        now = time.time()
        for i in range(60):
            p.set_lidar(bins_with({0: 1.8}))
            r, b, _ = p._fuse(person(0.0, 1.4), W, now + i * 0.08)  # camera says 1.4 m, LiDAR says 1.8 m
        self.assertAlmostEqual(r, 1.8, delta=0.1)
        self.assertEqual(p.fusion_mode, "LIDAR+CAM")
        self.assertGreater(p.vision_scale, 1.1)  # camera scale pulled toward the LiDAR

    def test_bearing_sign_left_is_positive(self):
        p = self.pipe
        p.set_lidar(bins_with({}))
        r, b, _ = p._fuse(person(15.0, 2.0), W, time.time())
        self.assertAlmostEqual(math.degrees(b), 15.0, delta=1.0)

    def test_pid_command_forward_and_turns_toward_person(self):
        p = self.pipe
        p.set_lidar(bins_with({25: 1.9}))
        now = time.time()
        cand = person(25.0, 2.0)
        for i in range(10):
            p.set_lidar(bins_with({25: 1.9}))
            r, b, f_px = p._fuse(cand, W, now + i * 0.07)
            v, w, status = p._follow_command(cand, b, r, f_px, r, {"front": 3, "left": 3, "right": 3}, now + i * 0.07)
        self.assertGreater(w, 0.0)  # person on the left -> turn left (+w)
        self.assertLessEqual(v, Config.MAX_LINEAR_VEL)
        self.assertEqual(status, "LIDAR_CLEAR")

    def test_obstacle_beside_person_stops_robot_but_person_does_not(self):
        p = self.pipe
        now = time.time()
        cand = person(0.0, 1.8)
        for i in range(10):
            p.set_lidar(bins_with({0: 1.8}))  # only the person ahead
            r, b, f_px = p._fuse(cand, W, now + i * 0.07)
            v, w, status = p._follow_command(cand, b, r, f_px, r, {"front": 3, "left": 3, "right": 3}, now + i * 0.07)
        self.assertEqual(status, "LIDAR_CLEAR")
        self.assertGreater(v, 0.0)
        for i in range(10, 14):
            p.set_lidar(bins_with({0: 1.8, 12: 0.25}))  # box right in the way, off to one side of the person
            r, b, f_px = p._fuse(cand, W, now + i * 0.07)
            v, w, status = p._follow_command(cand, b, r, f_px, r, {"front": 3, "left": 3, "right": 3}, now + i * 0.07)
        self.assertEqual(status, "LIDAR_STOP")
        self.assertEqual(v, 0.0)


if __name__ == "__main__":
    unittest.main()
