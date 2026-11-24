"""Camera/LiDAR projection geometry, config overrides, and the stale-video gate."""

import json
import math
import os
import tempfile
import threading
import unittest

import numpy as np

import config as config_mod
from config import Config
import fusion


class Projection(unittest.TestCase):
    def setUp(self):
        self.saved = {k: getattr(Config, k) for k in ("CAM_X_M", "CAM_Y_M", "CAM_HEIGHT_M", "LIDAR_HEIGHT_M",
                                                     "CAM_YAW_DEG", "CAM_PITCH_DEG", "CAMERA_HFOV_DEG")}
        Config.CAM_X_M = Config.CAM_Y_M = 0.0
        Config.CAM_HEIGHT_M, Config.LIDAR_HEIGHT_M = 0.20, 0.17
        Config.CAM_YAW_DEG = Config.CAM_PITCH_DEG = 0.0
        Config.CAMERA_HFOV_DEG = 60.0

    def tearDown(self):
        for k, v in self.saved.items():
            setattr(Config, k, v)

    def test_straight_ahead_projects_to_the_centre_column_below_the_horizon(self):
        cam = fusion.CameraModel(640, 480)
        u, v, depth, ok = cam.project(np.array([2.0]), np.array([0.0]), np.array([0.17]))
        self.assertTrue(ok[0])
        self.assertAlmostEqual(u[0], 320.0, places=3)
        self.assertGreater(v[0], 240.0, "the scan plane is below the camera, so below the image centre")
        self.assertAlmostEqual(v[0], 240 + cam.f * 0.03 / 2.0, places=3)

    def test_left_is_left_and_matches_the_pinhole(self):
        cam = fusion.CameraModel(640, 480)
        b = math.radians(15)
        u, _, _, _ = cam.project(np.array([2.0 * math.cos(b)]), np.array([2.0 * math.sin(b)]), np.array([0.2]))
        self.assertAlmostEqual(u[0], 320 - cam.f * math.tan(b), places=2)
        self.assertAlmostEqual(cam.pixel_bearing(u[0]), b, places=4)

    def test_camera_yaw_and_pitch(self):
        Config.CAM_YAW_DEG = 10.0
        cam = fusion.CameraModel(640, 480)
        b = math.radians(10)
        u, _, _, _ = cam.project(np.array([3 * math.cos(b)]), np.array([3 * math.sin(b)]), np.array([0.2]))
        self.assertAlmostEqual(u[0], 320.0, places=2, msg="a camera turned 10 deg left centres a point 10 deg left")
        Config.CAM_YAW_DEG, Config.CAM_PITCH_DEG = 0.0, 10.0
        cam = fusion.CameraModel(640, 480)
        z = 0.20 - 2.0 * math.tan(math.radians(10))        # on the tilted optical axis
        _, v, _, _ = cam.project(np.array([2.0]), np.array([0.0]), np.array([z]))
        self.assertAlmostEqual(v[0], 240.0, places=2)

    def test_points_behind_or_outside_the_fov_are_dropped(self):
        cam = fusion.CameraModel(640, 480)
        r = np.full(360, np.nan)
        r[0], r[90], r[180] = 2.0, 2.0, 2.0          # ahead, left (outside 60 deg FOV), behind
        proj = cam.project_scan(r)
        self.assertEqual(list(proj["idx"]), [0])

    def test_box_range_takes_the_person_not_the_wall_behind(self):
        cam = fusion.CameraModel(640, 480)
        r = np.full(360, np.nan)
        for i in (358, 359, 0, 1):
            r[i] = 1.5                                  # person
        for i in (355, 356, 357, 2, 3, 4):
            r[i] = 4.0                                  # wall behind, also inside the box
        proj = cam.project_scan(r)
        half = cam.f * math.tan(math.radians(5))
        rng, n = fusion.box_range(proj, cam.box_beams(proj, [320 - half, 100, 320 + half, 400]), near=1.6)
        self.assertAlmostEqual(rng, 1.5, places=2)
        self.assertGreaterEqual(n, 3)


class Overrides(unittest.TestCase):
    def setUp(self):
        self.saved = {k: getattr(Config, k) for k in ("DESIRED_DISTANCE", "TARGET_DISTANCE", "CAM_HEIGHT_M", "MAX_POSES",
                                                     "LIDAR_FUSION", "PI_IP")}

    def tearDown(self):
        for k, v in self.saved.items():
            setattr(Config, k, v)

    def test_file_then_env_and_types_are_kept(self):
        tmp = tempfile.mkdtemp()
        path = os.path.join(tmp, "config.local.json")
        with open(path, "w") as f:
            json.dump({"CAM_HEIGHT_M": 0.31, "MAX_POSES": "2", "NOT_A_SETTING": 1, "DESIRED_DISTANCE": "abc"}, f)
        env = {"HF_DESIRED_DISTANCE": "0.9", "HF_LIDAR_FUSION": "false", "PI_IP": "192.168.4.7"}
        applied = config_mod.apply_overrides(env=env, path=path)
        self.assertEqual(Config.CAM_HEIGHT_M, 0.31)
        self.assertEqual(Config.MAX_POSES, 2)
        self.assertIsInstance(Config.MAX_POSES, int)
        self.assertEqual(Config.DESIRED_DISTANCE, 0.9)
        self.assertEqual(Config.TARGET_DISTANCE, 0.9, "the alias follows")
        self.assertIs(Config.LIDAR_FUSION, False)
        self.assertEqual(Config.PI_IP, "192.168.4.7")
        self.assertNotIn("NOT_A_SETTING", applied)

    def test_states_and_unknown_names_cannot_be_set(self):
        for bad in ("STATE_IDLE", "TUNABLE", "nonexistent"):
            with self.assertRaises(KeyError):
                Config.set(bad, 1)
        with self.assertRaises(ValueError):
            Config.set("DESIRED_DISTANCE", "nan")

    def test_save_local_merges(self):
        tmp = tempfile.mkdtemp()
        path = os.path.join(tmp, "c.json")
        Config.save_local({"CAM_X_M": 0.1}, path)
        Config.save_local({"CAM_Y_M": 0.02}, path)
        with open(path) as f:
            self.assertEqual(json.load(f), {"CAM_X_M": 0.1, "CAM_Y_M": 0.02})


class VideoGate(unittest.TestCase):
    """The offline pattern and the Pi's 'no camera' picture are shown but never reach perception."""

    def streamer(self):
        from video_streamer import VideoStreamer
        s = VideoStreamer.__new__(VideoStreamer)       # no sockets, no threads
        s.lock = threading.RLock()
        s.cond = threading.Condition(s.lock)
        s.latest_jpeg, s.latest_synthetic, s.frame_count = None, True, 0
        s.last_frame_time, s.is_connected = 0.0, False
        s.pi_status, s.source_mode = {"camera": True}, "network"
        s.pi_status_time = __import__("time").time()
        return s

    def test_synthetic_frames_do_not_count(self):
        s = self.streamer()
        s._publish(b"pattern", synthetic=True)
        self.assertFalse(s.camera_frame_ok())
        self.assertEqual(s.last_frame_time, 0.0, "the offline pattern must not make the link look fresh")
        s._publish(b"camera", connected=True)
        self.assertTrue(s.camera_frame_ok())
        self.assertGreater(s.last_frame_time, 0.0)

    def test_frames_before_any_pi_status_are_not_trusted(self):
        s = self.streamer()
        s.pi_status, s.pi_status_time = {}, 0.0
        s._publish(b"frame", connected=True)
        self.assertFalse(s.camera_frame_ok())

    def test_pi_without_camera_is_not_a_camera_frame(self):
        s = self.streamer()
        s.pi_status = {"camera": False}
        s._publish(b"placeholder", connected=True)
        self.assertFalse(s.camera_frame_ok())


if __name__ == "__main__":
    unittest.main()
