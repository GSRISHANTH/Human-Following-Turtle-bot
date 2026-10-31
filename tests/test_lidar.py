"""LiDAR: scan packet, sectors with person masking, fusion lookup, occupancy grid, avoidance."""

import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pi"))
sys.path.insert(0, str(ROOT / "laptop"))

import tb_proto  # noqa: E402
import lidar  # noqa: E402


def bins_with(obstacles, n=360, default=0):
    """obstacles: {bearing_deg: metres}; every bin within 3 degrees gets that range."""
    out = [default] * n
    for i in range(n):
        deg = (i + 0.5) * 360.0 / n
        deg = deg - 360 if deg > 180 else deg
        for b, r in obstacles.items():
            if abs(deg - b) <= 3.0:
                out[i] = int(r * 1000)
    return out


class ScanPacket(unittest.TestCase):
    def test_roundtrip_with_and_without_odom(self):
        asm = tb_proto.Reassembler()
        bins = list(range(360))
        kind, p, _ = asm.feed(tb_proto.pack_scan(bins, (1.0, -2.0, 0.5), 1.0))
        self.assertEqual(kind, "scan")
        self.assertEqual(p["bins"], bins)
        self.assertAlmostEqual(p["pose"][1], -2.0, places=5)
        _, p, _ = asm.feed(tb_proto.pack_scan(bins, None, 1.0))
        self.assertIsNone(p["pose"])

    def test_fits_one_datagram(self):
        self.assertLess(len(tb_proto.pack_scan([1000] * 360, (0, 0, 0), 0.0)), 1200)


class Sectors(unittest.TestCase):
    def test_front_left_right(self):
        r = lidar.to_meters(bins_with({0: 0.5, 50: 0.7, -50: 1.1}))
        s = lidar.sectors(r)
        self.assertAlmostEqual(s["front"], 0.5, delta=0.01)
        self.assertAlmostEqual(s["left"], 0.7, delta=0.01)
        self.assertAlmostEqual(s["right"], 1.1, delta=0.01)

    def test_person_is_masked_but_wall_behind_is_not(self):
        r = lidar.to_meters(bins_with({0: 1.2, 20: 0.6}))
        s = lidar.sectors(r, exclude=(0.0, math.radians(8), 1.2))
        self.assertAlmostEqual(s["front"], 0.6, delta=0.01)  # the box at 20 degrees remains
        self.assertAlmostEqual(lidar.sectors(r)["front"], 0.6, delta=0.01)
        r2 = lidar.to_meters(bins_with({0: 1.2}))
        self.assertGreater(lidar.sectors(r2, exclude=(0.0, math.radians(8), 1.2))["front"], 3.0)

    def test_person_cluster_is_found_by_bearing_and_size(self):
        r = lidar.to_meters(bins_with({0: 1.3}))
        pc = lidar.person_cluster(lidar.segment(r), 0.0, near=1.2)
        self.assertAlmostEqual(pc["range"], 1.3, delta=0.05)
        self.assertLess(abs(pc["bearing"]), 0.03)
        self.assertIsNone(lidar.person_cluster(lidar.segment(lidar.to_meters([0] * 360)), 0.0, near=1.0))

    def test_wall_is_not_a_person(self):
        wall = [int(1500 / max(math.cos(math.radians(a)), 0.2)) if abs(a) < 50 else 0
                for a in [((i + 0.5) - 360 if (i + 0.5) > 180 else (i + 0.5)) for i in range(360)]]
        clusters = lidar.segment(lidar.to_meters(wall))
        self.assertGreater(max(c["width"] for c in clusters), 1.5)
        self.assertIsNone(lidar.person_cluster(clusters, 0.0, near=1.5))


class Avoidance(unittest.TestCase):
    def test_stops_and_turns_toward_open_side(self):
        av = lidar.LidarAvoidance()
        v, w, status = av.apply(0.2, 0.0, {"front": 0.25, "left": 2.0, "right": 0.5})
        self.assertEqual(v, 0.0)
        self.assertGreater(w, 0.0)  # left is the open side, +w is left
        self.assertEqual(status, "LIDAR_STOP")

    def test_slows_in_warning_zone_and_passes_clear_path(self):
        av = lidar.LidarAvoidance()
        v, w, status = av.apply(0.2, 0.0, {"front": 0.55, "left": 1.0, "right": 1.0})
        self.assertTrue(0.0 < v < 0.2)
        self.assertEqual(status, "LIDAR_AVOIDING")
        self.assertEqual(av.apply(0.2, 0.1, {"front": 3.0, "left": 3.0, "right": 3.0}), (0.2, 0.1, "LIDAR_CLEAR"))


class Map(unittest.TestCase):
    def test_wall_registers_and_render_runs(self):
        m = lidar.LidarMap()
        wall = [int(1500 / max(math.cos(math.radians(a)), 0.2)) if abs(a) < 60 else 0
                for a in [((i + 0.5) - 360 if (i + 0.5) > 180 else (i + 0.5)) for i in range(360)]]
        for _ in range(5):
            m.update(wall, (0.0, 0.0, 0.0))
        gx, gy = m._cell(np.array([1.49]), np.array([0.0]))
        self.assertGreater(m.grid[gy[0], gx[0]], 0.8, "wall cell should be occupied")
        gx, gy = m._cell(np.array([0.7]), np.array([0.0]))
        self.assertLess(m.grid[gy[0], gx[0]], 0.0, "cell in front of the wall should be free")
        img = m.render(person={"x": 1.0, "y": 0.2, "vx": 0.1, "vy": 0.0}, sec=lidar.sectors(m.bins))
        self.assertEqual(img.shape, (lidar.LidarMap.VIEW, lidar.LidarMap.VIEW, 3))

    def test_no_odometry_does_not_accumulate(self):
        m = lidar.LidarMap()
        m.update([1000] * 360, None)
        self.assertEqual(float(np.abs(m.grid).sum()), 0.0)
        m.render()


if __name__ == "__main__":
    unittest.main()


class BadOdometry(unittest.TestCase):
    """A NaN or a restarted odometry must never kill the map (this blanked the dashboard map for a whole session)."""

    def _warm(self):
        m = lidar.LidarMap()
        scan = [1500] * 360
        for _ in range(30):
            m.update(scan, (0.0, 0.0, 0.0))
        return m, scan

    def test_nan_odometry_is_ignored_and_rendering_still_works(self):
        m, scan = self._warm()
        m.update(scan, (float("nan"), 0.0, 0.0))
        m.update(scan, (0.0, 0.0, 0.0))
        self.assertEqual(m.render().shape[2], 3)
        self.assertTrue(all(math.isfinite(x) and math.isfinite(y) for x, y in m.trail))

    def test_odometry_restart_starts_a_new_map_instead_of_smearing(self):
        m, scan = self._warm()
        before = m.scans
        m.update(scan, (6.0, 16.0, 1.2))       # the Pi rebooted: odometry is in a different frame
        self.assertEqual(m.resets, 1)
        self.assertLess(m.scans, before)
        self.assertEqual(m.render().shape[2], 3)

    def test_normal_motion_is_not_mistaken_for_a_restart(self):
        m, scan = self._warm()
        for i in range(10):
            m.update(scan, (0.05 * (i + 1), 0.0, 0.01 * i))
        self.assertEqual(m.resets, 0)
