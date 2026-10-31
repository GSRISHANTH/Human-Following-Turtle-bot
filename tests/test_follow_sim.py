"""Closed-loop check of EKF + PID against a walking person.

Plant: unicycle with the controller's 0.9 m/s^2 and 3.6 rad/s^2 slew limits, 0.15 s of
camera/processing latency on measurements, 0.10 s of command latency. Measurements are noisy
polar readings: camera every frame (~12 Hz, sigma 0.12*range, 1.5 deg) and LiDAR (10 Hz, 5 cm).
"""

import math
import random
import sys
import unittest
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "laptop"))

from config import Config  # noqa: E402
from ekf_person import PersonEKF  # noqa: E402
from follow_controller import FollowController  # noqa: E402


def simulate(person_path, seconds=40.0, seed=3, start_range=1.6, lidar=True):
    rnd = random.Random(seed)
    dt = 1.0 / 30.0
    rx = ry = rth = 0.0
    v = w = 0.0
    px, py = start_range, 0.0
    ekf, ctl = PersonEKF(), FollowController()
    meas_q = deque()
    cmd_q = deque()
    t = 0.0
    log = []
    next_cam = next_lidar = 0.0
    tgt_v = tgt_w = 0.0
    last_ctl = 0.0
    while t < seconds:
        pvx, pvy = person_path(t)
        px += pvx * dt
        py += pvy * dt
        # true polar measurement of the person in the robot frame
        dx, dy = px - rx, py - ry
        c, s = math.cos(-rth), math.sin(-rth)
        lx, ly = c * dx - s * dy, s * dx + c * dy
        r_true, b_true = math.hypot(lx, ly), math.atan2(ly, lx)
        if t >= next_cam:
            next_cam += 1 / 12.0
            meas_q.append((t + 0.15, "cam", r_true + rnd.gauss(0, 0.12 * r_true), b_true + rnd.gauss(0, math.radians(1.5))))
        if lidar and t >= next_lidar:
            next_lidar += 0.1
            meas_q.append((t + 0.05, "lidar", r_true + rnd.gauss(0, 0.05), b_true + rnd.gauss(0, math.radians(1.0))))
        while meas_q and meas_q[0][0] <= t:
            _, kind, r_m, b_m = meas_q.popleft()
            if not ekf.active:
                ekf.init(r_m, b_m, t)
            ekf.predict(t, v, w)
            ekf.update(r_m, b_m, 0.05 if kind == "lidar" else 0.12 * r_m, math.radians(1.0 if kind == "lidar" else 1.5))
        if ekf.active and t - last_ctl >= 1 / 15.0:
            ekf.predict(t, v, w)
            r_e, b_e = ekf.range_bearing()
            ux, uy = ekf.s[2], ekf.s[3]
            los = (ekf.s[0] * ux + ekf.s[1] * uy) / max(r_e, 1e-3)
            tgt_v, tgt_w = ctl.command(r_e, b_e, ekf.range_rate(), ekf.bearing_rate(w), los, t - last_ctl)
            last_ctl = t
            cmd_q.append((t + 0.10, tgt_v, tgt_w))
        while cmd_q and cmd_q[0][0] <= t:
            _, cv_, cw_ = cmd_q.popleft()
            applied_v, applied_w = cv_, cw_
            tgt_v, tgt_w = applied_v, applied_w
        dv, dw = tgt_v - v, tgt_w - w
        v += max(-0.9 * dt, min(0.9 * dt, dv))
        w += max(-3.6 * dt, min(3.6 * dt, dw))
        rx += v * math.cos(rth) * dt
        ry += v * math.sin(rth) * dt
        rth += w * dt
        log.append((t, r_true, b_true, v, w))
        t += dt
    return log


def straight_then_turn(t):
    if t < 12:
        return 0.18, 0.0
    if t < 22:  # walks an arc to the left
        a = 0.15 * (t - 12)
        return 0.18 * math.cos(a), 0.18 * math.sin(a)
    if t < 30:
        return 0.0, 0.0
    return 0.12, 0.0


class FollowSim(unittest.TestCase):
    def test_tracks_walking_person(self):
        log = simulate(straight_then_turn)
        vmax = max(l[3] for l in log)
        self.assertLessEqual(vmax, Config.MAX_LINEAR_VEL + 1e-6)
        # while the person walks (3 s to settle, then until they stop at 22 s)
        walking = [l for l in log if 4.0 <= l[0] <= 22.0]
        rms_range = math.sqrt(sum((l[1] - Config.DESIRED_DISTANCE) ** 2 for l in walking) / len(walking))
        max_bearing = max(abs(math.degrees(l[2])) for l in walking)
        self.assertLess(rms_range, 0.25, f"range rms {rms_range:.2f}")
        self.assertLess(max_bearing, 25.0, f"bearing {max_bearing:.1f}")
        # after the person stops, the robot settles near the desired distance and is centred
        end = [l for l in log if 26.0 <= l[0] <= 30.0]
        self.assertLess(abs(sum(l[1] for l in end) / len(end) - Config.DESIRED_DISTANCE), 0.15)
        self.assertLess(max(abs(math.degrees(l[2])) for l in end), 8.0)
        self.assertLess(max(abs(l[3]) for l in end), 0.06, "robot should be nearly stopped")

    def test_never_reverses_or_collides(self):
        log = simulate(straight_then_turn)
        self.assertGreaterEqual(min(l[3] for l in log), 0.0)
        self.assertGreater(min(l[1] for l in log), 0.7)

    def test_works_with_camera_only(self):
        log = simulate(straight_then_turn, lidar=False)
        walking = [l for l in log if 6.0 <= l[0] <= 22.0]
        rms = math.sqrt(sum((l[1] - Config.DESIRED_DISTANCE) ** 2 for l in walking) / len(walking))
        self.assertLess(rms, 0.35)

    def test_ekf_estimates_person_velocity(self):
        log = simulate(lambda t: (0.18, 0.0), seconds=15.0)
        self.assertGreater(len(log), 100)


class EKFUnit(unittest.TestCase):
    def test_converges_on_static_target(self):
        ekf = PersonEKF()
        ekf.init(2.0, 0.0, 0.0)
        for i in range(60):
            t = (i + 1) * 0.1
            ekf.predict(t)
            ekf.update(1.5 + random.Random(i).gauss(0, 0.05), 0.2, 0.05, 0.02)
        r, b = ekf.range_bearing()
        self.assertAlmostEqual(r, 1.5, delta=0.08)
        self.assertAlmostEqual(b, 0.2, delta=0.03)
        self.assertLess(math.hypot(ekf.s[2], ekf.s[3]), 0.15)

    def test_gate_rejects_outlier(self):
        ekf = PersonEKF()
        ekf.init(2.0, 0.0, 0.0)
        for i in range(20):
            ekf.predict((i + 1) * 0.1)
            ekf.update(2.0, 0.0, 0.05, 0.02)
        ekf.predict(2.2)
        self.assertFalse(ekf.update(4.5, 1.0, 0.05, 0.02))

    def test_ego_motion_compensation(self):
        """Robot drives 0.2 m/s toward a still person: range must fall without any measurement."""
        ekf = PersonEKF()
        ekf.init(2.0, 0.0, 0.0)
        for i in range(10):
            ekf.predict((i + 1) * 0.1, v_robot=0.2, w_robot=0.0)
        self.assertAlmostEqual(ekf.range_bearing()[0], 2.0 - 0.2 * 1.0, delta=0.02)

    def test_turning_robot_shifts_bearing(self):
        ekf = PersonEKF()
        ekf.init(2.0, 0.0, 0.0)
        for i in range(10):
            ekf.predict((i + 1) * 0.1, v_robot=0.0, w_robot=0.3)
        self.assertAlmostEqual(ekf.range_bearing()[1], -0.3, delta=0.02)  # robot turned left, person now to the right


if __name__ == "__main__":
    unittest.main()
