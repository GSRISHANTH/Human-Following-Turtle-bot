"""
PID follow controller driven by the EKF state (range, bearing, and their rates).

    v = Kp_d e_d + Ki_d INT e_d + Kd_d d(e_d)/dt  +  Kff * (person speed along the line of sight)
    w = Kp_b e_b + Ki_b INT e_b + Kd_b d(e_b)/dt

e_d = range - desired distance (m), e_b = bearing (rad, positive = person on the left, so turn left).
The derivative terms use the EKF's range and bearing rates, not a finite difference of noisy
measurements. The feed-forward term lets the robot match a walking person instead of lagging by
Kp-worth of distance. Integrators are clamped and frozen while the output saturates.

Gains (Config.PID_*) were tuned in tests/test_follow_sim.py: a Burger-like plant with the 0.9 m/s^2
slew limit and ~0.15 s of video and command latency, person walking 0.2 to 0.3 m/s with turns.
"""

import math
from config import Config


class PID:
    def __init__(self, kp, ki, kd, i_limit):
        self.kp, self.ki, self.kd, self.i_limit = kp, ki, kd, i_limit
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def step(self, err, err_rate, dt, lo, hi):
        p = self.kp * err
        d = self.kd * err_rate
        out = p + self.ki * self.integral + d
        saturated = out < lo or out > hi
        if not saturated or (err * self.integral) < 0:  # anti-windup: only integrate when it helps
            self.integral = max(-self.i_limit, min(self.i_limit, self.integral + err * dt))
        out = p + self.ki * self.integral + d
        return max(lo, min(hi, out))


class FollowController:
    def __init__(self):
        self.dist = PID(Config.PID_DIST_KP, Config.PID_DIST_KI, Config.PID_DIST_KD, 0.6)
        self.turn = PID(Config.PID_TURN_KP, Config.PID_TURN_KI, Config.PID_TURN_KD, 0.4)

    def reset(self):
        self.dist.reset()
        self.turn.reset()

    def command(self, rng, bearing, range_rate, bearing_rate, person_los_speed, dt, desired=None):
        """Returns (v, w). bearing in rad, rates in per-second units."""
        desired = Config.DESIRED_DISTANCE if desired is None else desired
        dt = max(1e-3, min(0.3, dt))

        # steering first: it also gates the forward speed
        w = self.turn.step(bearing, bearing_rate, dt, -Config.MAX_ANGULAR_VEL, Config.MAX_ANGULAR_VEL)
        if abs(bearing) < math.radians(Config.STEERING_DEADZONE_DEG):
            w = 0.0
            self.turn.integral *= 0.9

        e_d = rng - desired
        v = self.dist.step(e_d, range_rate, dt, 0.0, Config.MAX_LINEAR_VEL)
        # feed-forward fades out once we are closer than desired, so a stopping person cannot drag us in
        ff_scale = max(0.0, min(1.0, (rng - (desired - 0.15)) / 0.15))
        v += Config.PID_FF_GAIN * max(0.0, person_los_speed) * ff_scale
        # do not charge ahead while the person is well off-axis; turn first
        v *= max(0.0, math.cos(min(abs(bearing), math.pi / 2))) ** 2
        v = max(0.0, min(Config.MAX_LINEAR_VEL, v))
        if v < Config.PID_MIN_V:
            v = 0.0
        elif v < Config.MIN_ROLLING_VEL:
            v = Config.MIN_ROLLING_VEL
        return v, w
