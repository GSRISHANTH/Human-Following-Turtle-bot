"""TurtleBot3 Burger kinematic plant.

Same wheel update as ROBOTIS turtlebot3_fake_node (Humble):

    https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/humble/turtlebot3_fake_node/src/turtlebot3_fake_node.cpp

Burger wheel numbers are the fake-node defaults used by that package
(wheels.separation = 0.160 m, wheels.radius = 0.033 m). Body speed is
clamped to the same limits as pi/pi_turtlebot_node.py so the bench
matches the real motor bridge, not the 2.84 rad/s catalogue maximum.
"""

import math
import threading
import time


# ROBOTIS TurtleBot3 Burger
WHEEL_SEPARATION_M = 0.160
WHEEL_RADIUS_M = 0.033
# Catalogue limits (emanual.robotis.com TurtleBot3 specifications)
SPEC_MAX_LINEAR_MPS = 0.22
SPEC_MAX_ANGULAR_RPS = 2.84
# Limits actually applied by the Pi companion before /cmd_vel
BRIDGE_MAX_LINEAR_MPS = 0.22
BRIDGE_MAX_ANGULAR_RPS = 1.40
# Same heartbeat as the Pi motor watchdog
CMD_TIMEOUT_SEC = 0.60


def wrap_angle(theta):
    return math.atan2(math.sin(theta), math.cos(theta))


class BurgerPlant:
    """Integrates body velocity into odometry and wheel joints."""

    def __init__(self, cmd_timeout=CMD_TIMEOUT_SEC):
        self.wheel_separation = WHEEL_SEPARATION_M
        self.wheel_radius = WHEEL_RADIUS_M
        self.cmd_timeout = cmd_timeout
        self._lock = threading.Lock()
        self._goal_v = 0.0
        self._goal_w = 0.0
        self._last_cmd_time = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.wheel_left = 0.0
        self.wheel_right = 0.0
        self.wheel_left_vel = 0.0
        self.wheel_right_vel = 0.0
        self.trail = []
        self._trail_dist = 0.0

    def set_body_velocity(self, linear, angular, now=None):
        linear = max(-BRIDGE_MAX_LINEAR_MPS, min(BRIDGE_MAX_LINEAR_MPS, float(linear)))
        angular = max(-BRIDGE_MAX_ANGULAR_RPS, min(BRIDGE_MAX_ANGULAR_RPS, float(angular)))
        with self._lock:
            self._goal_v = linear
            self._goal_w = angular
            self._last_cmd_time = time.time() if now is None else now

    def reset(self):
        with self._lock:
            self._goal_v = 0.0
            self._goal_w = 0.0
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            self.v = 0.0
            self.w = 0.0
            self.wheel_left = 0.0
            self.wheel_right = 0.0
            self.wheel_left_vel = 0.0
            self.wheel_right_vel = 0.0
            self.trail = []
            self._trail_dist = 0.0

    def step(self, dt, now=None):
        """One fake-node update. dt is seconds."""
        if dt <= 0.0:
            return
        now = time.time() if now is None else now
        with self._lock:
            if self._last_cmd_time <= 0.0 or (now - self._last_cmd_time) > self.cmd_timeout:
                v_body = 0.0
                w_body = 0.0
            else:
                v_body = self._goal_v
                w_body = self._goal_w

            # Wheel linear speeds (m/s), then wheel rotation (rad), matching fake_node.
            v_left = v_body - (w_body * self.wheel_separation / 2.0)
            v_right = v_body + (w_body * self.wheel_separation / 2.0)
            w_left = v_left / self.wheel_radius
            w_right = v_right / self.wheel_radius
            wheel_l = w_left * dt
            wheel_r = w_right * dt
            if math.isnan(wheel_l):
                wheel_l = 0.0
            if math.isnan(wheel_r):
                wheel_r = 0.0

            self.wheel_left += wheel_l
            self.wheel_right += wheel_r
            self.wheel_left_vel = w_left
            self.wheel_right_vel = w_right

            delta_s = self.wheel_radius * (wheel_r + wheel_l) / 2.0
            delta_theta = self.wheel_radius * (wheel_r - wheel_l) / self.wheel_separation

            self.x += delta_s * math.cos(self.theta + (delta_theta / 2.0))
            self.y += delta_s * math.sin(self.theta + (delta_theta / 2.0))
            self.theta = wrap_angle(self.theta + delta_theta)
            self.v = delta_s / dt
            self.w = delta_theta / dt

            self._trail_dist += abs(delta_s)
            if not self.trail or self._trail_dist >= 0.03:
                self.trail.append((self.x, self.y))
                self._trail_dist = 0.0
                if len(self.trail) > 500:
                    self.trail = self.trail[-500:]

    def snapshot(self):
        with self._lock:
            return {
                "x": self.x,
                "y": self.y,
                "theta": self.theta,
                "linear": self.v,
                "angular": self.w,
                "goal_linear": self._goal_v,
                "goal_angular": self._goal_w,
                "wheel_left": self.wheel_left,
                "wheel_right": self.wheel_right,
                "trail": list(self.trail),
                "cmd_age": None if self._last_cmd_time <= 0.0 else time.time() - self._last_cmd_time,
            }
