"""
Extended Kalman filter for the followed person, in the robot frame (x forward, y left).

State   s = [px, py, ux, uy]   position (m) and ground velocity (m/s) of the person, robot frame
Predict constant-velocity person, with the robot's own motion removed. If the robot drives
        v m/s and yaws w rad/s over dt, a fixed point appears to move by
            p' = R(-w dt) (p + u dt - [v dt, 0]),   u' = R(-w dt) u
        which is linear in s, so F is exact. Process noise models a person's acceleration.
Measure z = [range, bearing]. The measurement is polar, so h(s) is nonlinear and H is its
        Jacobian. The camera and the LiDAR each give a range with its own noise:
            LiDAR  sigma_r = 0.05 m            (direct time-of-flight)
            camera sigma_r = 0.12 * range      (pinhole size cue, error grows with distance)
            bearing sigma = 1.5 deg (camera pinhole), 1.0 deg when the LiDAR agrees
Gate    A measurement whose Mahalanobis distance^2 exceeds 11.3 (99%, 3 dof is overkill, 2 dof 9.2)
        is rejected as a different person or a detection glitch.
"""

import math
import numpy as np


class PersonEKF:
    SIGMA_ACCEL = 0.9   # m/s^2, how abruptly a walking person changes velocity
    GATE_CHI2 = 11.3
    MAX_SPEED = 2.0

    def __init__(self):
        self.s = None
        self.P = None
        self.misses = 0
        self.rejects = 0
        self.last_t = None
        self.nis = 0.0

    @property
    def active(self):
        return self.s is not None

    def reset(self):
        self.s = self.P = self.last_t = None
        self.misses = self.rejects = 0

    def init(self, rng, bearing, t):
        self.s = np.array([rng * math.cos(bearing), rng * math.sin(bearing), 0.0, 0.0])
        self.P = np.diag([0.05, 0.05, 1.0, 1.0])
        self.last_t = t
        self.misses = self.rejects = 0

    def predict(self, t, v_robot=0.0, w_robot=0.0):
        if self.s is None:
            return
        dt = max(0.0, min(0.5, t - self.last_t))
        self.last_t = t
        if dt == 0.0:
            return
        c, sn = math.cos(-w_robot * dt), math.sin(-w_robot * dt)
        R = np.array([[c, -sn], [sn, c]])
        F = np.eye(4)
        F[:2, :2] = R
        F[:2, 2:] = R * dt
        F[2:, 2:] = R
        shift = np.array([-v_robot * dt, 0.0])
        self.s = F @ self.s
        self.s[:2] += R @ shift
        # discrete white-noise acceleration
        q = self.SIGMA_ACCEL ** 2
        Q = np.zeros((4, 4))
        Q[:2, :2] = np.eye(2) * q * dt ** 4 / 4
        Q[:2, 2:] = np.eye(2) * q * dt ** 3 / 2
        Q[2:, :2] = Q[:2, 2:]
        Q[2:, 2:] = np.eye(2) * q * dt ** 2
        self.P = F @ self.P @ F.T + Q

    def coast(self, t, v_robot, w_robot, tau=2.0):
        """Predict with no measurement. A person who just left view usually slows down, so velocity decays."""
        if self.s is None:
            return
        dt = max(0.0, min(0.5, t - self.last_t))
        self.predict(t, v_robot, w_robot)
        self.s[2:] *= math.exp(-dt / tau)
        self.misses += 1

    def update(self, rng, bearing, sigma_r, sigma_b):
        """Returns True if the measurement was accepted."""
        if self.s is None:
            return False
        px, py = self.s[0], self.s[1]
        r = max(math.hypot(px, py), 1e-3)
        h = np.array([r, math.atan2(py, px)])
        H = np.zeros((2, 4))
        H[0, 0], H[0, 1] = px / r, py / r
        H[1, 0], H[1, 1] = -py / (r * r), px / (r * r)
        z = np.array([rng, bearing])
        y = z - h
        y[1] = (y[1] + math.pi) % (2 * math.pi) - math.pi
        Rm = np.diag([sigma_r ** 2, sigma_b ** 2])
        S = H @ self.P @ H.T + Rm
        Sinv = np.linalg.inv(S)
        self.nis = float(y @ Sinv @ y)
        if self.nis > self.GATE_CHI2:
            self.rejects += 1
            return False
        K = self.P @ H.T @ Sinv
        self.s = self.s + K @ y
        speed = math.hypot(self.s[2], self.s[3])
        if speed > self.MAX_SPEED:
            self.s[2:] *= self.MAX_SPEED / speed
        I_KH = np.eye(4) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T  # Joseph form keeps P symmetric positive
        self.misses = 0
        self.rejects = 0
        return True

    # --- derived quantities used by the controller and the dashboard ---

    def range_bearing(self):
        px, py = self.s[0], self.s[1]
        return math.hypot(px, py), math.atan2(py, px)

    def range_rate(self):
        px, py, ux, uy = self.s
        r = max(math.hypot(px, py), 1e-3)
        return (px * ux + py * uy) / r

    def bearing_rate(self, w_robot=0.0):
        """d(bearing)/dt as seen from the robot, including the robot's own yaw rate."""
        px, py, ux, uy = self.s
        r2 = max(px * px + py * py, 1e-6)
        return (px * uy - py * ux) / r2 - w_robot

    def predicted(self, lead):
        """Where the person will be in `lead` seconds, robot not moving."""
        return self.s[0] + self.s[2] * lead, self.s[1] + self.s[3] * lead

    def as_dict(self):
        if self.s is None:
            return None
        r, b = self.range_bearing()
        return {"x": float(self.s[0]), "y": float(self.s[1]), "vx": float(self.s[2]), "vy": float(self.s[3]),
                "range": float(r), "bearing_deg": math.degrees(b), "speed": math.hypot(self.s[2], self.s[3]),
                "misses": self.misses, "nis": self.nis}
