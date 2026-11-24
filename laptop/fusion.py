"""
LiDAR <-> camera co-mapping.

Frames (all metres, right-handed):
  robot / LiDAR   x forward, y left, z up, origin on the LiDAR axis at floor level
  camera          optical centre at (CAM_X_M, CAM_Y_M, CAM_HEIGHT_M), yawed CAM_YAW_DEG (+ left) and pitched
                  CAM_PITCH_DEG (+ down) from the robot axis
  image           u right, v down, pinhole with f = (W/2) / tan(HFOV/2), principal point at the centre

A LiDAR return at range r, bearing b lies at (r cos b, r sin b, LIDAR_HEIGHT_M). Moving it into the camera frame
and dividing by depth gives its pixel. The camera yaw also absorbs the bearing bias the pipeline learns online
from LiDAR/camera agreement, so the projection tightens up by itself while the owner is tracked.

What this is used for:
  project_scan()   every visible beam as a pixel -> drawn over the live video (the "laser curtain")
  box_beams()      beams whose pixel column falls inside a detection box -> the LiDAR range of that person
  pixel_bearing()  inverse: image column -> robot-frame bearing (what the EKF measures)
"""

import math

import numpy as np

from config import Config


class CameraModel:
    def __init__(self, width, height, bias_rad=0.0):
        self.w, self.h = int(width), int(height)
        hfov = math.radians(Config.CAMERA_HFOV_DEG)
        self.f = (self.w / 2.0) / math.tan(hfov / 2.0)
        self.cx, self.cy = self.w / 2.0, self.h / 2.0
        self.yaw = math.radians(Config.CAM_YAW_DEG) + bias_rad
        self.pitch = math.radians(Config.CAM_PITCH_DEG)
        self.pos = np.array([Config.CAM_X_M, Config.CAM_Y_M, Config.CAM_HEIGHT_M], dtype=np.float64)
        self.z_lidar = Config.LIDAR_HEIGHT_M

    def project(self, x, y, z):
        """Robot-frame points -> (u, v, depth, in_front). Arrays in, arrays out."""
        dx = np.asarray(x, dtype=np.float64) - self.pos[0]
        dy = np.asarray(y, dtype=np.float64) - self.pos[1]
        dz = np.asarray(z, dtype=np.float64) - self.pos[2]
        cy_, sy_ = math.cos(self.yaw), math.sin(self.yaw)
        xf = cy_ * dx + sy_ * dy            # undo the camera yaw
        yl = -sy_ * dx + cy_ * dy
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        depth = cp * xf - sp * dz           # undo the pitch (tilted down by +pitch)
        up = sp * xf + cp * dz
        ok = depth > 0.05
        d = np.where(ok, depth, 1.0)
        u = self.cx - self.f * yl / d
        v = self.cy - self.f * up / d
        return u, v, depth, ok

    def project_scan(self, ranges, max_range=6.0):
        """ranges: per-bin metres (NaN = no return), bin i centred at (i + 0.5) * 360/n degrees.
        Returns dict of arrays for beams that land inside the image: idx, u, v, r."""
        r = np.asarray(ranges, dtype=np.float64)
        n = len(r)
        ang = (np.arange(n) + 0.5) * (2 * math.pi / n)
        ang = np.where(ang > math.pi, ang - 2 * math.pi, ang)
        valid = np.isfinite(r) & (r > 0.05) & (r < max_range)
        x, y = r * np.cos(ang), r * np.sin(ang)
        u, v, _, front = self.project(x, y, np.full(n, self.z_lidar))
        inside = valid & front & (u >= 0) & (u < self.w) & (v >= -self.h) & (v < 2 * self.h)
        idx = np.nonzero(inside)[0]
        return {"idx": idx, "u": u[idx], "v": v[idx], "r": r[idx], "bearing": ang[idx], "w": self.w, "h": self.h}

    def pixel_bearing(self, u):
        """Image column -> robot-frame bearing (rad, + left), ignoring the small camera translation."""
        return -math.atan((u - self.cx) / self.f) + self.yaw

    def box_beams(self, proj, bbox, pad_px=4):
        """Indices (into proj arrays) of beams whose column lies inside the box's horizontal extent."""
        x1, _, x2, _ = bbox
        return np.nonzero((proj["u"] >= x1 - pad_px) & (proj["u"] <= x2 + pad_px))[0]


def box_range(proj, sel, near=None, band=0.35):
    """Range of the person inside a box: the nearest compact group of beams (legs/torso in front of the wall).
    near: camera's own range guess; beams further than 40% from it are ignored when given.
    Returns (range_m, n_beams) or (None, 0)."""
    if len(sel) == 0:
        return None, 0
    r = proj["r"][sel]
    if near is not None and near > 0:
        r = r[np.abs(r - near) < 0.5 * max(near, 0.5)]
        if len(r) == 0:
            return None, 0
    r = np.sort(r)
    lo = r[0]
    group = r[r < lo + band]
    if len(group) < 2:
        return None, int(len(group))
    return float(np.median(group)), int(len(group))


def overlay_points(proj, person_idx=None, limit=120):
    """Compact list for the dashboard: [u_norm, v_norm, range_m, kind] with kind 1 = owner beams, 0 = other."""
    n = len(proj["idx"])
    if n == 0:
        return []
    step = max(1, n // limit)
    owner = set(int(i) for i in person_idx) if person_idx is not None else set()
    out = []
    for k in range(0, n, step):
        out.append([round(float(proj["u"][k]) / proj["w"], 4), round(float(proj["v"][k]) / proj["h"], 4),
                    round(float(proj["r"][k]), 2), 1 if int(proj["idx"][k]) in owner else 0])
    return out

