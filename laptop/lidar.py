"""
2D LiDAR helpers (TurtleBot3 LDS-02 on /scan, forwarded by the Pi as 360 bins of 1 degree).

Frame: x forward, y left, bearing 0 straight ahead, positive counter-clockwise (ROS convention).

- segment():       splits a scan into object clusters (width, centroid, range, bearing)
- person_cluster():the cluster that matches the camera's person; its centroid is the LiDAR measurement
- sectors():       nearest obstacle ahead / left / right, with the followed person masked out
- LidarMap:        occupancy grid, scan-to-map localisation, object classification, map renderer
- LidarAvoidance:  reactive layer that bends and slows the PID command around obstacles
"""

import math
import time
import cv2
import numpy as np

try:
    from config import Config as _Config
except ImportError:          # lidar.py is also usable on its own (HIL map view)
    _Config = None


def Config_get(name, default):
    return getattr(_Config, name, default) if _Config is not None else default


FRONT_HALF_DEG = 25.0
SIDE_OUTER_DEG = 80.0

PERSON_WIDTH = (0.06, 0.80)    # a torso or a pair of legs seen from 1 m: not a wall, not a post


def to_meters(bins_mm):
    r = np.asarray(bins_mm, dtype=np.float32) / 1000.0
    r[r <= 0.0] = np.nan
    return r


def bin_angles(n):
    """Bin centre angles in (-pi, pi]."""
    a = (np.arange(n, dtype=np.float32) + 0.5) * (2 * math.pi / n)
    return np.where(a > math.pi, a - 2 * math.pi, a)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def segment(ranges, max_range=6.0):
    """Cluster neighbouring returns. A new cluster starts at a range jump or a gap in returns."""
    n = len(ranges)
    ang = bin_angles(n)
    valid = np.isfinite(ranges) & (ranges < max_range)
    clusters, cur = [], []
    for i in range(n):
        if not valid[i]:
            if cur:
                clusters.append(cur)
                cur = []
            continue
        if cur and abs(ranges[i] - ranges[cur[-1]]) > max(0.10, 0.07 * ranges[i]):
            clusters.append(cur)
            cur = []
        cur.append(i)
    if cur:
        clusters.append(cur)
    # an object dead ahead straddles the 0/360 degree seam
    if len(clusters) > 1 and clusters[0][0] == 0 and clusters[-1][-1] == n - 1             and abs(ranges[0] - ranges[n - 1]) <= max(0.10, 0.07 * ranges[0]):
        clusters[0] = clusters.pop() + clusters[0]
    out = [] 
    for idx in clusters:
        if len(idx) < 2:
            continue
        idx = np.asarray(idx)
        x = ranges[idx] * np.cos(ang[idx])
        y = ranges[idx] * np.sin(ang[idx])
        cx, cy = float(x.mean()), float(y.mean())
        out.append({
            "idx": idx, "x": x, "y": y, "cx": cx, "cy": cy,
            "width": float(math.hypot(x.max() - x.min(), y.max() - y.min())),
            "range": math.hypot(cx, cy), "bearing": math.atan2(cy, cx), "n": len(idx),
        })
    return _merge_legs(out)


def _merge_legs(clusters):
    """Two legs are two clusters a few centimetres apart. Merge neighbours that together are still person-sized."""
    merged = []
    for c in clusters:
        if merged:
            p = merged[-1]
            if math.hypot(c["cx"] - p["cx"], c["cy"] - p["cy"]) < 0.40:
                x = np.concatenate([p["x"], c["x"]])
                y = np.concatenate([p["y"], c["y"]])
                width = math.hypot(x.max() - x.min(), y.max() - y.min())
                if width <= PERSON_WIDTH[1]:
                    cx, cy = float(x.mean()), float(y.mean())
                    merged[-1] = {"idx": np.concatenate([p["idx"], c["idx"]]), "x": x, "y": y, "cx": cx, "cy": cy,
                                  "width": width, "range": math.hypot(cx, cy), "bearing": math.atan2(cy, cx),
                                  "n": len(x)}
                    continue
        merged.append(c)
    return merged


def person_cluster(clusters, bearing, near, window_deg=9.0):
    """Cluster of person size whose centroid is close to the camera's bearing and range."""
    best, best_cost = None, 1e9
    for c in clusters:
        if not (PERSON_WIDTH[0] <= c["width"] <= PERSON_WIDTH[1]):
            continue
        db = abs(wrap(c["bearing"] - bearing))
        if db > math.radians(window_deg):
            continue
        dr = abs(c["range"] - near)
        cost = dr + 1.5 * db
        if dr < 0.7 and cost < best_cost:
            best, best_cost = c, cost
    return best


def sectors(ranges, exclude=None, max_range=3.5):
    """exclude = (bearing_rad, half_width_rad, range_m): beams on the person are ignored."""
    ang = bin_angles(len(ranges))
    r = np.where(np.isfinite(ranges), ranges, max_range).astype(np.float32)
    if exclude is not None:
        bearing, half, rng = exclude
        on_person = (np.abs(wrap(ang - bearing)) <= half) & (np.abs(r - rng) < 0.5)
        r_masked = np.where(on_person, max_range, r)
    else:
        r_masked = r
    deg = np.degrees(ang)

    def nearest(src, lo, hi):
        m = (deg >= lo) & (deg <= hi)
        return float(src[m].min()) if m.any() else max_range

    return {
        "front": min(nearest(r_masked, -FRONT_HALF_DEG, FRONT_HALF_DEG), max_range),
        "left": nearest(r_masked, FRONT_HALF_DEG, SIDE_OUTER_DEG),
        "right": nearest(r_masked, -SIDE_OUTER_DEG, -FRONT_HALF_DEG),
        "front_raw": nearest(r, -15.0, 15.0),
    }


class LidarAvoidance:
    """Bends and slows the follow command. Distances are from the LiDAR centre, robot radius is 0.105 m."""

    def __init__(self, critical=0.30, warn=0.85, side=0.40, side_critical=0.20):
        self.critical, self.warn, self.side = critical, warn, side
        self.side_critical = side_critical    # robot radius 0.105 m + margin: inside this a side is a keep-out
        self.bias = 1.0

    def apply(self, v, w, sec):
        d_f, d_l, d_r = sec["front"], sec["left"], sec["right"]
        if d_l > d_r + 0.15:
            self.bias = 1.0
        elif d_r > d_l + 0.15:
            self.bias = -1.0
        status = "LIDAR_CLEAR"
        if d_f < self.critical:
            v = min(v, 0.0)
            w = 0.8 * self.bias
            status = "LIDAR_STOP"
        elif d_f < self.warn:
            k = (d_f - self.critical) / (self.warn - self.critical)
            v *= max(0.25, k)
            w += self.bias * (1.0 - k) * 0.7
            status = "LIDAR_AVOIDING"
        if d_l < self.side and d_l < d_r:
            w -= 0.5 * (self.side - d_l) / self.side
            status = status if status != "LIDAR_CLEAR" else "LIDAR_SIDE_L"
        elif d_r < self.side and d_r < d_l:
            w += 0.5 * (self.side - d_r) / self.side
            status = status if status != "LIDAR_CLEAR" else "LIDAR_SIDE_R"
        # Side keep-out. The nudge above is a bias the PID can overpower (it will turn hard toward a person
        # round a corner). Inside side_critical the robot may not turn toward that side at all, is pushed away,
        # and slows as the gap closes, so it cannot clip a door frame or wall corner it is passing.
        for d_side, sign in ((d_l, 1.0), (d_r, -1.0)):
            if d_side < self.side_critical:
                k = max(0.0, d_side / self.side_critical)
                push = 0.6 * (1.0 - k)
                w = min(w, -push) if sign > 0 else max(w, push)
                v *= max(0.35, k)
                if status in ("LIDAR_CLEAR", "LIDAR_SIDE_L", "LIDAR_SIDE_R"):
                    status = "LIDAR_SIDE_STOP"
        return v, w, status


# colours are BGR
C_BG = (30, 24, 20)
C_FREE = (66, 56, 42)
C_WALL = (255, 205, 70)
C_STATIC = (255, 150, 60)
C_PERSON = (80, 230, 70)
C_MOVING = (40, 170, 255)


class LidarMap:
    """2-D SLAM: log-odds occupancy grid (0.05 m cells, 12 m x 12 m window that follows the robot) and
    scan-to-map localisation. Pose sources, best first:
      'slam'  map-frame pose from slam_toolbox / Cartographer on the Pi (pose_src 2): used as is
      'odom'  wheel odometry from /odom, corrected by scan matching (the correction T is kept between scans)
      'scan'  no odometry at all: scan matching alone, with a constant-velocity prior (Hector-style)"""

    RES = 0.05
    N = 240
    VIEW = 460
    VIEW_RADIUS_M = 4.5
    WARMUP_SCANS = 10                 # scans integrated before matching starts (odometry source)

    def __init__(self):
        self.grid = np.zeros((self.N, self.N), dtype=np.float32)  # log-odds
        self.origin = None
        self.bins = None
        self.pose_odom = None
        self.T = (0.0, 0.0, 0.0)          # odom -> map correction found by scan matching
        self.pose = None                  # corrected pose (x, y, yaw), the best estimate
        self.stamp = 0.0
        self.hz = 0.0
        self.dt_avg = 0.1
        self.scans = 0
        self.loc_score = 0.0
        self.loc_ok = False
        self.loc_shift = 0.0              # metres of correction applied so far
        self.resets = 0                   # odometry restarts detected
        self.recenters = 0
        self.source = None                # 'slam' | 'odom' | 'scan'
        self.vel = (0.0, 0.0, 0.0)        # pose change per scan, for the scan-only prior
        self.trail = []
        self.clusters = []
        self.labels = []
        self.dynamic = []                 # per cluster: True = person / moving, kept out of the map
        self._persist = {}                # world 0.3 m bucket -> consecutive scans a "moving" cluster stayed there

    def reset(self):
        self.grid[:] = 0.0
        self.origin = None
        self.T = (0.0, 0.0, 0.0)
        self.scans = 0
        self.trail = []
        self.loc_shift = 0.0
        self.pose = None
        self.vel = (0.0, 0.0, 0.0)

    # --- frames ---

    def _corrected(self, odom):
        tx, ty, tth = self.T
        c, s = math.cos(tth), math.sin(tth)
        return (c * odom[0] - s * odom[1] + tx, s * odom[0] + c * odom[1] + ty, wrap(odom[2] + tth))

    def _cell(self, wx, wy):
        ox, oy = self.origin if self.origin is not None else (0.0, 0.0)
        gx = np.floor((np.asarray(wx) - ox) / self.RES + self.N / 2).astype(np.int32)
        gy = np.floor(self.N / 2 - (np.asarray(wy) - oy) / self.RES).astype(np.int32)
        return gx, gy

    def robot_to_world(self, x, y):
        px, py, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        return px + c * x - s * y, py + s * x + c * y

    def world_to_robot(self, wx, wy):
        px, py, yaw = self.pose
        c, s = math.cos(-yaw), math.sin(-yaw)
        dx, dy = wx - px, wy - py
        return c * dx - s * dy, s * dx + c * dy

    # --- localisation: multi-resolution correlative scan matching + Gauss-Newton refinement ---
    #
    # 1. Likelihood field  L(c) = exp(-d(c)^2 / 2 sigma^2), d = distance from cell c to the nearest occupied cell.
    # 2. Pyramid: level k is L max-pooled over 2^k x 2^k cells, so a coarse score is an upper bound of the fine one
    #    (Olson, "M3RSM", ICRA 2015). Search the whole window coarsely, then refine around the best few candidates.
    # 3. Gauss-Newton on the bilinearly interpolated field (Kohlbrecher et al., Hector SLAM, SSRR 2011) for a
    #    sub-cell answer.
    # Score = mean field value at the scan endpoints, in [0, 1].

    def _field(self):
        occ = (self.grid > 1.2).astype(np.uint8)
        if int(occ.sum()) < 60:
            return None
        dist = cv2.distanceTransform(1 - occ, cv2.DIST_L2, 5) * self.RES
        sig = max(0.02, float(Config_get("SLAM_SIGMA_M", 0.06)))
        field = np.exp(-(dist * dist) / (2.0 * sig * sig)).astype(np.float32)
        pyr = [field]
        for _ in range(2):
            f = pyr[-1]
            h, w = (f.shape[0] + 1) // 2 * 2, (f.shape[1] + 1) // 2 * 2
            pad = np.zeros((h, w), np.float32)
            pad[:f.shape[0], :f.shape[1]] = f
            pooled = pad.reshape(h // 2, 2, w // 2, 2).max(axis=(1, 3))
            # dilate by one so a coarse cell bounds every fine offset that maps near it
            pyr.append(cv2.dilate(pooled, np.ones((3, 3), np.uint8)))
        return pyr

    def _score_grid(self, field, scale, pts, pose, offsets):
        """pts (M,2) robot frame; offsets (K,2) world dx,dy. Returns K mean field values at pose (+offset)."""
        c, s = math.cos(pose[2]), math.sin(pose[2])
        wx = pose[0] + c * pts[:, 0] - s * pts[:, 1]
        wy = pose[1] + s * pts[:, 0] + c * pts[:, 1]
        ox, oy = self.origin
        res = self.RES * scale
        n = field.shape[0]
        gx = np.floor(((wx[None, :] + offsets[:, 0:1]) - ox) / res + n / 2).astype(np.int32)
        gy = np.floor(n / 2 - ((wy[None, :] + offsets[:, 1:2]) - oy) / res).astype(np.int32)
        ok = (gx >= 0) & (gx < n) & (gy >= 0) & (gy < n)
        vals = np.where(ok, field[np.clip(gy, 0, n - 1), np.clip(gx, 0, n - 1)], 0.0)
        return vals.mean(axis=1)

    def _gauss_newton(self, field, pts, pose, iters=4):
        """Hector-style refinement: minimise sum (1 - M(S(xi)))^2 on the bilinear field."""
        ox, oy = self.origin
        n = field.shape[0]
        x, y, th = pose
        for _ in range(iters):
            c, s = math.cos(th), math.sin(th)
            wx = x + c * pts[:, 0] - s * pts[:, 1]
            wy = y + s * pts[:, 0] + c * pts[:, 1]
            gx = (wx - ox) / self.RES + n / 2 - 0.5
            gy = n / 2 - (wy - oy) / self.RES - 0.5
            ok = (gx >= 0) & (gx < n - 1) & (gy >= 0) & (gy < n - 1)
            if ok.sum() < 20:
                break
            gx, gy = gx[ok], gy[ok]
            px, py = pts[ok, 0], pts[ok, 1]
            x0, y0 = np.floor(gx).astype(np.int32), np.floor(gy).astype(np.int32)
            fx, fy = gx - x0, gy - y0
            m00, m10 = field[y0, x0], field[y0, x0 + 1]
            m01, m11 = field[y0 + 1, x0], field[y0 + 1, x0 + 1]
            m = (1 - fy) * ((1 - fx) * m00 + fx * m10) + fy * ((1 - fx) * m01 + fx * m11)
            dmdgx = (1 - fy) * (m10 - m00) + fy * (m11 - m01)
            dmdgy = (1 - fx) * (m01 - m00) + fx * (m11 - m10)
            dmdx = dmdgx / self.RES                 # world x grows with grid x
            dmdy = -dmdgy / self.RES                # world y grows against grid y
            dth = dmdx * (-s * px - c * py) + dmdy * (c * px - s * py)
            J = np.stack([dmdx, dmdy, dth], axis=1)
            H = J.T @ J + np.diag([1e-3, 1e-3, 1e-4])
            g = J.T @ (1.0 - m)
            try:
                d = np.linalg.solve(H, g)
            except np.linalg.LinAlgError:
                break
            d[:2] = np.clip(d[:2], -0.05, 0.05)
            d[2] = float(np.clip(d[2], -0.05, 0.05))
            x, y, th = x + float(d[0]), y + float(d[1]), wrap(th + float(d[2]))
            if abs(d[0]) < 1e-3 and abs(d[1]) < 1e-3 and abs(d[2]) < 1e-3:
                break
        return (x, y, th)

    def _localize(self, pts_robot, pose):
        """Returns the scan-matched pose, or the prior when the match is not trustworthy."""
        self.loc_ok = False
        if len(pts_robot) < 30 or self.origin is None:
            return pose
        pyr = self._field()
        if pyr is None:
            return pose
        sub = pts_robot[:: max(1, len(pts_robot) // 150)].astype(np.float32)
        win = float(Config_get("SLAM_SEARCH_XY_M", 0.30))
        ywin = math.radians(float(Config_get("SLAM_SEARCH_YAW_DEG", 15.0)))
        prior_score = float(self._score_grid(pyr[0], 1, sub, pose, np.zeros((1, 2), np.float32))[0])

        # coarse: level 1 (0.10 m) over the whole window, 1.5 degree yaw steps
        step = self.RES * 2
        k = int(math.ceil(win / step))
        g = np.arange(-k, k + 1, dtype=np.float32) * step
        offs = np.stack(np.meshgrid(g, g), axis=-1).reshape(-1, 2)
        cands = []
        for dth in np.arange(-ywin, ywin + 1e-9, math.radians(1.5)):
            p = (pose[0], pose[1], wrap(pose[2] + float(dth)))
            sc = self._score_grid(pyr[1], 2, sub, p, offs)
            # gentle preference for the motion prior so a featureless corridor does not slide
            sc = sc - 0.04 * (np.hypot(offs[:, 0], offs[:, 1]) / max(win, 1e-3)) - 0.03 * abs(float(dth)) / max(ywin, 1e-3)
            top = np.argsort(sc)[-3:]
            cands += [(float(sc[i]), float(offs[i, 0]), float(offs[i, 1]), float(dth)) for i in top]
        cands.sort(reverse=True)

        # fine: level 0 (0.05 m) around the best coarse candidates, 0.5 degree yaw steps
        fg = np.arange(-2, 3, dtype=np.float32) * self.RES
        foffs = np.stack(np.meshgrid(fg, fg), axis=-1).reshape(-1, 2)
        best, best_score = None, -1.0
        for _, dx, dy, dth in cands[:4]:
            for ddth in np.radians([-1.0, -0.5, 0.0, 0.5, 1.0]):
                p = (pose[0] + dx, pose[1] + dy, wrap(pose[2] + dth + float(ddth)))
                sc = self._score_grid(pyr[0], 1, sub, p, foffs)
                i = int(np.argmax(sc))
                if sc[i] > best_score:
                    best_score = float(sc[i])
                    best = (p[0] + float(foffs[i, 0]), p[1] + float(foffs[i, 1]), p[2])
        if best is None:
            return pose
        refined = self._gauss_newton(pyr[0], sub, best)
        ref_score = float(self._score_grid(pyr[0], 1, sub, refined, np.zeros((1, 2), np.float32))[0])
        if ref_score >= best_score:
            best, best_score = refined, ref_score
        self.loc_score = best_score
        # accept only a clearly good match that is at least as good as where the prior already is
        if best_score < float(Config_get("SLAM_MIN_SCORE", 0.35)) or best_score < prior_score - 0.01:
            self.loc_score = max(best_score, prior_score)
            return pose
        self.loc_ok = True
        self.loc_shift += math.hypot(best[0] - pose[0], best[1] - pose[1])
        return best

    def _recenter(self, pose):
        """Keep the robot inside the fixed-size grid: shift contents (and origin) in whole cells."""
        if self.origin is None:
            return
        margin = float(Config_get("SLAM_RECENTER_M", 2.5))
        half = self.N * self.RES / 2
        dx, dy = pose[0] - self.origin[0], pose[1] - self.origin[1]
        if abs(dx) < half - margin and abs(dy) < half - margin:
            return
        sx, sy = int(round(dx / self.RES)), int(round(dy / self.RES))
        shifted = np.zeros_like(self.grid)
        # world +x is grid +col, world +y is grid -row
        src = self.grid
        h = w = self.N
        r0, c0 = max(0, -sy), max(0, sx)       # rows/cols of src that survive
        r1, c1 = min(h, h - sy), min(w, w + sx)
        if r0 < r1 and c0 < c1:
            shifted[r0 + sy:r1 + sy, c0 - sx:c1 - sx] = src[r0:r1, c0:c1]
        self.grid = shifted
        self.origin = (self.origin[0] + sx * self.RES, self.origin[1] + sy * self.RES)
        self.recenters += 1

    # --- update ---

    def update(self, bins_mm, odom, person=None, pose_src=None):
        """bins_mm: 360 ranges in mm (0 = no return). odom: (x, y, yaw) or None.
        person: owner position (x, y) in the robot frame, kept out of the map. pose_src: 'odom' (default when
        odom is given) or 'slam' (odom is already a map-frame pose from slam_toolbox / Cartographer)."""
        now = time.time()
        if self.stamp:
            self.dt_avg = 0.8 * self.dt_avg + 0.2 * (now - self.stamp)
            self.hz = 1.0 / max(self.dt_avg, 1e-3)
        self.stamp = now
        self.bins = to_meters(bins_mm)
        if odom is not None and not all(math.isfinite(v) for v in odom):
            odom = None                      # a NaN here would poison the pose, the trail and every later render
        if odom is None:
            src = "scan"
        elif pose_src == "slam" and Config_get("SLAM_USE_EXTERNAL", True):
            src = "slam"
        else:
            src = "odom"
        if self.source is not None and src != self.source:
            # a different frame: odometry and the SLAM map frame do not line up, so start a fresh map
            self.reset()
            self.resets += 1
        self.source = src
        prev = self.pose_odom
        if odom is not None and prev is not None:
            jump = math.hypot(odom[0] - prev[0], odom[1] - prev[1])
            # slam_toolbox may correct by more than odometry ever jumps (loop closure): allow it a wider gate
            lim, ylim = (0.6, math.radians(70)) if src == "odom" else (1.5, math.radians(120))
            if jump > lim or abs(wrap(odom[2] - prev[2])) > ylim:
                # The robot cannot do that between two scans: odometry restarted (Pi or driver rebooted).
                # The old map is in a different frame, so start a new one instead of smearing both together.
                self.reset()
                self.resets += 1
        self.pose_odom = odom
        n = len(self.bins)
        ang_r = bin_angles(n)
        hit = np.isfinite(self.bins)
        pts_robot = np.stack([self.bins[hit] * np.cos(ang_r[hit]), self.bins[hit] * np.sin(ang_r[hit])], axis=1)
        self.clusters = segment(self.bins)

        last = self.pose
        if src == "slam":
            if self.origin is None:
                self.origin = (odom[0], odom[1])
            pose = tuple(odom)
            self.loc_ok, self.loc_score = True, 1.0
        elif src == "odom":
            if self.origin is None:
                self.origin = (odom[0], odom[1])
            pose = self._corrected(odom)
            if len(pts_robot) and self.scans >= self.WARMUP_SCANS:
                pose = self._localize(pts_robot, pose)
            # fold the improvement back into T so the next odom sample starts from it
            tth = wrap(pose[2] - odom[2])
            c, s = math.cos(tth), math.sin(tth)
            self.T = (pose[0] - (c * odom[0] - s * odom[1]), pose[1] - (s * odom[0] + c * odom[1]), tth)
        else:
            # no odometry: scan matching is the odometry. Prior = last pose moved by the last pose change.
            if self.origin is None or last is None:
                self.origin = self.origin or (0.0, 0.0)
                pose = (0.0, 0.0, 0.0) if last is None else last
            else:
                vx, vy, vth = self.vel
                pose = self._localize(pts_robot, (last[0] + vx, last[1] + vy, wrap(last[2] + vth)))
                if not self.loc_ok:
                    pose = last      # nothing better than standing still: never integrate a guessed pose
        if last is not None and src != "slam":
            d = (pose[0] - last[0], pose[1] - last[1], wrap(pose[2] - last[2]))
            self.vel = tuple(0.5 * a + 0.5 * b for a, b in zip(self.vel, d))
        self.pose = pose
        self._recenter(pose)
        if not self.trail or math.hypot(pose[0] - self.trail[-1][0], pose[1] - self.trail[-1][1]) > 0.08:
            self.trail.append(pose[:2])
            self.trail = self.trail[-400:]

        self._classify(pose)
        # A new object that stays put (a box set down, a door closed) is static after ~8 scans and is mapped;
        # a walking person never stays in one 0.3 m bucket that long.
        c, s = math.cos(pose[2]), math.sin(pose[2])
        seen = {}
        for i, (cl, label) in enumerate(zip(self.clusters, self.labels)):
            if label != "moving":
                continue
            key = (int(math.floor((pose[0] + c * cl["cx"] - s * cl["cy"]) / 0.3)),
                   int(math.floor((pose[1] + s * cl["cx"] + c * cl["cy"]) / 0.3)))
            seen[key] = self._persist.get(key, 0) + 1
            if seen[key] >= 8:
                self.labels[i] = "static"
        self._persist = seen
        self.dynamic = [label == "moving" or (person is not None and
                        math.hypot(cl["cx"] - person[0], cl["cy"] - person[1]) < 0.6)
                        for cl, label in zip(self.clusters, self.labels)]
        # scan-only mapping trusts a match only once something is mapped; a bad match is never integrated
        if src != "scan" or self.loc_ok or self.scans < 3:
            self._integrate(pose, ang_r, hit)
        self.scans += 1

    def _classify(self, pose):
        """static = was already occupied or unseen-until-now thin structure; moving = appeared in space
        that was observed free. The person is relabelled by the caller via set_person()."""
        labels = []
        c, s = math.cos(pose[2]), math.sin(pose[2])
        for cl in self.clusters:
            wx = pose[0] + c * cl["x"] - s * cl["y"]
            wy = pose[1] + s * cl["x"] + c * cl["y"]
            gx, gy = self._cell(wx, wy)
            ok = (gx >= 0) & (gx < self.N) & (gy >= 0) & (gy < self.N)
            if self.scans < self.WARMUP_SCANS or not ok.any():
                labels.append("static")
                continue
            # "new" = every cell within two cells of the return was observed free before. A wall that is merely
            # re-observed has occupied cells beside it, so it is never new; unknown space is not new either.
            g = self.grid
            prior_max = np.array([g[max(0, y - 2):y + 3, max(0, x - 2):x + 3].max() for x, y in zip(gx[ok], gy[ok])])
            moved_in = float((prior_max < -0.4).mean())
            if moved_in > 0.6 and cl["width"] < 1.2:
                labels.append("moving")
            elif cl["width"] > 0.9:
                labels.append("wall")
            else:
                labels.append("static")
        self.labels = labels

    def _integrate(self, pose, ang_r, hit):
        n = len(self.bins)
        yaw_ang = ang_r + pose[2]
        r_free = np.where(hit, np.maximum(self.bins - 0.10, 0.0), 3.0)   # stop short of the surface so grazing beams do not erase it
        fx, fy = pose[0] + np.cos(yaw_ang) * r_free, pose[1] + np.sin(yaw_ang) * r_free
        gx, gy = self._cell(fx, fy)
        rx, ry = self._cell(pose[0], pose[1])
        poly = np.stack([np.concatenate([[rx], gx]), np.concatenate([[ry], gy])], axis=1).astype(np.int32)
        free = np.zeros((self.N, self.N), dtype=np.uint8)
        cv2.fillPoly(free, [poly], 1)
        self.grid[free > 0] -= 0.30

        # people and moving objects are not walls: their returns clear free space but never mark it occupied,
        # so the planner does not route round where the owner stood a second ago
        hit = hit.copy()
        for cl, dyn in zip(self.clusters, self.dynamic):
            if dyn:
                hit[cl["idx"]] = False
        hx, hy = pose[0] + np.cos(yaw_ang) * self.bins, pose[1] + np.sin(yaw_ang) * self.bins
        hgx, hgy = self._cell(np.where(hit, hx, 0.0), np.where(hit, hy, 0.0))
        occ = np.zeros((self.N, self.N), dtype=np.uint8)
        pts = [(int(hgx[i]), int(hgy[i])) for i in range(n) if hit[i]]
        # join neighbouring returns on one surface so walls are solid instead of dotted
        # (the LDS-02 drops returns at range, so bridge up to 4 missing beams when the surface is clearly continuous)
        idx = np.nonzero(hit)[0]
        for a_i, i in enumerate(idx):
            for k in range(1, 6):
                if a_i + k >= len(idx):
                    break
                j = idx[a_i + k]
                gap = j - i
                if gap > 5:
                    break
                if abs(self.bins[i] - self.bins[j]) < max(0.08, 0.06 * self.bins[i]) * (1 + 0.5 * (gap - 1)):
                    cv2.line(occ, (int(hgx[i]), int(hgy[i])), (int(hgx[j]), int(hgy[j])), 1, 1)
                    break
        for x, y in pts:
            if 0 <= x < self.N and 0 <= y < self.N:
                occ[y, x] = 1
        self.grid[occ > 0] += 1.0
        np.clip(self.grid, -4.0, 6.0, out=self.grid)

    # --- rendering ---

    def render(self, person=None, sec=None, plan=None, hfov_deg=None, cam_yaw_deg=0.0):
        V = self.VIEW
        S = V / (2 * self.VIEW_RADIUS_M)
        cx = cy = V / 2.0
        pose = self.pose if self.pose is not None else (0.0, 0.0, 0.0)
        yaw = pose[2]
        if self.origin is not None and self.pose is not None:
            img = np.empty((self.N, self.N, 3), dtype=np.uint8)
            img[:] = C_BG
            img[cv2.blur(self.grid, (3, 3)) < -0.3] = C_FREE
            wall = cv2.dilate((self.grid > 0.9).astype(np.uint8), np.ones((2, 2), np.uint8)) > 0
            img[wall] = C_WALL

            def gpx(wx, wy):
                gx, gy = self._cell(np.array([wx]), np.array([wy]))
                return [float(gx[0]) + 0.5, float(gy[0]) + 0.5]
            cyaw, syaw = math.cos(yaw), math.sin(yaw)
            src = np.float32([gpx(pose[0], pose[1]), gpx(pose[0] + cyaw, pose[1] + syaw), gpx(pose[0] - syaw, pose[1] + cyaw)])
            dst = np.float32([[cx, cy], [cx, cy - S], [cx - S, cy]])
            view = cv2.warpAffine(img, cv2.getAffineTransform(src, dst), (V, V), flags=cv2.INTER_LINEAR, borderValue=C_BG)
        else:
            view = np.full((V, V, 3), C_BG, dtype=np.uint8)

        for m in (1, 2, 3, 4):
            cv2.circle(view, (int(cx), int(cy)), int(m * S), (80, 72, 60), 1, cv2.LINE_AA)
            cv2.putText(view, f"{m}m", (int(cx) + 3, int(cy - m * S) - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (140, 130, 115), 1, cv2.LINE_AA)

        def to_view(xf, yl):
            return int(cx - yl * S), int(cy - xf * S)

        # camera field of view: what the camera can see, so LiDAR-only tracking is visibly "outside the picture"
        if hfov_deg:
            ov = view.copy()
            a0, a1 = math.radians(cam_yaw_deg - hfov_deg / 2), math.radians(cam_yaw_deg + hfov_deg / 2)
            fov = [to_view(0, 0)] + [to_view(4.0 * math.cos(a), 4.0 * math.sin(a)) for a in np.linspace(a0, a1, 12)]
            cv2.fillPoly(ov, [np.array(fov, dtype=np.int32)], (90, 70, 40))
            view = cv2.addWeighted(ov, 0.25, view, 0.75, 0)
            for a in (a0, a1):
                cv2.line(view, to_view(0, 0), to_view(4.0 * math.cos(a), 4.0 * math.sin(a)), (150, 120, 70), 1, cv2.LINE_AA)
            cv2.putText(view, "CAMERA", to_view(1.2, 0.05 + 1.2 * math.tan(min(a1, 1.2))), cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                        (180, 150, 100), 1, cv2.LINE_AA)

        # sectors first so the points sit on top
        if sec is not None:
            ov = view.copy()
            for name, (lo, hi) in (("front", (-FRONT_HALF_DEG, FRONT_HALF_DEG)), ("left", (FRONT_HALF_DEG, SIDE_OUTER_DEG)),
                                   ("right", (-SIDE_OUTER_DEG, -FRONT_HALF_DEG))):
                d = sec[name]
                col = (60, 60, 255) if d < 0.35 else (40, 170, 255) if d < 0.85 else (90, 210, 90)
                poly = [to_view(0, 0)] + [to_view(min(d, 1.2) * math.cos(math.radians(a)), min(d, 1.2) * math.sin(math.radians(a)))
                                          for a in np.linspace(lo, hi, 9)]
                cv2.fillPoly(ov, [np.array(poly, dtype=np.int32)], col)
            view = cv2.addWeighted(ov, 0.20, view, 0.80, 0)

        # planned path, in map coordinates
        if plan and plan.get("path") and self.pose is not None:
            pts = [to_view(*self.world_to_robot(x, y)) for x, y in plan["path"]]
            for a, b in zip(pts, pts[1:]):
                cv2.line(view, a, b, (255, 120, 255), 2, cv2.LINE_AA)
            if plan.get("goal"):
                g = to_view(*self.world_to_robot(*plan["goal"]))
                cv2.drawMarker(view, g, (255, 120, 255), cv2.MARKER_DIAMOND, 12, 2, cv2.LINE_AA)

        # where the owner was last seen, remembered in the map while the camera cannot see them
        if plan and plan.get("memory") and plan.get("mem_world") and self.pose is not None:
            mx, my = to_view(*self.world_to_robot(*plan["mem_world"]))
            ov = view.copy()
            cv2.line(ov, (int(cx), int(cy)), (mx, my), (255, 120, 255), 1, cv2.LINE_AA)
            view = cv2.addWeighted(ov, 0.5, view, 0.5, 0)
            cv2.drawMarker(view, (mx, my), (80, 230, 70), cv2.MARKER_TILTED_CROSS, 18, 2, cv2.LINE_AA)
            cv2.circle(view, (mx, my), int(0.3 * S), (80, 230, 70), 1, cv2.LINE_AA)
            cv2.putText(view, f"LAST SEEN {plan.get('age', 0):.0f}s ago", (mx + 12, my + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (80, 230, 70), 1, cv2.LINE_AA)

        # current scan, coloured by what the cluster is
        person_xy = (person["x"], person["y"]) if person else None
        if self.bins is not None:
            ang = bin_angles(len(self.bins))
            for cl, label in zip(self.clusters, self.labels):
                col = C_WALL if label == "wall" else C_MOVING if label == "moving" else C_STATIC
                # both legs are separate clusters, so anything within 0.7 m of the tracked person is the person
                is_person = person_xy is not None and math.hypot(cl["cx"] - person_xy[0], cl["cy"] - person_xy[1]) < 0.7 \
                    and cl["width"] <= PERSON_WIDTH[1] + 0.2
                if is_person:
                    col = C_PERSON
                for x, y in zip(cl["x"], cl["y"]):
                    if math.hypot(x, y) <= self.VIEW_RADIUS_M:
                        cv2.circle(view, to_view(x, y), 2, col, -1, cv2.LINE_AA)
                if label == "moving" and not is_person and cl["range"] < self.VIEW_RADIUS_M:
                    p = to_view(cl["cx"], cl["cy"])
                    cv2.circle(view, p, int(0.28 * S), C_MOVING, 1, cv2.LINE_AA)
                    cv2.putText(view, "MOVING", (p[0] + 8, p[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.36, C_MOVING, 1, cv2.LINE_AA)

        if len(self.trail) > 1 and self.pose is not None:
            try:
                tp = [to_view(*self.world_to_robot(x, y)) for x, y in self.trail[-120:]
                      if math.isfinite(x) and math.isfinite(y)]
                if len(tp) > 1:
                    cv2.polylines(view, [np.array(tp, dtype=np.int32)], False, (200, 200, 120), 1, cv2.LINE_AA)
            except (ValueError, OverflowError):
                self.trail = []

        cv2.circle(view, (int(cx), int(cy)), max(4, int(0.105 * S)), (255, 240, 120), 2, cv2.LINE_AA)
        cv2.line(view, (int(cx), int(cy)), (int(cx), int(cy - 0.16 * S)), (255, 240, 120), 2, cv2.LINE_AA)

        if person:
            p = to_view(person["x"], person["y"])
            cov = person.get("cov")
            if cov:
                # 2-sigma EKF position ellipse. View axes: u = -y, v = -x, so Pv = A P A^T with A = [[0,-1],[-1,0]].
                Pv = np.array([[cov[2], cov[1]], [cov[1], cov[0]]])
                ev, evec = np.linalg.eigh(Pv)                       # ascending
                sd = 2.0 * np.sqrt(np.clip(ev, 1e-6, 4.0))
                ang = math.degrees(math.atan2(evec[1, 1], evec[0, 1]))  # major axis, image convention (y down)
                axes = (max(2, int(sd[1] * S)), max(2, int(sd[0] * S)))
                cv2.ellipse(view, p, axes, ang, 0, 360, (60, 220, 255), 1, cv2.LINE_AA)
            cv2.circle(view, p, int(0.24 * S), C_PERSON, 2, cv2.LINE_AA)
            tip = to_view(person["x"] + person["vx"], person["y"] + person["vy"])
            cv2.arrowedLine(view, p, tip, (60, 220, 255), 2, cv2.LINE_AA, tipLength=0.3)
            cv2.putText(view, f"PERSON {math.hypot(person['x'], person['y']):.2f}m", (p[0] + 10, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, C_PERSON, 1, cv2.LINE_AA)

        age = time.time() - self.stamp if self.stamp else None
        label = "LiDAR LDS-02  %.0f Hz" % self.hz if age is not None and age < 1.0 else "LiDAR: no data"
        cv2.putText(view, label, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1, cv2.LINE_AA)
        if self.stamp and self.source:
            src = {"slam": "SLAM pose from Pi (slam_toolbox)", "odom": "odometry + scan matching",
                   "scan": "scan matching only (no odometry)"}[self.source]
            col = (120, 230, 120) if self.loc_ok or self.source == "slam" else (60, 180, 255)
            cv2.putText(view, f"{src}  match {self.loc_score:.2f}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)
        for i, (name, col) in enumerate((("person", C_PERSON), ("moving object", C_MOVING), ("static obstacle", C_STATIC), ("wall", C_WALL))):
            cv2.circle(view, (14, V - 14 - 16 * i), 4, col, -1, cv2.LINE_AA)
            cv2.putText(view, name, (24, V - 10 - 16 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 200, 200), 1, cv2.LINE_AA)
        return view
