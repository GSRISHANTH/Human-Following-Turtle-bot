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

    def __init__(self, critical=0.30, warn=0.85, side=0.40):
        self.critical, self.warn, self.side = critical, warn, side
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
        return v, w, status


# colours are BGR
C_BG = (30, 24, 20)
C_FREE = (66, 56, 42)
C_WALL = (255, 205, 70)
C_STATIC = (255, 150, 60)
C_PERSON = (80, 230, 70)
C_MOVING = (40, 170, 255)


class LidarMap:
    """Occupancy grid (0.05 m cells, 12 m x 12 m) in a corrected odometry frame, with scan matching."""

    RES = 0.05
    N = 240
    VIEW = 460
    VIEW_RADIUS_M = 4.5
    WARMUP_SCANS = 25

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
        self.loc_shift = 0.0              # metres of correction applied so far
        self.resets = 0                   # odometry restarts detected
        self.trail = []
        self.clusters = []
        self.labels = []

    def reset(self):
        self.grid[:] = 0.0
        self.origin = None
        self.T = (0.0, 0.0, 0.0)
        self.scans = 0
        self.trail = []
        self.loc_shift = 0.0

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

    # --- localisation: correlative scan matching against the map built so far ---

    def _localize(self, pts_robot, pose):
        occ = (self.grid > 1.2).astype(np.float32)
        if occ.sum() < 150 or self.scans < self.WARMUP_SCANS or len(pts_robot) < 30:
            return pose
        field = cv2.GaussianBlur(occ, (0, 0), 1.6)
        field /= max(field.max(), 1e-6)
        sub = pts_robot[:: max(1, len(pts_robot) // 120)]
        best, best_score, zero_score = (0.0, 0.0, 0.0), -1.0, 0.0
        for dth in (-0.07, -0.035, 0.0, 0.035, 0.07):
            yaw = pose[2] + dth
            c, s = math.cos(yaw), math.sin(yaw)
            wx0 = pose[0] + c * sub[:, 0] - s * sub[:, 1]
            wy0 = pose[1] + s * sub[:, 0] + c * sub[:, 1]
            for dx in (-0.10, -0.05, 0.0, 0.05, 0.10):
                for dy in (-0.10, -0.05, 0.0, 0.05, 0.10):
                    gx, gy = self._cell(wx0 + dx, wy0 + dy)
                    ok = (gx >= 0) & (gx < self.N) & (gy >= 0) & (gy < self.N)
                    if ok.sum() < 20:
                        continue
                    score = float(field[gy[ok], gx[ok]].sum() / len(sub))
                    score -= 0.02 * (abs(dx) + abs(dy)) / 0.05 + 0.01 * abs(dth) / 0.035
                    if dx == 0.0 and dy == 0.0 and dth == 0.0:
                        zero_score = score
                    if score > best_score:
                        best, best_score = (dx, dy, dth), score
        self.loc_score = best_score
        if best_score < 0.25 or best_score - zero_score < 0.03:
            return pose
        k = 0.6  # damped, so one bad match cannot throw the pose
        self.loc_shift += k * math.hypot(best[0], best[1])
        return (pose[0] + k * best[0], pose[1] + k * best[1], wrap(pose[2] + k * best[2]))

    # --- update ---

    def update(self, bins_mm, odom):
        now = time.time()
        if self.stamp:
            self.dt_avg = 0.8 * self.dt_avg + 0.2 * (now - self.stamp)
            self.hz = 1.0 / max(self.dt_avg, 1e-3)
        self.stamp = now
        self.bins = to_meters(bins_mm)
        if odom is not None and not all(math.isfinite(v) for v in odom):
            odom = None                      # a NaN here would poison the pose, the trail and every later render
        prev = self.pose_odom
        if odom is not None and prev is not None:
            jump = math.hypot(odom[0] - prev[0], odom[1] - prev[1])
            if jump > 0.6 or abs(wrap(odom[2] - prev[2])) > math.radians(70):
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

        if odom is None:
            self.pose = None
            self.grid[:] = 0.0
            self.labels = ["static"] * len(self.clusters)
            return
        if self.origin is None:
            self.origin = (odom[0], odom[1])
        pose = self._corrected(odom)
        if len(pts_robot):
            pose = self._localize(pts_robot, pose)
            # fold the improvement back into T so the next odom sample starts from it
            tth = wrap(pose[2] - odom[2])
            c, s = math.cos(tth), math.sin(tth)
            self.T = (pose[0] - (c * odom[0] - s * odom[1]), pose[1] - (s * odom[0] + c * odom[1]), tth)
        self.pose = pose
        if not self.trail or math.hypot(pose[0] - self.trail[-1][0], pose[1] - self.trail[-1][1]) > 0.08:
            self.trail.append(pose[:2])
            self.trail = self.trail[-400:]

        self._classify(pose)
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

    def render(self, person=None, sec=None, plan=None):
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
            cv2.circle(view, p, int(0.24 * S), C_PERSON, 2, cv2.LINE_AA)
            tip = to_view(person["x"] + person["vx"], person["y"] + person["vy"])
            cv2.arrowedLine(view, p, tip, (60, 220, 255), 2, cv2.LINE_AA, tipLength=0.3)
            cv2.putText(view, f"PERSON {math.hypot(person['x'], person['y']):.2f}m", (p[0] + 10, p[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, C_PERSON, 1, cv2.LINE_AA)

        age = time.time() - self.stamp if self.stamp else None
        label = "LiDAR LDS-02  %.0f Hz" % self.hz if age is not None and age < 1.0 else "LiDAR: no data"
        cv2.putText(view, label, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1, cv2.LINE_AA)
        if self.pose is None and self.stamp:
            cv2.putText(view, "no odometry: current scan only", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4, C_MOVING, 1, cv2.LINE_AA)
        for i, (name, col) in enumerate((("person", C_PERSON), ("moving object", C_MOVING), ("static obstacle", C_STATIC), ("wall", C_WALL))):
            cv2.circle(view, (14, V - 14 - 16 * i), 4, col, -1, cv2.LINE_AA)
            cv2.putText(view, name, (24, V - 10 - 16 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 200, 200), 1, cv2.LINE_AA)
        return view
