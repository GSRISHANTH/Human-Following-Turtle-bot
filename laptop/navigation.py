"""
Path planning on the LiDAR occupancy grid.

Goal: the spot on the line robot -> person that is `desired` metres short of the person.
If that line is clear of inflated obstacles the follower just drives at the person. If it is blocked,
A* finds a way round and the controller steers at a waypoint 0.6 m ahead along the path.

Costmap: occupied cells inflated by robot radius + margin, the person's own blob and the robot's
footprint cleared so neither blocks the plan, free space near obstacles costs more so paths keep to the middle.
Planning runs on a 0.10 m grid (the map is 0.05 m).
"""

import heapq
import math
import numpy as np
import cv2

CELL = 0.10
INFLATE_M = 0.22      # robot radius 0.105 m + margin
LOOKAHEAD_M = 0.6


def _los(free, a, b):
    """True if the straight line between grid cells a and b (x, y) is free."""
    (x0, y0), (x1, y1) = a, b
    n = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
    xs = np.linspace(x0, x1, n).round().astype(int)
    ys = np.linspace(y0, y1, n).round().astype(int)
    return bool(free[ys, xs].all())


class Planner:
    def __init__(self, map_res=0.05, map_n=240):
        self.map_res, self.map_n = map_res, map_n
        self.k = int(round(CELL / map_res))
        self.n = map_n // self.k

    def _costmap(self, grid, person_cell, robot_cell):
        occ = (grid > 1.2).astype(np.uint8)
        k = self.k
        coarse = occ[: self.n * k, : self.n * k].reshape(self.n, k, self.n, k).max(axis=(1, 3))
        r = max(1, int(round(INFLATE_M / CELL)))
        inflated = cv2.dilate(coarse, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1)))
        if person_cell is not None:
            cv2.circle(inflated, person_cell, int(round(0.55 / CELL)), 0, -1)
        cv2.circle(inflated, robot_cell, int(round(0.30 / CELL)), 0, -1)
        free = inflated == 0
        dist = cv2.distanceTransform(free.astype(np.uint8), cv2.DIST_L2, 3)
        return free, dist

    def _to_cell(self, origin, wx, wy):
        gx = (wx - origin[0]) / CELL + self.n / 2
        gy = self.n / 2 - (wy - origin[1]) / CELL
        return int(math.floor(gx)), int(math.floor(gy))

    def _to_world(self, origin, cx, cy):
        return origin[0] + (cx + 0.5 - self.n / 2) * CELL, origin[1] + (self.n / 2 - cy - 0.5) * CELL

    def plan(self, grid, origin, pose, person_xy_robot, desired):
        """Returns dict(blocked, path, goal, wp_bearing, status). Coordinates in the map frame."""
        if origin is None or pose is None or person_xy_robot is None:
            return {"blocked": False, "path": [], "goal": None, "wp_bearing": None, "status": "NO_TARGET"}
        c, s = math.cos(pose[2]), math.sin(pose[2])
        px = pose[0] + c * person_xy_robot[0] - s * person_xy_robot[1]
        py = pose[1] + s * person_xy_robot[0] + c * person_xy_robot[1]
        dx, dy = px - pose[0], py - pose[1]
        dist = math.hypot(dx, dy)
        if dist <= desired + 0.05:
            return {"blocked": False, "path": [], "goal": None, "wp_bearing": None, "status": "AT_GOAL"}
        gx, gy = px - dx / dist * desired, py - dy / dist * desired

        start = self._to_cell(origin, pose[0], pose[1])
        goal = self._to_cell(origin, gx, gy)
        person_cell = self._to_cell(origin, px, py)
        n = self.n
        inside = lambda p: 0 <= p[0] < n and 0 <= p[1] < n
        if not (inside(start) and inside(goal)):
            return {"blocked": False, "path": [], "goal": None, "wp_bearing": None, "status": "OFF_MAP"}
        free, dist_t = self._costmap(grid, person_cell if inside(person_cell) else None, start)

        if not free[goal[1], goal[0]]:   # nudge the goal to the nearest free cell
            ys, xs = np.nonzero(free)
            if len(xs) == 0:
                return {"blocked": True, "path": [], "goal": (gx, gy), "wp_bearing": None, "status": "NO_PATH"}
            i = int(np.argmin((xs - goal[0]) ** 2 + (ys - goal[1]) ** 2))
            goal = (int(xs[i]), int(ys[i]))
        gwx, gwy = self._to_world(origin, *goal)

        if _los(free, start, goal):
            return {"blocked": False, "path": [(pose[0], pose[1]), (gwx, gwy)], "goal": (gwx, gwy),
                    "wp_bearing": None, "status": "DIRECT"}

        path = self._astar(free, dist_t, start, goal)
        if path is None:
            return {"blocked": True, "path": [], "goal": (gwx, gwy), "wp_bearing": None, "status": "NO_PATH"}
        path = self._smooth(free, path)
        world = [self._to_world(origin, x, y) for x, y in path]
        world[0] = (pose[0], pose[1])
        # waypoint LOOKAHEAD_M along the path
        acc, wp = 0.0, world[-1]
        for a, b in zip(world, world[1:]):
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            if acc + seg >= LOOKAHEAD_M:
                t = (LOOKAHEAD_M - acc) / max(seg, 1e-6)
                wp = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
                break
            acc += seg
        bearing = (math.atan2(wp[1] - pose[1], wp[0] - pose[0]) - pose[2] + math.pi) % (2 * math.pi) - math.pi
        return {"blocked": True, "path": world, "goal": (gwx, gwy), "wp_bearing": bearing, "status": "DETOUR"}

    def _astar(self, free, dist_t, start, goal):
        n = self.n
        sx, sy = start
        g = {(sx, sy): 0.0}
        came = {}
        heap = [(0.0, 0.0, (sx, sy))]
        steps = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414))
        expansions = 0
        while heap and expansions < 14000:
            _, cost, cur = heapq.heappop(heap)
            if cur == goal:
                out = [cur]
                while cur in came:
                    cur = came[cur]
                    out.append(cur)
                return out[::-1]
            if cost > g.get(cur, 1e9):
                continue
            expansions += 1
            for ddx, ddy, w in steps:
                nx, ny = cur[0] + ddx, cur[1] + ddy
                if not (0 <= nx < n and 0 <= ny < n) or not free[ny, nx]:
                    continue
                if ddx and ddy and not (free[cur[1], nx] and free[ny, cur[0]]):
                    continue  # do not cut a corner
                near_wall = max(0.0, 3.0 - dist_t[ny, nx]) * 0.35
                nc = cost + w + near_wall
                if nc < g.get((nx, ny), 1e9):
                    g[(nx, ny)] = nc
                    came[(nx, ny)] = cur
                    h = math.hypot(goal[0] - nx, goal[1] - ny)
                    heapq.heappush(heap, (nc + h, nc, (nx, ny)))
        return None

    def _smooth(self, free, path):
        """String pulling: keep only the corners that line of sight needs."""
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not _los(free, path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out
