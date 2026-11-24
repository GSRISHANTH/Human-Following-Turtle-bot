"""Simulated room for the HIL bench: 360-beam LiDAR raycast against walls and boxes, LDS-02-like noise.

Room 6 m x 4 m centred on the start pose, plus furniture. The scan is in the robot frame, one bin per degree,
bin 0 straight ahead, counter-clockwise, millimetres, 0 = no return (tb_proto wire format).
"""

import math
import random

ROOM_HALF = (3.0, 2.0)
FURNITURE = [(-1.0, 0.9, -0.6, 1.3), (1.0, 1.0, 1.4, 1.4), (1.6, -1.6, 2.4, -1.2), (-2.2, -1.5, -1.8, -0.9)]
MAX_RANGE = 3.5            # LDS-02 nominal; beyond this: no return
MIN_RANGE = 0.16


def raycast(pose, boxes=FURNITURE, half=ROOM_HALF, n=360, noise_m=0.01, dropout=0.02, rng=random):
    x0, y0, yaw = pose
    out = []
    for i in range(n):
        a = yaw + (i + 0.5) * 2 * math.pi / n
        dx, dy = math.cos(a), math.sin(a)
        best = 1e9
        for d, o, h in ((dx, x0, half[0]), (dy, y0, half[1])):
            if abs(d) > 1e-9:
                for wall in (-h, h):
                    t = (wall - o) / d
                    if t > 0:
                        best = min(best, t)
        for (bx0, by0, bx1, by1) in boxes:
            tmin, tmax = 0.0, 1e9
            for o, d, lo, hi in ((x0, dx, bx0, bx1), (y0, dy, by0, by1)):
                if abs(d) < 1e-9:
                    if not lo <= o <= hi:
                        tmin, tmax = 1.0, 0.0
                else:
                    t1, t2 = (lo - o) / d, (hi - o) / d
                    tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
            if tmin <= tmax and tmin > 0:
                best = min(best, tmin)
        if best > MAX_RANGE or best < MIN_RANGE or rng.random() < dropout:
            out.append(0)
        else:
            out.append(int(max(0.0, best + rng.gauss(0.0, noise_m)) * 1000))
    return out
