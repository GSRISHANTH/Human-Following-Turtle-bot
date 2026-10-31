"""Top-down drawing of the Burger plant. x is forward, y is left."""

import math
import cv2
import numpy as np

from hil.plant import WHEEL_SEPARATION_M


def render_map(snapshot, size=640):
    img = np.full((size, size, 3), (22, 26, 32), dtype=np.uint8)
    ppm = 90.0
    cx = cy = size // 2

    def to_px(x, y):
        # Screen up = +x (forward), screen left = +y.
        px = int(cx - y * ppm)
        py = int(cy - x * ppm)
        return px, py

    for meter in range(-4, 5):
        x0, y0 = to_px(meter, -4)
        x1, y1 = to_px(meter, 4)
        cv2.line(img, (x0, y0), (x1, y1), (42, 48, 56), 1)
        x0, y0 = to_px(-4, meter)
        x1, y1 = to_px(4, meter)
        cv2.line(img, (x0, y0), (x1, y1), (42, 48, 56), 1)
    ox, oy = to_px(0, 0)
    cv2.drawMarker(img, (ox, oy), (90, 140, 160), cv2.MARKER_CROSS, 16, 1)

    trail = snapshot["trail"]
    if len(trail) >= 2:
        pts = np.array([to_px(x, y) for x, y in trail], dtype=np.int32)
        cv2.polylines(img, [pts], False, (0, 180, 160), 2)

    x = snapshot["x"]
    y = snapshot["y"]
    th = snapshot["theta"]
    # Burger body is about 138 mm wide and 178 mm long.
    length, width = 0.178, 0.138
    corners = [(-length / 2, -width / 2), (length / 2, -width / 2),
               (length / 2, width / 2), (-length / 2, width / 2)]
    body = []
    c, s = math.cos(th), math.sin(th)
    for lx, ly in corners:
        wx = x + c * lx - s * ly
        wy = y + s * lx + c * ly
        body.append(to_px(wx, wy))
    cv2.fillPoly(img, [np.array(body, dtype=np.int32)], (0, 120, 180))

    half = WHEEL_SEPARATION_M / 2.0
    for side in (-half, half):
        wx = x + c * 0.0 - s * side
        wy = y + s * 0.0 + c * side
        cv2.circle(img, to_px(wx, wy), 5, (230, 230, 230), -1)

    nose = to_px(x + c * 0.16, y + s * 0.16)
    cv2.arrowedLine(img, to_px(x, y), nose, (80, 255, 180), 2, tipLength=0.35)

    hud = (
        f"x={snapshot['x']:+.2f} m   y={snapshot['y']:+.2f} m   "
        f"yaw={math.degrees(snapshot['theta']):+.0f} deg"
    )
    cmd = (
        f"cmd  v={snapshot['goal_linear']:+.2f} m/s   "
        f"w={snapshot['goal_angular']:+.2f} rad/s"
    )
    cv2.putText(img, "HIL  TurtleBot3 Burger plant", (16, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 230, 235), 1, cv2.LINE_AA)
    cv2.putText(img, hud, (16, size - 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 220, 200), 1, cv2.LINE_AA)
    cv2.putText(img, cmd, (16, size - 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 220, 200), 1, cv2.LINE_AA)
    return img
