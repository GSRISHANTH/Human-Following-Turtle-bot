"""Tune the follower on the real robot against a virtual person.

    python tests/hw_follow_virtual.py [PI_IP] [--gains name]

The person is simulated (it walks straight, then turns 60 degrees left, then stops) but everything else is
real: the robot's wheels and odometry, the UDP link and its latency, and the project's own PersonEKF and
FollowController. Noisy, delayed range/bearing measurements are generated from the robot's true odometry pose.
Needs about 2.5 m clear ahead and the dashboard stopped. Aborts if the LiDAR sees anything within 0.55 m ahead.
"""

import math
import random
import socket
import sys
import time
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pi"))
sys.path.insert(0, str(ROOT / "laptop"))

import numpy as np  # noqa: E402
import tb_proto  # noqa: E402
import lidar  # noqa: E402
from config import Config  # noqa: E402
from ekf_person import PersonEKF  # noqa: E402
from follow_controller import FollowController  # noqa: E402

PI = "10.252.159.140"
args = [a for a in sys.argv[1:] if not a.startswith("--")]
if args:
    PI = args[0]

GAIN_SETS = {
    "current": {},
    "snappier-turn": {"PID_TURN_KP": 2.8, "PID_TURN_KD": 0.45},
    "softer-turn": {"PID_TURN_KP": 1.6, "PID_TURN_KD": 0.25},
    "firmer-distance": {"PID_DIST_KP": 0.9, "PID_DIST_KD": 0.35},
    "gentle-distance": {"PID_DIST_KP": 0.45, "PID_DIST_KD": 0.2},
}

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", tb_proto.VIDEO_PORT))
sock.settimeout(0.005)
asm = tb_proto.Reassembler()
seq = int(time.time() * 100) & 0xFFFFFFFF
state = {"pose": None, "bins": None}
_hello = [0.0]


def pump():
    try:
        while True:
            data, _ = sock.recvfrom(2048)
            r = asm.feed(data)
            if r and r[0] == "scan":
                state["bins"], state["pose"] = r[1]["bins"], r[1]["pose"]
    except socket.timeout:
        pass


def send(v, w):
    global seq
    seq += 1
    if time.time() - _hello[0] > 0.5:
        _hello[0] = time.time()
        sock.sendto(tb_proto.HELLO, (PI, tb_proto.CMD_PORT))
    sock.sendto(tb_proto.encode_command(v, w, seq), (PI, tb_proto.CMD_PORT))


def front_raw():
    r = lidar.to_meters(state["bins"])
    deg = np.degrees(lidar.bin_angles(len(r)))
    m = (np.abs(deg) <= 15) & np.isfinite(r)
    return float(r[m].min()) if m.any() else 9.9


def side_clearance(deg_center):
    r = lidar.to_meters(state["bins"])
    deg = np.degrees(lidar.bin_angles(len(r)))
    m = (np.abs(deg - deg_center) <= 25) & np.isfinite(r)
    return float(r[m].min()) if m.any() else 9.9


def choose_turn():
    """Turn the virtual person toward the roomier side, by as much as the room allows."""
    left, right = side_clearance(55), side_clearance(-55)
    sign = 1.0 if left >= right else -1.0
    room = max(left, right)
    amount = 60.0 if room > 2.2 else 35.0 if room > 1.6 else 0.0
    return sign, amount, left, right


def settle(seconds):
    end = time.time() + seconds
    while time.time() < end:
        pump()
        send(0.0, 0.0)
        time.sleep(1 / 30)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def run_once(name, overrides, seconds=15.0, speed=0.12, start_range=1.7):
    saved = {k: getattr(Config, k) for k in overrides}
    for k, v in overrides.items():
        setattr(Config, k, v)
    rnd = random.Random(7)
    pump()
    turn_sign, turn_deg, cl_l, cl_r = choose_turn()
    print(f"  clearance at +/-55 deg: left {cl_l:.1f} m, right {cl_r:.1f} m -> virtual person turns {'left' if turn_sign > 0 else 'right'} {turn_deg:.0f} deg")
    x0, y0, yaw0 = state["pose"]
    px, py = x0 + start_range * math.cos(yaw0), y0 + start_range * math.sin(yaw0)
    ekf, ctl = PersonEKF(), FollowController()
    cam_q, lid_q = deque(), deque()
    v = w = 0.0
    rows = []
    t0 = time.time()
    last = t0
    next_cam = next_lid = 0.0
    aborted = False
    while True:
        now = time.time()
        t = now - t0
        if t > seconds:
            break
        pump()
        if front_raw() < 0.55:
            aborted = True
            print(f"  LiDAR sees {front_raw():.2f} m ahead at t={t:.1f} s (robot pose {state['pose'][0]-x0:+.2f}, {state['pose'][1]-y0:+.2f} m from start)")
            break
        dt = now - last
        last = now
        # virtual person: straight, then a 60 degree turn to the left over 4 s, then stands still
        if t < 5:
            heading, sp = yaw0, speed
        elif t < 9:
            heading, sp = yaw0 + turn_sign * math.radians(turn_deg) * (t - 5) / 4.0, speed
        else:
            heading, sp = yaw0 + turn_sign * math.radians(turn_deg), 0.0
        px += sp * math.cos(heading) * dt
        py += sp * math.sin(heading) * dt
        rx, ry, ryaw = state["pose"]
        dx, dy = px - rx, py - ry
        c, s = math.cos(-ryaw), math.sin(-ryaw)
        lx, ly = c * dx - s * dy, s * dx + c * dy
        r_true, b_true = math.hypot(lx, ly), math.atan2(ly, lx)
        if t >= next_cam:
            next_cam = t + 1 / 13.0
            cam_q.append((now + 0.15, r_true + rnd.gauss(0, 0.12 * r_true), b_true + rnd.gauss(0, math.radians(1.5)), "cam"))
        if t >= next_lid:
            next_lid = t + 1 / 9.0
            lid_q.append((now + 0.10, r_true + rnd.gauss(0, 0.05), b_true + rnd.gauss(0, math.radians(1.0)), "lidar"))
        for q in (cam_q, lid_q):
            while q and q[0][0] <= now:
                _, rm, bm, kind = q.popleft()
                if not ekf.active:
                    ekf.init(rm, bm, now)
                ekf.predict(now, v, w)
                ekf.update(rm, bm, 0.05 if kind == "lidar" else 0.12 * rm, math.radians(1.0 if kind == "lidar" else 1.5))
        if ekf.active:
            ekf.predict(now, v, w)
            r_e, b_e = ekf.range_bearing()
            lx_, ly_ = ekf.predicted(Config.EKF_LEAD_SEC)
            b_ctl = math.atan2(ly_, lx_) if lx_ > 0.1 else b_e
            los = (ekf.s[0] * ekf.s[2] + ekf.s[1] * ekf.s[3]) / max(r_e, 0.1)
            tv, tw = ctl.command(r_e, b_ctl, ekf.range_rate(), ekf.bearing_rate(w), los, max(dt, 1e-3), desired=Config.DESIRED_DISTANCE)
            if r_e <= Config.DESIRED_DISTANCE - 0.10:
                tv = 0.0
            tv = max(-0.0, min(Config.MAX_LINEAR_VEL, tv))
            # the same slew limiter the dashboard's RobotController applies
            v += max(-Config.MAX_ACCEL_LINEAR * dt, min(Config.MAX_ACCEL_LINEAR * dt, tv - v))
            w += max(-Config.MAX_ACCEL_ANGULAR * dt, min(Config.MAX_ACCEL_ANGULAR * dt, tw - w))
        send(v, w)
        rows.append((t, r_true, math.degrees(b_true), v, w))
        time.sleep(max(0.0, 1 / 30 - (time.time() - now)))
    for k, vv in saved.items():
        setattr(Config, k, vv)
    settle(1.2)
    if aborted or len(rows) < 50:
        return {"name": name, "aborted": True}
    walking = [r for r in rows if 3.0 <= r[0] <= 9.0]
    end = [r for r in rows if r[0] >= 12.0]
    rms_r = math.sqrt(sum((r[1] - Config.DESIRED_DISTANCE) ** 2 for r in walking) / len(walking))
    rms_b = math.sqrt(sum(r[2] ** 2 for r in walking) / len(walking))
    jerk_w = sum(abs(rows[i][4] - rows[i - 1][4]) for i in range(1, len(rows))) / len(rows)
    return {"name": name, "aborted": False, "rms_range": rms_r, "rms_bearing": rms_b,
            "max_bearing": max(abs(r[2]) for r in rows), "end_range": sum(r[1] for r in end) / len(end),
            "end_bearing": sum(abs(r[2]) for r in end) / len(end), "max_v": max(r[3] for r in rows),
            "max_w": max(abs(r[4]) for r in rows), "w_activity": jerk_w, "samples": len(rows)}


def go_home(x0, y0, yaw0):
    """Back to where the run started, so every gain set sees the same room."""
    end = time.time() + 20
    while time.time() < end:
        pump()
        x, y, yaw = state["pose"]
        d = (x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)   # signed distance along the start heading
        e_yaw = wrap(yaw0 - yaw)
        if abs(d) < 0.04 and abs(e_yaw) < math.radians(4):
            break
        vv = max(-0.10, min(0.10, -1.0 * d)) if abs(d) > 0.04 else 0.0
        if vv < 0 and lidar_rear() < 0.45:
            vv = 0.0
        ww = max(-0.6, min(0.6, 1.5 * e_yaw))
        send(vv, ww)
        time.sleep(1 / 30)
    settle(0.8)


def lidar_rear():
    r = lidar.to_meters(state["bins"])
    deg = np.abs(np.degrees(lidar.bin_angles(len(r))))
    m = (deg >= 165) & np.isfinite(r)
    return float(r[m].min()) if m.any() else 9.9


def wait_ready():
    end = time.time() + 8
    while time.time() < end:
        send(0.0, 0.0)
        pump()
        if state["pose"] is not None and state["bins"] is not None:
            return True
        time.sleep(0.1)
    return False


if not wait_ready():
    print("No odometry/LiDAR from the Pi.")
    sys.exit(1)
wanted = [a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--gains=")]
names = wanted[0].split(",") if wanted else list(GAIN_SETS)
print(f"front clear {front_raw():.2f} m, rear {lidar_rear():.2f} m. Virtual person: 1.7 m ahead, walks 0.12 m/s, turns 60 deg left, stops.")
home = state["pose"]
results = []
for name in names:
    if front_raw() < 2.0:
        print(f"[{name}] skipped: need 2.0 m clear ahead")
        continue
    print(f"\n[{name}] running {GAIN_SETS[name] or 'current gains'} ...", flush=True)
    res = run_once(name, GAIN_SETS[name])
    results.append(res)
    if res["aborted"]:
        print("  aborted (obstacle ahead)")
    else:
        print(f"  range rms {res['rms_range']:.3f} m | bearing rms {res['rms_bearing']:.1f} deg, max {res['max_bearing']:.1f} | "
              f"end range {res['end_range']:.2f} m, end bearing {res['end_bearing']:.1f} deg | max v {res['max_v']:.2f} m/s, max w {res['max_w']:.2f} rad/s | "
              f"w activity {res['w_activity']:.3f}")
    go_home(*home)

ok = [r for r in results if not r["aborted"]]
if ok:
    def score(r):
        return r["rms_range"] + 0.02 * r["rms_bearing"] + 0.5 * r["w_activity"] + 2.0 * abs(r["end_range"] - Config.DESIRED_DISTANCE)
    best = min(ok, key=score)
    print("\nscores (lower is better):", ", ".join(f"{r['name']}={score(r):.3f}" for r in ok))
    print("best:", best["name"], GAIN_SETS[best["name"]])
settle(1.0)
