"""Hardware movement check. Not a unit test: it drives the real robot a few centimetres.

    python tests/hw_move_check.py [PI_IP]

Sends the same UDP packets as the dashboard and reads odometry and the LiDAR back from the Pi.
Each move is aborted if the LiDAR shows anything closer than 0.6 m in the direction of travel.
Needs the dashboard stopped (it uses UDP 5000).
"""

import math
import socket
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pi"))
sys.path.insert(0, str(ROOT / "laptop"))

import numpy as np  # noqa: E402
import tb_proto  # noqa: E402
import lidar  # noqa: E402

PI = sys.argv[1] if len(sys.argv) > 1 else "10.252.159.140"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", tb_proto.VIDEO_PORT))
sock.settimeout(0.02)
asm = tb_proto.Reassembler()
seq = int(time.time() * 100) & 0xFFFFFFFF
state = {"pose": None, "bins": None, "status": {}}


def pump():
    """Drain incoming packets; keep the newest scan, pose and Pi status."""
    try:
        while True:
            data, _ = sock.recvfrom(2048)
            r = asm.feed(data)
            if r and r[0] == "scan":
                state["bins"], state["pose"] = r[1]["bins"], r[1]["pose"]
            elif r and r[0] == "status":
                state["status"] = r[1]
    except socket.timeout:
        pass


_last_hello = [0.0]


def send(v, w):
    global seq
    seq += 1
    if time.time() - _last_hello[0] > 0.5:  # the Pi only streams scans and status to a peer that keeps saying HELLO
        _last_hello[0] = time.time()
        sock.sendto(tb_proto.HELLO, (PI, tb_proto.CMD_PORT))
    sock.sendto(tb_proto.encode_command(v, w, seq), (PI, tb_proto.CMD_PORT))


def sector_min(lo, hi):
    r = lidar.to_meters(state["bins"])
    deg = np.degrees(lidar.bin_angles(len(r)))
    m = (deg >= lo) & (deg <= hi) & np.isfinite(r)
    return float(r[m].min()) if m.any() else 9.9


def front():
    return sector_min(-12, 12)


def rear():
    r = lidar.to_meters(state["bins"])
    deg = np.abs(np.degrees(lidar.bin_angles(len(r))))
    m = (deg >= 168) & np.isfinite(r)
    return float(r[m].min()) if m.any() else 9.9


def wait_fresh(n=3, timeout=8.0):
    end = time.time() + timeout
    got = 0
    last = None
    while time.time() < end and got < n:
        sock.sendto(tb_proto.HELLO, (PI, tb_proto.CMD_PORT))
        send(0.0, 0.0)
        pump()
        if state["pose"] is not None and state["pose"] != last:
            last = state["pose"]
            got += 1
        time.sleep(0.1)
    return got >= n


def hold(v, w, seconds, guard):
    """Send a command at 30 Hz. Returns False if the guard tripped."""
    end = time.time() + seconds
    while time.time() < end:
        pump()
        if guard is not None and guard() < 0.6:
            send(0.0, 0.0)
            print("  ABORT: obstacle within 0.6 m")
            return False
        send(v, w)
        time.sleep(1 / 30)
    return True


def stop(seconds=1.0):
    end = time.time() + seconds
    while time.time() < end:
        pump()
        send(0.0, 0.0)
        time.sleep(1 / 30)


def pose():
    pump()
    return state["pose"]


def run(name, v, w, seconds, guard, expect):
    p0, f0 = pose(), front()
    print(f"\n[{name}] v={v:+.2f} m/s  w={w:+.2f} rad/s  for {seconds:.1f} s")
    ok = hold(v, w, seconds, guard)
    stop(1.2)
    p1, f1 = pose(), front()
    dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) * (1 if v >= 0 else -1)
    dyaw = math.degrees(math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2])))
    print(f"  odometry: moved {dist:+.3f} m, turned {dyaw:+.1f} deg   (expected about {expect})")
    if abs(v) > 0:
        print(f"  LiDAR front range: {f0:.3f} -> {f1:.3f} m  (change {f0 - f1:+.3f} m)")
    return ok, dist, dyaw, f0 - f1


if not wait_fresh():
    print("No LiDAR/odometry from the Pi. Is the node running and is the dashboard stopped?")
    sys.exit(1)
st = state["status"]
print(f"Pi status: ros={st.get('ros')} motors={st.get('motors')} /cmd_vel subscribers={st.get('subs')} scan={st.get('scan_hz')} Hz")
print(f"Clear space: front {front():.2f} m, rear {rear():.2f} m")
if not st.get("motors") or not st.get("subs"):
    print("Motors are not enabled or OpenCR is not subscribed. Aborting.")
    sys.exit(2)

results = {}
if front() > 1.0:
    results["fwd"] = run("forward", 0.10, 0.0, 2.0, front, "about 0.18 m after the ramp")
else:
    print("\n[forward] skipped: less than 1.0 m clear ahead")
if rear() > 1.0 and results.get("fwd", (False,))[0]:
    results["back"] = run("backward", -0.10, 0.0, 2.0, rear, "about -0.18 m")
else:
    print("\n[backward] skipped: need 1.0 m clear behind and a completed forward move")
results["left"] = run("spin left", 0.0, 0.5, 2.0, None, "about +50 deg")
results["right"] = run("spin right", 0.0, -0.5, 2.0, None, "about -50 deg")

print("\n[watchdog] send forward for 0.6 s then go silent; the robot must stop by itself")
if front() > 1.0:
    p0 = pose()
    hold(0.10, 0.0, 0.6, front)
    t_silence = time.time()
    time.sleep(0.4)
    pump()
    pa = pose()
    time.sleep(1.2)
    pump()
    pb = pose()
    creep = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
    print(f"  after 0.4 s of silence: moved {math.hypot(pa[0]-p0[0], pa[1]-p0[1]):.3f} m; "
          f"in the following 1.2 s it moved {creep:.3f} m  -> {'STOPPED' if creep < 0.01 else 'STILL MOVING'}")
else:
    print("  skipped: not enough room")

stop(1.0)
if "left" in results and "right" in results:
    l, r = results["left"][2], results["right"][2]
    print(f"\nSpin symmetry: left {l:+.1f} deg, right {r:+.1f} deg, net {l + r:+.1f} deg")
if "fwd" in results and "back" in results:
    print(f"Out-and-back net odometry: {results['fwd'][1] + results['back'][1]:+.3f} m")
print("done")
