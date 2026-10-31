#!/usr/bin/env python3
"""Start the HIL bench: virtual Pi, then the laptop vision dashboard.

The real robot is not contacted. Put the LiPo on the charger and run:

    python hil/run_hil.py

or double-click run_hil.bat.

Then open:
    http://127.0.0.1:8000/   vision, gestures, teleop
    http://127.0.0.1:5001/   Burger pose map
"""

import os
import runpy
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _port_open(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.3)
    try:
        sock.connect(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        sock.close()


def _udp_in_use(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("0.0.0.0", port))
        return False
    except OSError:
        return True
    finally:
        sock.close()


def _wait_health(timeout=20.0):
    deadline = time.time() + timeout
    url = "http://127.0.0.1:5001/health"
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(url, timeout=0.5) as resp:
                if resp.status == 200:
                    return True
        except Exception:
            time.sleep(0.25)
    return False


def main():
    if _port_open(5001) or _udp_in_use(8888) or _udp_in_use(5000):
        print("TCP 5001 or UDP 8888/5000 is already in use.")
        print("Stop the other virtual Pi or dashboard first.")
        return 1

    os.environ["HIL"] = "1"
    os.environ["PI_IP"] = "127.0.0.1"

    proc = subprocess.Popen(
        [sys.executable, "-u", "-m", "hil.virtual_pi"],
        cwd=str(ROOT),
    )
    print("Starting virtual TurtleBot3 Burger (webcam + kinematic plant)...")
    try:
        if not _wait_health():
            print("Virtual Pi did not open port 5001. See the log above.")
            proc.terminate()
            return 1
        print("Virtual Pi is up. Starting the vision dashboard.")
        print("  Dashboard : http://127.0.0.1:8000/")
        print("  Pose map  : http://127.0.0.1:5001/")
        print("Stand in front of this laptop camera. The robot on the floor does not move.")
        sys.path.insert(0, str(ROOT / "laptop"))
        runpy.run_path(str(ROOT / "laptop" / "app.py"), run_name="__main__")
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
    return 0


if __name__ == "__main__":
    sys.exit(main())
