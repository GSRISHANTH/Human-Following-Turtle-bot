"""Tests must never talk to the real robot.

The dashboard's RobotController sends velocity over UDP to Config.PI_IP. Tests build real controllers (through
CVPipeline), so without this the test suite would drive a live TurtleBot. Everything runs in 'mock' mode, which
keeps the slew limiting and watchdog but transmits nothing. The one test that needs real UDP (the loopback link
test) switches itself to udp and points at 127.0.0.1.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT, ROOT / "pi", ROOT / "laptop"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from config import Config  # noqa: E402

Config.COMMAND_MODE = "mock"
Config.PI_IP = "127.0.0.1"
