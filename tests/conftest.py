"""Tests must never talk to the real robot.

The dashboard's RobotController sends velocity over UDP to Config.PI_IP. Tests build real controllers (through
CVPipeline), so without this the test suite would drive a live TurtleBot. Everything runs in 'mock' mode, which
keeps the slew limiting and watchdog but transmits nothing. The one test that needs real UDP (the loopback link
test) switches itself to udp and points at 127.0.0.1.
"""

import os
import sys
from pathlib import Path

# A developer's config.local.json / HF_* variables must not change what the tests see.
os.environ["HF_NO_OVERRIDES"] = "1"

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT, ROOT / "pi", ROOT / "laptop"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from config import Config  # noqa: E402

Config.COMMAND_MODE = "mock"
# The behaviour suites predate hold-to-follow and drive the follow logic without gestures;
# tests/test_pipeline_states.py switches it on where it is the subject.
Config.FOLLOW_REQUIRES_RAISED_HAND = False
# The recall suites exercise the optional drive-to-last-seen recovery; the default (rotate only) is tested on its own.
Config.RECOVERY_DRIVE_TO_LAST_SEEN = True
# The older suites were written for a 1.2 m follow distance; the 2 m default and back-off have their own tests.
Config.DESIRED_DISTANCE = Config.TARGET_DISTANCE = 1.2
Config.PI_IP = "127.0.0.1"
