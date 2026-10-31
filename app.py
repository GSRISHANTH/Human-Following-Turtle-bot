"""Launch the laptop vision dashboard. Code lives in laptop/app.py."""

import runpy
import sys
from pathlib import Path

if __name__ == "__main__":
    laptop = Path(__file__).resolve().parent / "laptop"
    sys.path.insert(0, str(laptop))
    runpy.run_path(str(laptop / "app.py"), run_name="__main__")
