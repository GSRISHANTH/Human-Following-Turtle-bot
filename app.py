"""Launch the laptop vision dashboard. Code lives in laptop/app.py.

The dashboard needs mediapipe, OpenCV and Flask. If the Python that started this script lacks any of them
(for example `python app.py` picked a newer Python), this re-launches itself with one that has them.
"""

import os
import re
import runpy
import shutil
import subprocess
import sys
from pathlib import Path

NEEDED = "import mediapipe, cv2, flask, numpy"


def _has_deps(python):
    try:
        return subprocess.run([python, "-c", NEEDED], capture_output=True, timeout=60).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def _other_pythons():
    """Interpreters the Windows `py` launcher knows about, newest first."""
    found = []
    py = shutil.which("py")
    if py:
        try:
            out = subprocess.run([py, "-0p"], capture_output=True, text=True, timeout=20).stdout
        except (OSError, subprocess.SubprocessError):
            out = ""
        for line in out.splitlines():
            m = re.search(r"([A-Za-z]:\\.*python[w]?\.exe)\s*$", line.strip(), re.I)
            if m:
                found.append(m.group(1))
    for name in ("python3", "python"):
        p = shutil.which(name)
        if p:
            found.append(p)
    seen, result = set(), []
    for p in found:
        key = os.path.normcase(os.path.abspath(p))
        if key not in seen and key != os.path.normcase(os.path.abspath(sys.executable)):
            seen.add(key)
            result.append(p)
    return result


def main():
    try:
        exec(NEEDED)
    except ImportError as exc:
        print(f"[launcher] {sys.executable} cannot run the dashboard: {exc}")
        for candidate in _other_pythons():
            if _has_deps(candidate):
                print(f"[launcher] Restarting with {candidate}")
                sys.exit(subprocess.call([candidate, str(Path(__file__).resolve())] + sys.argv[1:]))
        print("[launcher] No installed Python has mediapipe, opencv-python, flask and numpy.\n"
              "           Run:  py -3.11 -m pip install mediapipe opencv-python flask numpy\n"
              "           then: py -3.11 app.py")
        sys.exit(1)
    laptop = Path(__file__).resolve().parent / "laptop"
    sys.path.insert(0, str(laptop))
    runpy.run_path(str(laptop / "app.py"), run_name="__main__")


if __name__ == "__main__":
    main()
