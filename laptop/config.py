"""
Configuration settings for Robust Owner-Locked Human-Following TurtleBot System.
Centralizes all perception, Re-ID, state-machine, control, safety, and network parameters.

Nothing here has to be edited in place. Every UPPER_CASE value can be overridden, in this order:
  1. config.local.json in the project root   {"PI_IP": "192.168.1.42", "CAM_HEIGHT_M": 0.21}
  2. environment variables HF_<NAME>          $env:HF_DESIRED_DISTANCE = "1.0"
  (PI_IP and VIDEO_PORT also accept their bare names, as before.)
The dashboard's settings panel writes the tunable subset (TUNABLE below) back to config.local.json.
"""

import json
import os

# Project root stays the folder that holds the pose model, templates, and static files.
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

class Config:
    # --- Network / Stream Settings ---
    # Override with PI_IP=127.0.0.1 when running the HIL bench (hil/run_hil.py).
    # The Pi hosts nothing. The laptop listens on VIDEO_PORT (UDP) for video and
    # sends commands plus a HELLO to PI_IP:UDP_PORT so the Pi knows where to reply.
    PI_IP = os.environ.get("PI_IP", "10.242.147.140")
    VIDEO_PORT = int(os.environ.get("VIDEO_PORT", "5000"))

    @classmethod
    def get_stream_url(cls):
        return f"udp://{cls.PI_IP}:{cls.UDP_PORT}"

    # Local web dashboard
    DASHBOARD_HOST = "0.0.0.0"
    DASHBOARD_PORT = 8000

    # Model asset path (kept at the project root, not inside this package)
    MODEL_PATH = os.path.join(PROJECT_ROOT, "pose_landmarker_lite.task")
    OWNER_PROFILE_PATH = os.path.join(PROJECT_ROOT, "owner_profile.pkl")

    # --- Processing Parameters ---
    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480
    TARGET_FPS = 25
    STALE_FRAME_TIMEOUT = 0.80  # seconds before declaring video stream stale

    # --- State Machine States ---
    STATE_IDLE = "IDLE"
    STATE_OWNER_ENROLLMENT = "OWNER_ENROLLMENT"
    STATE_OWNER_LOCKED = "OWNER_LOCKED"
    STATE_FOLLOWING = "FOLLOWING"
    STATE_WAIT_FOR_OWNER_MOVEMENT = "WAIT_FOR_OWNER_MOVEMENT"
    STATE_TARGET_TEMPORARILY_LOST = "TARGET_TEMPORARILY_LOST"
    STATE_SEARCHING = "SEARCH_ROTATING"
    STATE_TARGET_AMBIGUOUS = "TARGET_AMBIGUOUS"
    STATE_RECOVERING = "RECALLING_LAST_POSITION"
    STATE_SAFETY_STOP = "SAFETY_STOP"
    STATE_MANUAL_CONTROL = "MANUAL_CONTROL"
    STATE_OWNER_RELEASED = "OWNER_RELEASED"
    STATE_AWAITING_GESTURE = "AWAITING_FOLLOW_GESTURE"   # owner in view, hand down: stay, keep them centred
    STATE_LIDAR_GUIDED = "LIDAR_GUIDED_SEARCH"           # owner out of the camera, LiDAR still has them
    STATE_PAUSED = "PAUSED"                              # operator paused following (dashboard PAUSE / WAIT gesture)
    STATE_OWNER_LOST = "OWNER_LOST"                      # search exhausted (360 done or timed out): stationary
    STATE_SENSOR_FAULT = "SENSOR_FAULT"                  # display override: LiDAR silent or no camera frames
    RELEASED_SHOW_SEC = 2.0                              # OWNER_RELEASED is shown this long, then IDLE

    # --- Hold-to-follow ---
    # The robot drives only while the owner holds a hand above shoulder level. Hand down: it stays put and keeps
    # the owner centred (no search). Search only once the owner is lost from the camera, guided by the LiDAR.
    FOLLOW_REQUIRES_RAISED_HAND = True
    RAISE_CONFIRM_FRAMES = 3      # consecutive frames with the hand up before driving
    FOLLOW_RELEASE_SEC = 0.6      # hand down this long before stopping (detection flicker does not stutter)

    # --- Target Loss & Active Search ---
    SEARCH_ROTATION_SPEED = 0.60          # Slow, safe search angular velocity (rad/s)
    SEARCH_MAX_DURATION_SEC = 30.0        # Max seconds to search before timing out and stopping
    SEARCH_START_DELAY_SEC = 0.4          # Seconds after losing target before starting rotation

    # --- Following & Distance Hysteresis (Meters) ---
    DESIRED_DISTANCE = 2.00       # Desired following distance (also the minimum gap it keeps: see BACK_OFF_*)
    TARGET_DISTANCE = DESIRED_DISTANCE  # Compatibility alias
    STOP_DISTANCE = 1.10          # Enter WAIT_FOR_OWNER_MOVEMENT when d <= STOP_DISTANCE
    RESUME_DISTANCE = 1.35        # Resume FOLLOWING when d >= RESUME_DISTANCE (prevents chatter)
    RESUME_HYSTERESIS_FRAMES = 3  # Must exceed RESUME_DISTANCE for 3 consecutive frames
    MIN_CRITICAL_DISTANCE = 0.55  # Immediate safety stop boundary (forward blocked; backing away still allowed)
    # Back-off: someone walks up closer than the follow distance -> reverse to restore it, facing them.
    BACK_OFF_ENABLED = True
    BACK_OFF_MARGIN = 0.25        # start reversing below DESIRED - margin (1.75 m at 2 m) ...
    BACK_OFF_RELEASE = 0.05       # ... stop once back to DESIRED - release (hysteresis, no shuffling)
    BACK_OFF_GAIN = 0.6           # m/s per metre too close
    MAX_REVERSE_VEL = 0.15        # m/s; slower than forward because the camera cannot see behind
    REVERSE_CLEARANCE_M = 0.35    # LiDAR rear sector must be at least this clear to reverse at all
    LIDAR_REAR_STOP_M = 0.20      # controller hard stop for reverse, like LIDAR_HARD_STOP_M for forward

    # --- PID & Velocity Control Limits ---
    MAX_LINEAR_VEL = 0.22         # Physical limit for TurtleBot3 Burger (m/s)
    MIN_ROLLING_VEL = 0.08        # Minimum speed to overcome static carpet/tile wheel friction (m/s)
    MAX_ANGULAR_VEL = 2.50        # rad/s (Burger hardware limit 2.84)
    STEERING_DEADZONE = 0.05      # ±5% normalized horizontal deadzone
    ANGULAR_DEADZONE = STEERING_DEADZONE  # Compatibility alias
    
    # Slew-rate acceleration limits
    MAX_ACCEL_LINEAR = 1.60       # m/s^2
    MAX_ACCEL_ANGULAR = 12.0      # rad/s^2: snappy turns toward a person who steps sideways

    # --- Camera geometry and LiDAR fusion ---
    CAMERA_HFOV_DEG = 57.0        # horizontal field of view; focal length in px = (w/2)/tan(hfov/2)
    # Camera pose relative to the LiDAR centre (robot frame: x forward, y left, z up). Used to project the
    # LiDAR scan into the image (co-mapping) and to pick the LiDAR beams that fall inside a person's box.
    CAM_X_M = 0.04                # camera ahead of the LiDAR axis
    CAM_Y_M = 0.0
    CAM_HEIGHT_M = 0.20           # camera optical centre above the floor
    LIDAR_HEIGHT_M = 0.17         # LDS-02 scan plane above the floor (Burger top plate)
    CAM_YAW_DEG = 0.0             # + = camera looks to the left of the robot axis
    CAM_PITCH_DEG = 0.0           # + = camera tilted down
    # Sensor orientation fixes. Check: stand 1.5 m to the robot's RIGHT. In the live view you must appear on the
    # RIGHT of the image (else CAM_MIRROR), and in the LiDAR map to the right of the robot icon (else LIDAR_MIRROR).
    CAM_MIRROR = False            # camera delivers a selfie-style (horizontally flipped) image
    LIDAR_MIRROR = False          # scan is clockwise instead of ROS counter-clockwise
    LIDAR_YAW_DEG = 0.0           # LiDAR zero direction is rotated this much (+ = to the left) from robot forward

    # Monocular range: real body sizes (m). Range = f_px * size / pixels * DISTANCE_SCALE.
    # With the 57 deg HFOV these give the same ranges as the earlier px*m constants (235, 160, 230, 320 at 640 px).
    PERSON_SHOULDER_M = 0.40
    PERSON_HIP_M = 0.27
    PERSON_TORSO_M = 0.39
    PERSON_BOX_M = 0.54           # fallback: landmark bounding-box height
    DISTANCE_SCALE = 1.0          # dashboard calibration; the LiDAR also corrects it online (vision_scale)
    MAX_POSES = 3                 # people MediaPipe returns per frame; >1 is what makes bystander rejection real
    LIDAR_FUSION = True
    LIDAR_HARD_STOP_M = 0.20      # anything this close dead ahead stops forward motion, whatever the vision says
    LIDAR_STALE_SEC = 0.6         # older scans are ignored and the vision corridors take over
    LIDAR_AGREE_FRAC = 0.40       # LiDAR and camera ranges must agree within this fraction to fuse
    # Identity: a stranger must never inherit the lock. After any loss the owner is only
    # re-accepted if the appearance score clears this for several consecutive frames.
    REACQ_THRESHOLD = 0.40
    REACQ_RELATIVE = 0.60         # ...and 60% of what the owner normally scores while tracked
    REACQ_CONFIRM_FRAMES = 5
    TRACK_RELATIVE = 0.50         # while the track is alive: position continuity is the main evidence
    LIDAR_TRACK_MAX_SEC = 4.0     # trust a LiDAR-only track this long without the camera confirming it (a chair leg
                                  # near the prediction looks like a person's leg; a long leash drives straight at it)
    LIDAR_TRACK_GATE_M = 0.50     # LiDAR person-cluster must be this close to where the EKF expects the owner
    MEMORY_PURSUIT_SEC = 25.0     # drive to where the owner was last seen (around obstacles) for up to this long
    MEMORY_SPEED = 0.15           # m/s while recalling, slower than following because nobody is in view
    MEMORY_PLAN_GRACE_SEC = 1.0   # wait this long for the first plan before giving up on the map and searching
    SEARCH_SWEEP_SEC = 6.0        # sweep phase length before the full-circle scan starts
    SEARCH_SCAN_SPEED = 0.80      # rad/s during the full-circle scan
    SEARCH_MEMORY_SEC = 30.0      # how long the EKF keeps coasting the owner while searching
    MIN_FOLLOW_DISTANCE = 0.60    # the dashboard cannot set the follow distance below this
    EKF_LEAD_SEC = 0.45           # steer toward where the person will be, not where they were

    # --- PID follow controller (tuned in tests/test_follow_sim.py) ---
    PID_DIST_KP = 0.45
    PID_DIST_KI = 0.04
    PID_DIST_KD = 0.20
    PID_TURN_KP = 3.00
    PID_TURN_KI = 0.05
    PID_TURN_KD = 0.35
    PID_FF_GAIN = 0.80            # feed-forward on the person's speed along the line of sight
    PID_TURN_FF = 0.80            # feed-forward on the person's own angular rate about the robot (turns with them)
    FACE_TURN_GAIN = 2.0          # stationary (waiting): turn to keep the owner centred
    FACE_TURN_MAX = 1.5
    # Owner just lost: turn at once toward where the EKF (LiDAR + camera motion) says they went, and if they were
    # walking and no map plan exists, drive toward their extrapolated position before falling back to the search.
    PREDICT_TURN_GAIN = 2.0
    PREDICT_TURN_MAX = 1.5
    PREDICT_MIN_SPEED = 0.15      # m/s: below this the owner was standing, so there is no direction to follow
    PREDICT_PURSUIT_SEC = 8.0
    # Lost-owner recovery (Do Hoang et al., URAI 2017): rotate toward where they went, sweep, full 360, then stop.
    # Driving to the last-seen / predicted position is optional, off by default: rotating finds a person who stepped
    # aside, driving blind toward a remembered LiDAR point does not.
    RECOVERY_DRIVE_TO_LAST_SEEN = False

    # --- Dynamic following distance (grows with the owner's speed and with tracking uncertainty) ---
    DIST_SPEED_GAIN = 0.5         # s of time headway: +0.5 m at 1 m/s
    DIST_CONF_GAIN = 0.35         # m added at the lowest tracking confidence
    DIST_MAX_EXTRA = 0.6          # never more than this beyond the operator's set distance
    # --- Sensor health ---
    LIDAR_FAULT_SEC = 1.5         # LiDAR was streaming and has gone silent this long: stop (sensor abnormality)
    PID_MIN_V = 0.06
    STEERING_DEADZONE_DEG = 2.0

    # --- Owner Re-Identification & Identity Protection ---
    IDENTITY_MATCH_THRESHOLD = 0.40        # Minimum score to accept candidate as Owner when bystanders present
    IDENTITY_SINGLE_PERSON_THRESHOLD = 0.28 # Minimum score when only 1 candidate exists in FOV
    IDENTITY_AMBIGUITY_DELTA = 0.08        # Trigger TARGET_AMBIGUOUS if top two candidates differ by less than this
    MAX_OWNER_TEMPLATES = 8                # Multi-view template bank size
    TEMPLATE_UPDATE_MIN_CONF = 0.45        # Only add new template when identity match is very high
    TEMPLATE_DRIFT_ALPHA = 0.05            # Maximum EMA update rate to prevent template corruption

    # --- Gesture Recognition Parameters ---
    GESTURE_CONFIRM_FRAMES = 4             # Multi-frame debounce duration
    GESTURE_COOLDOWN_SEC = 1.5             # Cooldown after accepting gesture

    # --- Recovery & Timeouts ---
    OCCLUSION_RECOVERY_SEC = 3.0           # Project motion for up to 3.0s during occlusion
    COMMAND_WATCHDOG_TIMEOUT = 0.60        # Seconds before zeroing velocity commands

    # --- Hardware Bridge Settings ---
    COMMAND_MODE = "udp"                   # 'udp' or 'mock'
    UDP_PORT = 8888
    COMMAND_HZ = 30
    TELEMETRY_HZ = 20

    # --- SLAM (laptop occupancy grid + multi-resolution correlative scan matching) ---
    SLAM_SEARCH_XY_M = 0.30       # translation search window around the motion prior
    SLAM_SEARCH_YAW_DEG = 15.0    # rotation search window
    SLAM_SIGMA_M = 0.06           # likelihood-field width: how far a return may sit from a wall and still count
    SLAM_MIN_SCORE = 0.35         # below this the match is ignored and odometry (or the prior) is kept
    SLAM_RECENTER_M = 2.5         # shift the 12 m grid once the robot is this close to its edge
    SLAM_USE_EXTERNAL = True      # trust a map-frame pose from slam_toolbox/Cartographer when the Pi sends one

    # Settings the dashboard may change live (and save to config.local.json).
    TUNABLE = ("DESIRED_DISTANCE", "DISTANCE_SCALE", "CAMERA_HFOV_DEG", "CAM_X_M", "CAM_Y_M", "CAM_HEIGHT_M",
               "LIDAR_HEIGHT_M", "CAM_YAW_DEG", "CAM_PITCH_DEG", "CAM_MIRROR", "LIDAR_MIRROR", "LIDAR_YAW_DEG", "MAX_LINEAR_VEL", "MAX_ANGULAR_VEL",
               "PID_DIST_KP", "PID_DIST_KI", "PID_DIST_KD", "PID_TURN_KP", "PID_TURN_KI", "PID_TURN_KD",
               "FOLLOW_REQUIRES_RAISED_HAND", "RECOVERY_DRIVE_TO_LAST_SEEN", "BACK_OFF_ENABLED", "MAX_REVERSE_VEL", "DIST_SPEED_GAIN", "DIST_CONF_GAIN",
               "PID_FF_GAIN", "PID_TURN_FF", "MAX_ACCEL_ANGULAR", "IDENTITY_MATCH_THRESHOLD", "REACQ_THRESHOLD", "PI_IP")

    @classmethod
    def coerce(cls, name, value):
        """Convert a JSON/env value to the type of the existing setting. Raises ValueError/KeyError."""
        old = getattr(cls, name)
        if isinstance(old, bool):
            if isinstance(value, str):
                return value.strip().lower() in ("1", "true", "yes", "on")
            return bool(value)
        if isinstance(old, int):
            return int(float(value))
        if isinstance(old, float):
            v = float(value)
            if v != v or v in (float("inf"), float("-inf")):
                raise ValueError(f"{name}: not a finite number")
            return v
        if isinstance(old, str):
            return str(value).strip()
        raise KeyError(name)

    @classmethod
    def set(cls, name, value):
        name = name.upper()
        if not name.isupper() or name.startswith("STATE_") or not hasattr(cls, name) or name == "TUNABLE":
            raise KeyError(name)
        setattr(cls, name, cls.coerce(name, value))
        if name == "DESIRED_DISTANCE":
            cls.TARGET_DISTANCE = cls.DESIRED_DISTANCE
        return getattr(cls, name)

    @classmethod
    def tunables(cls):
        return {k: getattr(cls, k) for k in cls.TUNABLE}

    @classmethod
    def save_local(cls, updates, path=None):
        """Merge updates into config.local.json so the next start uses them."""
        path = path or LOCAL_CONFIG_PATH
        data = {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError):
            data = {}
        data.update(updates)
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, sort_keys=True)
        os.replace(tmp, path)


LOCAL_CONFIG_PATH = os.environ.get("HF_CONFIG", os.path.join(PROJECT_ROOT, "config.local.json"))


def apply_overrides(env=None, path=None):
    """config.local.json first, then HF_<NAME> environment variables. Bad entries are reported, never fatal."""
    env = os.environ if env is None else env
    applied = {}
    path = path or LOCAL_CONFIG_PATH
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                for k, v in json.load(f).items():
                    try:
                        applied[k.upper()] = Config.set(k, v)
                    except (KeyError, ValueError, TypeError) as e:
                        print(f"[Config] ignoring {k!r} in {path}: {e}")
        except (OSError, ValueError) as e:
            print(f"[Config] cannot read {path}: {e}")
    for key, raw in env.items():
        if key.startswith("HF_") and key != "HF_CONFIG":
            try:
                applied[key[3:]] = Config.set(key[3:], raw)
            except (KeyError, ValueError, TypeError) as e:
                print(f"[Config] ignoring {key}: {e}")
    if "PI_IP" in env:                # the long-standing bare names still win
        Config.PI_IP = env["PI_IP"].strip()
    if "VIDEO_PORT" in env:
        Config.VIDEO_PORT = int(env["VIDEO_PORT"])
    return applied


if os.environ.get("HF_NO_OVERRIDES") != "1":
    apply_overrides()
