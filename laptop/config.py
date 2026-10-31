"""
Configuration settings for Robust Owner-Locked Human-Following TurtleBot System.
Centralizes all perception, Re-ID, state-machine, control, safety, and network parameters.
"""

import os

# Project root stays the folder that holds the pose model, templates, and static files.
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

class Config:
    # --- Network / Stream Settings ---
    # Override with PI_IP=127.0.0.1 when running the HIL bench (hil/run_hil.py).
    # The Pi hosts nothing. The laptop listens on VIDEO_PORT (UDP) for video and
    # sends commands plus a HELLO to PI_IP:UDP_PORT so the Pi knows where to reply.
    PI_IP = os.environ.get("PI_IP", "10.252.159.140")
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

    # --- Target Loss & Active Search ---
    SEARCH_ROTATION_SPEED = 0.60          # Slow, safe search angular velocity (rad/s)
    SEARCH_MAX_DURATION_SEC = 30.0        # Max seconds to search before timing out and stopping
    SEARCH_START_DELAY_SEC = 0.4          # Seconds after losing target before starting rotation

    # --- Following & Distance Hysteresis (Meters) ---
    DESIRED_DISTANCE = 1.20       # Desired following distance
    TARGET_DISTANCE = DESIRED_DISTANCE  # Compatibility alias
    STOP_DISTANCE = 1.10          # Enter WAIT_FOR_OWNER_MOVEMENT when d <= STOP_DISTANCE
    RESUME_DISTANCE = 1.35        # Resume FOLLOWING when d >= RESUME_DISTANCE (prevents chatter)
    RESUME_HYSTERESIS_FRAMES = 3  # Must exceed RESUME_DISTANCE for 3 consecutive frames
    MIN_CRITICAL_DISTANCE = 0.55  # Immediate safety stop boundary

    # --- PID & Velocity Control Limits ---
    MAX_LINEAR_VEL = 0.22         # Physical limit for TurtleBot3 Burger (m/s)
    MIN_ROLLING_VEL = 0.08        # Minimum speed to overcome static carpet/tile wheel friction (m/s)
    MAX_ANGULAR_VEL = 2.00        # Maximum angular velocity (rad/s)
    STEERING_DEADZONE = 0.05      # ±5% normalized horizontal deadzone
    ANGULAR_DEADZONE = STEERING_DEADZONE  # Compatibility alias
    
    # Slew-rate acceleration limits
    MAX_ACCEL_LINEAR = 1.60       # m/s^2
    MAX_ACCEL_ANGULAR = 8.00      # rad/s^2

    # --- Camera geometry and LiDAR fusion ---
    CAMERA_HFOV_DEG = 57.0        # horizontal field of view; focal length in px = (w/2)/tan(hfov/2)
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
    LIDAR_TRACK_MAX_SEC = 12.0    # keep following on LiDAR alone this long when the camera cannot see the owner
    LIDAR_TRACK_GATE_M = 0.50     # LiDAR person-cluster must be this close to where the EKF expects the owner
    MEMORY_PURSUIT_SEC = 25.0     # drive to where the owner was last seen (around obstacles) for up to this long
    MEMORY_SPEED = 0.15           # m/s while recalling, slower than following because nobody is in view
    MEMORY_PLAN_GRACE_SEC = 1.0   # wait this long for the first plan before giving up on the map and searching
    SEARCH_SWEEP_SEC = 6.0        # sweep phase length before the full-circle scan starts
    SEARCH_SCAN_SPEED = 0.80      # rad/s during the full-circle scan
    SEARCH_MEMORY_SEC = 30.0      # how long the EKF keeps coasting the owner while searching
    MIN_FOLLOW_DISTANCE = 0.60    # the dashboard cannot set the follow distance below this
    EKF_LEAD_SEC = 0.35           # steer toward where the person will be, not where they were

    # --- PID follow controller (tuned in tests/test_follow_sim.py) ---
    PID_DIST_KP = 0.45
    PID_DIST_KI = 0.04
    PID_DIST_KD = 0.20
    PID_TURN_KP = 2.20
    PID_TURN_KI = 0.05
    PID_TURN_KD = 0.35
    PID_FF_GAIN = 0.80            # feed-forward on the person's speed along the line of sight
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
