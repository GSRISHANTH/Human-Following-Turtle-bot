"""
Configuration settings for Robust Owner-Locked Human-Following TurtleBot System.
Centralizes all perception, Re-ID, state-machine, control, safety, and network parameters.
"""

import os

class Config:
    # --- Network / Stream Settings ---
    PI_IP = "10.252.159.140"
    PI_STREAM_PORT = 5000
    PI_STREAM_PATH = "/video_feed"
    
    @classmethod
    def get_stream_url(cls):
        return f"http://{cls.PI_IP}:{cls.PI_STREAM_PORT}{cls.PI_STREAM_PATH}"

    # Local web dashboard
    DASHBOARD_HOST = "0.0.0.0"
    DASHBOARD_PORT = 8000

    # Model asset path
    MODEL_PATH = os.path.join(os.path.dirname(__file__), "pose_landmarker_lite.task")

    # --- Processing Parameters ---
    FRAME_WIDTH = 640
    FRAME_HEIGHT = 480
    TARGET_FPS = 25
    STALE_FRAME_TIMEOUT = 0.60  # seconds before declaring video stream stale

    # --- State Machine States ---
    STATE_IDLE = "IDLE"
    STATE_OWNER_ENROLLMENT = "OWNER_ENROLLMENT"
    STATE_OWNER_LOCKED = "OWNER_LOCKED"
    STATE_FOLLOWING = "FOLLOWING"
    STATE_WAIT_FOR_OWNER_MOVEMENT = "WAIT_FOR_OWNER_MOVEMENT"
    STATE_TARGET_TEMPORARILY_LOST = "TARGET_TEMPORARILY_LOST"
    STATE_SEARCHING = "SEARCH_ROTATING"
    STATE_TARGET_AMBIGUOUS = "TARGET_AMBIGUOUS"
    STATE_SAFETY_STOP = "SAFETY_STOP"
    STATE_MANUAL_CONTROL = "MANUAL_CONTROL"
    STATE_OWNER_RELEASED = "OWNER_RELEASED"

    # --- Target Loss & Active Search ---
    SEARCH_ROTATION_SPEED = 0.35          # Slow, safe search angular velocity (rad/s)
    SEARCH_MAX_DURATION_SEC = 6.0         # Max seconds to search before timing out and stopping
    SEARCH_START_DELAY_SEC = 0.6          # Seconds after losing target before starting rotation

    # --- Following & Distance Hysteresis (Meters) ---
    DESIRED_DISTANCE = 1.20       # Desired following distance
    TARGET_DISTANCE = DESIRED_DISTANCE  # Compatibility alias
    STOP_DISTANCE = 1.10          # Enter WAIT_FOR_OWNER_MOVEMENT when d <= STOP_DISTANCE
    RESUME_DISTANCE = 1.35        # Resume FOLLOWING when d >= RESUME_DISTANCE (prevents chatter)
    RESUME_HYSTERESIS_FRAMES = 3  # Must exceed RESUME_DISTANCE for 3 consecutive frames
    MIN_CRITICAL_DISTANCE = 0.55  # Immediate safety stop boundary

    # --- PID & Velocity Control Limits ---
    MAX_LINEAR_VEL = 0.22         # Physical limit for TurtleBot3 Burger (m/s)
    MIN_ROLLING_VEL = 0.12        # Minimum speed to overcome static carpet/tile wheel friction (m/s)
    MAX_ANGULAR_VEL = 1.20        # Maximum angular velocity (rad/s)
    STEERING_DEADZONE = 0.05      # ±5% normalized horizontal deadzone
    ANGULAR_DEADZONE = STEERING_DEADZONE  # Compatibility alias
    
    # Slew-rate acceleration limits
    MAX_ACCEL_LINEAR = 0.90       # m/s^2
    MAX_ACCEL_ANGULAR = 3.60      # rad/s^2

    # --- Owner Re-Identification & Identity Protection ---
    IDENTITY_MATCH_THRESHOLD = 0.40        # Minimum score to accept candidate as Owner when bystanders present
    IDENTITY_SINGLE_PERSON_THRESHOLD = 0.28 # Minimum score when only 1 candidate exists in FOV
    IDENTITY_AMBIGUITY_DELTA = 0.08        # Trigger TARGET_AMBIGUOUS if top two candidates differ by less than this
    MAX_OWNER_TEMPLATES = 5                # Multi-view template bank size
    TEMPLATE_UPDATE_MIN_CONF = 0.75        # Only add new template when identity match is very high
    TEMPLATE_DRIFT_ALPHA = 0.05            # Maximum EMA update rate to prevent template corruption

    # --- Gesture Recognition Parameters ---
    GESTURE_CONFIRM_FRAMES = 4             # Multi-frame debounce duration
    GESTURE_COOLDOWN_SEC = 1.5             # Cooldown after accepting gesture

    # --- Recovery & Timeouts ---
    OCCLUSION_RECOVERY_SEC = 3.0           # Project motion for up to 3.0s during occlusion
    COMMAND_WATCHDOG_TIMEOUT = 0.60        # Seconds before zeroing velocity commands

    # --- Hardware Bridge Settings ---
    COMMAND_MODE = "udp"                   # 'udp', 'http', 'mock'
    HTTP_CMD_URL = f"http://{PI_IP}:5000/cmd_vel"
    UDP_HOST = PI_IP
    UDP_PORT = 8888
    TELEMETRY_HZ = 20
