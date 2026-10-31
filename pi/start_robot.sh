#!/bin/bash
# -----------------------------------------------------------------------------
# TurtleBot3 Complete Companion Launcher (Raspberry Pi Side)
# Runs hardware bringup (OpenCR motor controllers) + AI Companion Node (Camera & /cmd_vel)
# -----------------------------------------------------------------------------

export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}
export LDS_MODEL=${LDS_MODEL:-LDS-02}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
export ROS_LOCALHOST_ONLY=0

# 1. Source ROS Distro & Workspace
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash ]; then
    source /opt/ros/foxy/setup.bash
fi

if [ -f ~/turtlebot3_ws/install/setup.bash ]; then
    source ~/turtlebot3_ws/install/setup.bash
fi

# 2. Launch TurtleBot Hardware Bringup (OpenCR motor controller) if not active
if ! pgrep -f "turtlebot3_ros" > /dev/null; then
    echo "[TurtleBot] Starting OpenCR hardware bringup..."
    ros2 launch turtlebot3_bringup robot.launch.py &
    sleep 5
    # Enable motor torque
    ros2 service call /motor_power std_srvs/srv/SetBool "{data: true}" 2>/dev/null &
    sleep 1
else
    echo "[TurtleBot] OpenCR hardware bringup already active."
fi

# 3. Launch Camera Streamer & Motor Control Bridge
# The node lives next to this script (the pi/ folder), wherever it was copied.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[TurtleBot] Starting AI Companion Bridge (Camera + /cmd_vel)..."
exec python3 -u "$SCRIPT_DIR/pi_turtlebot_node.py"

