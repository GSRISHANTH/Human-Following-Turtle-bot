#!/bin/bash
# Camera and command bridge only. Does not launch OpenCR and does not enable torque.
# Use this while the motors are not connected.

export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}
export LDS_MODEL=${LDS_MODEL:-LDS-02}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
export ROS_LOCALHOST_ONLY=0

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/foxy/setup.bash ]; then
    source /opt/ros/foxy/setup.bash
fi

if [ -f "$HOME/turtlebot3_ws/install/setup.bash" ]; then
    source "$HOME/turtlebot3_ws/install/setup.bash"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[TurtleBot] Sensors only. Motors stay off."
exec python3 -u "$SCRIPT_DIR/pi_turtlebot_node.py" --no-motors
