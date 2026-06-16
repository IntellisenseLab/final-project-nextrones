#!/usr/bin/env bash
# PI side: hardware drivers ONLY (Kobuki + Kinect + scan). Light load.
# All compute runs on the PC (see run_pc.sh).
# Usage:  bash ~/ros_final/scripts/run_pi.sh
set -e

echo "[run_pi] killing leftovers..."
pkill -9 -f "[r]os2 launch"       2>/dev/null || true
pkill -9 -f "[k]inect_driver"     2>/dev/null || true
pkill -9 -f "[k]obuki"            2>/dev/null || true
sleep 2

echo "[run_pi] sourcing ROS..."
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
source "$HOME/ros_final/install/setup.bash"

echo "[run_pi] launching DRIVERS only..."
exec ros2 launch nextrones_bringup pi_drivers_launch.py
