#!/bin/bash
# Build the workspace on the Raspberry Pi.
# Usage: bash build_pi.sh
set -e
cd ~/ros_final

source /opt/ros/jazzy/setup.bash
# ros2_ws contains ecl_build cmake macros — must be in the chain
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
[ -f ~/ros_final/install/setup.bash ] && source ~/ros_final/install/setup.bash || true

rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true

colcon build --symlink-install --parallel-workers 2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --allow-overriding ecl_geometry ecl_mobile_robot kobuki_ros_interfaces kobuki_core

source install/setup.bash
echo "Build complete."
