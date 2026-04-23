#!/bin/bash
# 1. Kill old nodes
sudo pkill -9 -f ros2
sudo pkill -9 -f python3
sudo rm -rf /dev/shm/*

# 2. Source ROS
source /opt/ros/jazzy/setup.bash
source /home/yasiru/ros2_ws/install/setup.bash

# 3. Network Config
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/yasiru/ros2_ws/src/robot_bringup/config/cyclone_dds.xml

# 4. Launch
echo "🚀 Starting Autonomous Mission..."
ros2 launch robot_bringup lightweight_pi.launch.py target_object:=person
