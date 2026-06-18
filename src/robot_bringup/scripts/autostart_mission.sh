#!/bin/bash
# 1. Source the global ROS 2 and workspace environments
source /opt/ros/jazzy/setup.bash
source /home/yasiru/ros2_ws/install/setup.bash

# 2. Tell ROS 2 to use the local loopback if needed or the specific IP
export ROS_IP=$(hostname -I | awk '{print $1}')
export DISPLAY=:0

# 3. Launch the safety tracker mission
# We use a slight delay to ensure the Kinect and Kobuki are fully powered
sleep 5
ros2 launch robot_bringup safety_tracker_launch.py target_object:='bottle'
