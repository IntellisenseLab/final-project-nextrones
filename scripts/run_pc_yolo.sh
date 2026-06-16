#!/usr/bin/env bash
# PC side: run YOLO detection here (heavy PyTorch stays off the Pi).
# Subscribes to the Pi's /camera/rgb/image_raw over the ROS 2 network.
# Usage:  bash ~/ros_final/scripts/run_pc_yolo.sh
set -e

echo "[run_pc_yolo] killing any leftover YOLO process..."
pkill -9 -f yolo_detection_node 2>/dev/null || true
sleep 1

echo "[run_pc_yolo] sourcing ROS..."
source /opt/ros/jazzy/setup.bash
source "$HOME/ros_final/install/setup.bash"

echo "[run_pc_yolo] starting YOLO on COMPRESSED camera stream (reliable over WiFi)..."
exec ros2 run nextrones_vision yolo_detection_node --ros-args \
    -p image_topic:=/camera/rgb/image_raw/compressed \
    -p use_compressed:=true \
    -p model:="$HOME/ros_final/yolov8n.pt"
