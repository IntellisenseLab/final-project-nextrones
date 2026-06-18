# Semantic Mapping and Navigation System (ROS 2)

An autonomous robotic system built on ROS 2 (Jazzy) that combines 2D AI object detection (YOLOv8) with 3D depth projection, SLAM mapping, and Nav2 autonomous driving.

##  Features
- **Semantic Mapping:** Integrates YOLOv8 object detection with a physical map to give the robot a spatial "memory" of objects.
- **Hardware Agnostic AI:** Processes raw RGB-D data from a Microsoft Kinect and projects 2D pixels into a 3D global coordinate frame using TF2.
- **Autonomous Navigation:** Utilizes Nav2 and SLAM Toolbox to map unknown environments and navigate to specific semantic goals.
- **Optimized for Edge Compute:** Built to run the entire heavy inference workload, mapping, and navigation strictly on a single Raspberry Pi.

## Hardware Architecture
* **Base:** Kobuki (Turtlebot 2)
* **Vision:** Microsoft Kinect (RGB-D)
* **Compute Brain:** Raspberry Pi (Runs the entire stack: Drivers, AI Inference, SLAM Toolbox, and Navigation)

## Package Structure
The workspace consists of several custom ROS 2 packages:
* `semantic_msgs`: Custom message definitions (`Detection.msg`, `DetectionArray.msg`).
* `robot_bringup`: Core hardware drivers, URDF models, and master launch files.
* `yolo_detection`: Subscribes to camera feeds and runs YOLOv8 nano inference.
* `object_localization`: Fuses YOLO bounding boxes with Kinect depth data to calculate physical 3D world coordinates.
* `semantic_map`: Deduplicates 3D detections and drops markers into the global SLAM map.
* `nav_goal_sender`: Interface for sending goal coordinates to the Nav2 stack.

## How to Build
```bash
# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Navigate to workspace and build
cd ros2_ws
colcon build

# Source the custom workspace
source install/setup.bash
```

## How to Run
To bring up the entire robot, hardware, mapping, and AI pipeline:
```bash
ros2 launch robot_bringup pi_driver_launch.py
```
*(Note: Ensure your `yolov8n.pt` weights file is correctly located as defined in the launch parameters).*
