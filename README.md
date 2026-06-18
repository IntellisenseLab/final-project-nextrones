# Semantic Mapping and Navigation System (ROS 2)

An autonomous robotic system built on ROS 2 (Jazzy) that combines 2D AI object detection (YOLOv8) with 3D depth projection, SLAM mapping, and Nav2 autonomous driving. This project was developed as a final academic robotics project by the IntellisenseLab team.

## Features
- **Semantic Mapping:** Integrates YOLOv8 object detection with a physical map to give the robot a spatial "memory" of objects.
- **Hardware Agnostic AI:** Processes raw RGB-D data from a Microsoft Kinect and projects 2D pixels into a 3D global coordinate frame using TF2.
- **Autonomous Navigation:** Utilizes Nav2 and SLAM Toolbox to map unknown environments and navigate to specific semantic goals.
- **Optimized for Edge Compute:** Built to run the entire heavy inference workload, mapping, and navigation strictly on a single Raspberry Pi.

## How it Works (The Pipeline)
1. **Perception (YOLOv8):** The robot subscribes to the Kinect RGB camera and runs a lightweight YOLOv8 Nano model to identify objects (e.g., bottles, chairs) and find their exact 2D pixel center.
2. **3D Localization:** Using the Kinect's depth sensor, the system extracts the physical distance of that specific pixel. ROS 2 TF (Transform) then mathematically projects that coordinate from the camera's lens into the global SLAM floorplan.
3. **Semantic Mapping:** The system filters out duplicate detections (e.g., if two bottles are within 0.5m of each other) and saves the unique 3D coordinates into a live, spatial database.
4. **Autonomous Navigation:** When a target is selected, the ROS 2 Nav2 stack automatically calculates an obstacle-free path across the SLAM Occupancy Grid and sends velocity commands to the Kobuki wheels to reach the object.

## Hardware Architecture
* **Base:** Kobuki (Turtlebot 2)
* **Vision:** Microsoft Kinect (RGB-D)
* **Compute Brain:** Raspberry Pi (Runs the entire stack: Drivers, AI Inference, SLAM Toolbox, and Navigation)

## Package Structure
* `semantic_msgs`: Custom message definitions (`Detection.msg`, `DetectionArray.msg`).
* `robot_bringup`: Core hardware drivers, URDF models, and master launch files.
* `yolo_detection`: Subscribes to camera feeds and runs YOLOv8 nano inference.
* `object_localization`: Fuses YOLO bounding boxes with Kinect depth data to calculate physical 3D world coordinates.
* `semantic_map`: Deduplicates 3D detections and drops markers into the global SLAM map.
* `nav_goal_sender`: Interface for sending goal coordinates to the Nav2 stack.

## 📋 Prerequisites
* **OS:** Ubuntu 24.04 (Noble Numbat)
* **ROS Version:** ROS 2 Jazzy Jalisco
* **Python:** 3.10+

## How to Build & Install

```bash
# 1. Clone the repository
git clone git@github.com:IntellisenseLab/final-project-nextrones.git
cd final-project-nextrones

# 2. Install Python Dependencies
pip install -r requirements.txt

# 3. Download YOLOv8 Weights
# This downloads the required AI model directly to the root of your workspace
wget https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt

# 4. Source ROS 2 Jazzy and Build
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# 5. Source the custom workspace
source install/setup.bash
```

## How to Run
To bring up the entire robot, hardware, mapping, and AI pipeline:
```bash
ros2 launch robot_bringup pi_driver_launch.py
```
*(Note: Ensure your `yolov8n.pt` weights file is located at the path defined in your launch parameters).*

**Custom Launch Arguments:**
You can tell the robot to search for different objects dynamically by passing the `target_object` argument:
```bash
ros2 launch robot_bringup pi_driver_launch.py target_object:='chair'
```

## Team Members
* **Yasiru Bandara**
* **Hasini Dilmanie**
* **Sandali Divyangi**

## License
This project is licensed under the [MIT License](LICENSE).
