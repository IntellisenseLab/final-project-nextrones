# Nextrones — Claude Code Context

Semantic navigation robot: Kobuki base + Kinect camera. Detects objects with YOLOv8n,
localizes them in 3D, builds a semantic map, and drives to a named target.

## Workspace layout

```
ros_final/
├── src/
│   ├── nextrones_bringup/       launch files (main entry points)
│   ├── nextrones_vision/        YOLO detection node
│   ├── nextrones_localization/  depth deprojection → 3D map coords
│   ├── nextrones_mapping/       semantic marker map
│   ├── nextrones_navigation/    Nav2 goal sender
│   ├── nextrones_diagnostics/   hardware health monitor (Pi only)
│   ├── nextrones_interfaces/    custom ROS 2 message types
│   └── ThirdParty/              kobuki_ros, kobuki_ros_interfaces, AWS world
├── scripts/
│   ├── setup_pi.sh              one-time Pi dependency installer
│   ├── build_pi.sh              colcon build for Pi
│   ├── sync_to_pi.sh            rsync PC → Pi
│   ├── diagnose.sh              full diagnostics (run on Pi or via SSH)
│   ├── ssh_debug.sh             remote diagnose from PC
│   └── convert_yolo_ncnn.py     convert .pt → NCNN (3x faster on ARM)
├── yolov8n.pt                   YOLO weights (PyTorch)
└── yolov8n_ncnn_model/          NCNN weights (faster on Pi, created by convert script)
```

## Launch files

| File | Purpose |
|---|---|
| `simulation_launch.py` | Full simulation (PC, Kinect-scan mode) |
| `pi_simulation_launch.py` | Simulation mirroring Pi config exactly |
| `real_robot_launch.py` | Real Kobuki + Kinect on Raspberry Pi |

## How to run

```bash
cd ~/ros_final
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# Simulation (standard)
ros2 launch nextrones_bringup simulation_launch.py target_object:=chair

# Simulation (Pi config verification)
ros2 launch nextrones_bringup pi_simulation_launch.py target_object:=chair

# Real robot (on Pi)
ros2 launch nextrones_bringup real_robot_launch.py target_object:=bottle
```

## Deploying to Raspberry Pi

```bash
# 1. Convert YOLO to NCNN (on PC, once)
python3 scripts/convert_yolo_ncnn.py

# 2. Sync to Pi
bash scripts/sync_to_pi.sh pi@<PI_IP>

# 3. Build on Pi (SSH in first)
ssh pi@<PI_IP>
bash ~/ros_final/scripts/build_pi.sh

# 4. Run
ros2 launch nextrones_bringup real_robot_launch.py target_object:=bottle
```

## Debugging Pi deployment (Claude Code)

When the user reports an error on the Pi, run this to get full diagnostics:

```bash
bash scripts/ssh_debug.sh pi@<PI_IP>
```

This captures: USB hardware status, RAM, CPU temp, ROS node/topic list,
topic frequencies, recent error logs, and installed package status.

**Common errors and fixes:**

| Error | Likely cause | Fix |
|---|---|---|
| `Kobuki NOT FOUND` | USB not connected or udev rule missing | Check cable; re-run `setup_pi.sh` for udev rules |
| `Kinect NOT FOUND` | USB 2.0 port used | Move to USB 3.0 port |
| `scan_filtered NOT PUBLISHING` | Kinect depth not streaming | Check `openni2_launch` started; check USB |
| `AMCL no scan received` | depthimage_to_laserscan not running | Check node is in process list; check depth topic |
| `TF extrapolation` | Clock skew between nodes | All nodes must have `use_sim_time: False` on real robot |
| `ModuleNotFoundError: ultralytics` | pip install missing | `pip3 install ultralytics --break-system-packages` |
| `numpy incompatible` | NumPy 2.x with cv_bridge | `pip3 install "numpy<2" --break-system-packages` |
| `Nav2 goal rejected` | RTAB-Map map not built yet | Wait 10-15s for RTAB-Map to initialise before objects detected |
| `CPU throttling` | Overheating | Add heatsink/fan; check temp with diagnose.sh |
| `Low memory` | 4GB RAM full | Close other apps; verify swap is active |

## Key topic remappings

Gazebo publishes under `/rgbd_camera/*`; real Kinect under `/camera/*`.
The launch files handle all remappings — nodes always subscribe to `/camera/*`.

## TF tree

```
map → odom → base_footprint → base_link → camera_rgb_frame → camera_link
```

AMCL publishes `map → odom`. Kobuki odometry publishes `odom → base_footprint`.
Robot URDF publishes the rest statically.

## Important notes

- `--symlink-install` does NOT symlink Python files — must rebuild after editing nodes
- `max_depth: 7.9` in simulation (Gazebo camera), `4.5` on real Kinect
- RTAB-Map DB stored in `/dev/shm/rtabmap/` (RAM) to avoid SD card wear — lost on reboot
- NCNN model auto-selected if `yolov8n_ncnn_model/` exists in `~/ros_final/`
