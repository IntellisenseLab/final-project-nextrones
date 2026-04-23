from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    cyclone_cfg = os.path.join(pkg_bringup, 'config', 'cyclone_dds.xml')

    return LaunchDescription([
        # 1. Kobuki Driver (Minimal)
        Node(
            package='robot_bringup',
            executable='kobuki_driver',
            name='kobuki_driver',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB1'}]
        ),

        # 2. Kinect Bridge (Publishing Compressed)
        Node(
            package='robot_bringup',
            executable='/usr/bin/bash',
            name='kinect_bridge',
            arguments=['-c', 'source /opt/ros/jazzy/setup.bash && source /home/yasiru/ros2_ws/install/setup.bash && python3 /home/yasiru/ros2_ws/src/robot_bringup/robot_bringup/kinect_bridge.py'],
            output='screen'
        ),

        # 3. YOLO Detection (On Pi)
        Node(
            package='yolo_detection',
            executable='yolo_detection_node',
            name='yolo_node',
            parameters=[{
                'model_path': '/home/yasiru/yolov8n.pt',
                'debug_view': False
            }]
        ),
        
        # 4. Object Localization (On Pi)
        Node(
            package='object_localization',
            executable='object_localization_node',
            name='localization_node'
        ),
        
        # 5. System Monitor (Watchdog)
        Node(
            package='robot_bringup',
            executable='system_monitor',
            name='system_monitor',
            output='screen'
        )
    ])
