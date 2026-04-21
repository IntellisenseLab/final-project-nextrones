import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    
    return LaunchDescription([
        DeclareLaunchArgument('target_object', default_value='bottle'),

        # 1. Kobuki Driver

        Node(
            package='robot_bringup',
            executable='kobuki_driver',
            name='kobuki_driver',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0'}]
        ),

        # 2. Kinect Bridge
        Node(
            package='robot_bringup',
            executable='/usr/bin/bash',
            name='kinect_bridge',
            arguments=['-c', 'source /opt/ros/jazzy/setup.bash && source /home/yasiru/ros2_ws/install/setup.bash && python3 /home/yasiru/ros2_ws/src/robot_bringup/robot_bringup/kinect_bridge.py'],
            output='screen'
        ),

        # 3. YOLO Detection (Optimized)
        Node(
            package='yolo_detection',
            executable='yolo_detection_node',
            name='yolo_node',
            parameters=[{
                'model_path': '/home/yasiru/yolov8n.pt',
                'target_object': LaunchConfiguration('target_object'),
                'debug_view': False
            }]
        ),

        # 4. Emergency Visual Tracker
        Node(
            package='robot_bringup',
            executable='visual_tracker',
            name='visual_tracker',
            output='screen',
            parameters=[{'target_label': LaunchConfiguration('target_object')}]
        ),


        # 5. System Monitor
        Node(
            package='robot_bringup',
            executable='system_monitor',
            name='system_monitor',
            output='screen'
        )
    ])
