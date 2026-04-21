import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')

    return LaunchDescription([
        # 1. Kobuki Driver (Serial)
        Node(
            package='robot_bringup',
            executable='kobuki_driver',
            name='kobuki_driver',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0'}] # Standard port for Kobuki on Pi
        ),

        # 2. Kinect Bridge (Direct source execution via Bash to Ensure Environment)
        Node(
            package='robot_bringup',
            executable='/usr/bin/bash',
            name='kinect_bridge',
            arguments=['-c', 'source /opt/ros/jazzy/setup.bash && source /home/hasini/ROS_FP/final-project-nextrones/ros2_ws/install/setup.bash && python3 /home/hasini/ROS_FP/final-project-nextrones/ros2_ws/src/robot_bringup/robot_bringup/kinect_bridge.py'],
            output='screen'
        ),

        # 3. Laser Scan from Depth (To reduce bandwidth, scan processed on Pi)
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_laser',
            parameters=[{
                'output_frame': 'base_link',
                'range_min': 0.45,
                'range_max': 5.0,
                'scan_height': 10
            }],
            remappings=[
                ('depth', '/camera/depth/image_raw'),
                ('depth_camera_info', '/camera/depth/camera_info'),
                ('scan', '/scan')
            ]
        ),
    ])
