import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    params_file = os.path.join(pkg_bringup, 'config', 'navigation_params.yaml')

    # Load URDF for TF
    urdf_file = os.path.join(pkg_bringup, 'urdf', 'kobuki.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    return LaunchDescription([
        # 1. TF Management & URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        ),

        # 2. SLAM Toolbox (High Compute)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': params_file
            }.items()
        ),

        # 3. Nav2 Stack (High Compute)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': params_file,
                'autostart': 'true'
            }.items()
        ),

        # 4. AI & Autonomous Logic
        Node(
            package='yolo_detection',
            executable='yolo_detection_node',
            name='yolo_node',
            output='screen'
        ),
        Node(
            package='robot_bringup',
            executable='task_manager',
            name='task_manager',
            parameters=[{'target_object': 'bottle'}]
        ),
        
        # 5. Visualizer & Monitor
        Node(
            package='robot_bringup',
            executable='system_monitor',
            name='system_monitor',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_bringup, 'config', 'week9_report.rviz')]
        )
    ])
