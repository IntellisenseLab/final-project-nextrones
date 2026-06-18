import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_rtabmap_launch = get_package_share_directory('rtabmap_launch')

    return LaunchDescription([
        # 1. SLAM (Computationly Heavy - Offloaded to Laptop)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_rtabmap_launch, 'launch', 'rtabmap.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'rgb_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/depth/image_raw',
                'camera_info_topic': '/camera/color/camera_info',
                'approx_sync': 'true',
                'visual_odometry': 'true',
                'rtabmap_args': '--delete_db_on_start'
            }.items()
        ),

        # 2. Nav2 Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': os.path.join(pkg_bringup, 'config', 'nav2_params.yaml'),
                'autostart': 'true',
                'use_collision_monitor': 'false',
                'use_velocity_smoother': 'false'
            }.items()
        ),

        # 3. RViz2 (Monitor & UI)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_bringup, 'config', 'week9_report.rviz')]
        )
    ])
