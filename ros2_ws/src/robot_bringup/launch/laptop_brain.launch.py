import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    
    # Path to Nav2 and SLAM parameters (optimized for Laptop)
    params_file = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    mapper_params = os.path.join(pkg_bringup, 'config', 'mapper_params_online_async.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('target_object', default_value='bottle'),

        # 1. 2D SLAM (Running on Laptop for stability)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[mapper_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),

        # 2. Navigation 2 (Running on Laptop for high-performance planning)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': params_file,
                'autostart': 'true',
                'map': ''
            }.items()
        ),

        # 3. RViz2 (Visualization)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_bringup, 'config', 'semantic_navigation.rviz')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
        
        # 4. Mission Control (Goal Sender - can also run on Laptop)
        Node(
            package='nav_goal_sender',
            executable='nav_goal_sender_node',
            name='nav_goal_sender',
            parameters=[{'target_object': LaunchConfiguration('target_object')}]
        )
    ])
