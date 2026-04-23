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

    params_file = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_bringup, 'config', 'semantic_navigation.rviz')
    
    # Load URDF for TF (Laptop needs this to reconstruct 3D space)
    urdf_file = os.path.join(pkg_bringup, 'urdf', 'kobuki.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument('target_object', default_value='bottle'),

        # 1. TF Management
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config}]
        ),

        # 2. RTAB-Map SLAM (Brain Side)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_rtabmap_launch, 'launch', 'rtabmap.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'rgb_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/depth/image_raw',
                'camera_info_topic': '/camera/color/camera_info',
                'rgb_image_transport': 'compressed',
                'approx_sync': 'true',
                'approx_sync_max_interval': '0.5',
                'topic_queue_size': '100',
                'sync_queue_size': '100',
                'visual_odometry': 'true',
                'rtabmap_viz': 'false',
                'rtabmap_args': '--delete_db_on_start --Odom2/MinInliers 3 --Sched/ReporterInterval 5'
            }.items()
        ),

        # 3. Nav2 Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': params_file,
                'autostart': 'true'
            }.items()
        ),

        # 4. Perception Pipeline (Handled by Pi)
        Node(
            package='semantic_map',
            executable='semantic_map_node',
            name='semantic_map_node'
        ),
        Node(
            package='nav_goal_sender',
            executable='nav_goal_sender_node',
            name='nav_goal_sender',
            parameters=[{'target_object': os.getenv('TARGET_OBJECT', 'bottle')}]
        ),

        # 5. Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
    ])
