import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(pkg_bringup, 'config', 'navigation_params.yaml')

    # Load URDF for TF
    urdf_file = os.path.join(pkg_bringup, 'urdf', 'kobuki.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    return LaunchDescription([
        # 1. Provide Robot URDF/TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config}]
        ),
        
        # 2. Start Drivers
        Node(
            package='robot_bringup',
            executable='kobuki_driver',
            name='kobuki_driver'
        ),
        Node(
            package='robot_bringup',
            executable='kinect_bridge',
            name='kinect_bridge'
        ),
        
        # 3. Laser Scan (from Depth)
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            remappings=[
                ('depth', '/camera/depth/image_raw'),
                ('depth_camera_info', '/camera/depth/camera_info'),
                ('scan', '/scan')
            ],
            parameters=[{'output_frame': 'base_link'}]
        ),

        # 4. SLAM Toolbox (Asynchronous Mapping)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': params_file
            }.items()
        ),

        # 5. Nav2 (Navigation without a static map server)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': params_file,
                'use_lifecycle_mgr': 'true',
                'autostart': 'true'
            }.items()
        ),

        # 6. AI & Autonomous Logic
        Node(
            package='robot_bringup',
            executable='yolo_detection_node',
            name='yolo_3d_node'
        ),
        Node(
            package='robot_bringup',
            executable='task_manager',
            name='task_manager',
            parameters=[{'target_object': 'bottle'}]
        ),
    ])
