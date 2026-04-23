import os
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_rtabmap_launch = get_package_share_directory('rtabmap_launch')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    params_file = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')

    # Load URDF for TF (Required for Visual Odometry to know where the camera is)
    urdf_file = os.path.join(pkg_bringup, 'urdf', 'kobuki.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument('target_object', default_value='bottle'),
        DeclareLaunchArgument('use_rtabmap', default_value='true'),
        DeclareLaunchArgument('use_nav2', default_value='true'),


        # 1. TF Management
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config}]
        ),
        
        # 2. Kobuki Driver
        Node(
            package='robot_bringup',
            executable='kobuki_driver',
            name='kobuki_driver',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0'}]
        ),

        # 3. Kinect Bridge
        Node(
            package='robot_bringup',
            executable='/usr/bin/bash',
            name='kinect_bridge',
            arguments=['-c', 'source /opt/ros/jazzy/setup.bash && source /home/yasiru/ros2_ws/install/setup.bash && python3 /home/yasiru/ros2_ws/src/robot_bringup/robot_bringup/kinect_bridge.py'],
            output='screen'
        ),

        # 4. Depth to Scan (For 2D SLAM)
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[
                ('image', '/camera/depth/image_raw'),
                ('camera_info', '/camera/depth/camera_info'),
                ('scan', '/scan')
            ],
            parameters=[{
                'output_frame': 'camera_depth_optical_frame',
                'range_min': 0.45,
                'range_max': 5.0,
                'scan_height': 10
            }]
        ),

        # 5. SLAM Toolbox (Local 2D Mapping)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': os.path.join(pkg_bringup, 'config', 'mapper_params_online_async.yaml')
            }.items()
        ),

        # 6. Nav2 Stack (Local)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
            ]),
            condition=IfCondition(LaunchConfiguration('use_nav2')),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': params_file,
                'map': '', # We use SLAM Toolbox as map provider
                'autostart': 'true'
            }.items()
        ),

        # 7. Semantic Pipeline (Local - Everything on Pi)
        Node(
            package='yolo_detection',
            executable='yolo_detection_node',
            name='yolo_node',
            parameters=[{
                'model_path': '/home/yasiru/yolov8n.pt',
                'debug_view': False # Disable window on Pi to save headless resources
            }]
        ),
        Node(
            package='object_localization',
            executable='object_localization_node',
            name='localization_node'
        ),
        Node(
            package='semantic_map',
            executable='semantic_map_node',
            name='semantic_map_node'
        ),
        Node(
            package='nav_goal_sender',
            executable='nav_goal_sender_node',
            name='nav_goal_sender',
            parameters=[{'target_object': LaunchConfiguration('target_object')}]
        ),
        Node(
            package='robot_bringup',
            executable='system_monitor',
            name='system_monitor',
            output='screen'
        )
    ])
