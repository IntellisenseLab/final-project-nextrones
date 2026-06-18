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
        # online_async_launch.py's argument is named 'slam_params_file', not
        # 'params_file' - passing the wrong key silently no-ops, leaving
        # slam_toolbox on its stock defaults (base_frame: base_footprint, which
        # doesn't exist in this URDF, plus a too-tight transform_timeout). That
        # caused every single scan's odom-pose lookup to fail ("Failed to compute
        # odom pose"), which meant map->odom was never reliably published, which
        # made the global costmap (map frame) repeatedly time out, which sent
        # Nav2 into permanent Spin/BackUp recovery instead of ever following a path.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': params_file
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
        # The robot_bringup-internal yolo_3d_node (publishing String to /yolo/detections)
        # is intentionally NOT launched here: its only consumer, task_manager_node, was
        # removed above, so it would just burn ~65% CPU for no subscriber.
        Node(
            package='yolo_detection',
            executable='yolo_detection_node',
            name='yolo_detection_node',
            parameters=[{
                'model_path': '/home/yasiru/yolov8n.pt',
                'inference_interval': 1.8,
                'debug_view': False
            }]
        ),
        Node(
            package='object_localization',
            executable='object_localization_node',
            name='object_localization_node',
            remappings=[('/detections', '/yolo_detections')]
        ),
        Node(
            package='nav_goal_sender',
            executable='nav_goal_sender_node',
            name='nav_goal_sender_node'
        ),
    ])
