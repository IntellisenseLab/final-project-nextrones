import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    kobuki_dir = get_package_share_directory('kobuki')
    bringup_dir = get_package_share_directory('nextrones_bringup')

    # 1. Gazebo Sim: AWS small house world + Kobuki robot + camera/lidar bridges
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kobuki_dir, 'launch', 'simulation.launch.py')
        ),
    )

    # 2. Nav2 with AMCL + pre-built aws_house map + RViz2 (use_sim_time=true)
    #    Uses kinect_sim_nav_params.yaml so Nav2 reads /scan_kinect (from
    #    depthimage_to_laserscan below) instead of the simulated LiDAR /scan_raw.
    #    This mirrors the real robot setup where no physical LiDAR is present.
    nav2_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kobuki_dir, 'launch', 'navigation_sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam': 'False',
            'params_file': os.path.join(bringup_dir, 'config', 'kinect_sim_nav_params.yaml'),
        }.items(),
    )

    # Convert simulated Kinect depth image → fake laser scan on /scan_kinect
    # The simulated LiDAR (/scan_raw) is still bridged but Nav2 ignores it here,
    # matching real robot behaviour where only Kinect is available.
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth', '/rgbd_camera/depth_image'),
            ('depth_camera_info', '/rgbd_camera/camera_info'),
            ('scan', '/scan_kinect'),
        ],
        parameters=[{
            'use_sim_time': True,
            'output_frame': 'camera_link',
            'range_min': 0.3,
            'range_max': 7.9,
        }],
        output='screen',
    )

    # 3. YOLO detection – image_topic overridden to match Gazebo bridge output
    yolo_model = os.path.join(os.path.expanduser('~'), 'ros_final', 'yolov8n.pt')
    yolo_node = Node(
        package='nextrones_vision',
        executable='yolo_detection_node',
        name='yolo_detection_node',
        parameters=[
            {'use_sim_time': True},
            {'image_topic': '/rgbd_camera/image'},
            {'model': yolo_model},
        ],
        output='screen',
    )

    # 4. Object localization – remap hardcoded depth/camera_info topic names
    #    max_depth raised to 7.9 m (Gazebo camera range) vs 4.5 m (real Kinect limit)
    localization_node = Node(
        package='nextrones_localization',
        executable='object_localization_node',
        name='object_localization_node',
        parameters=[{'use_sim_time': True, 'max_depth': 7.9}],
        remappings=[
            ('/camera/depth/image_raw', '/rgbd_camera/depth_image'),
            ('/camera/rgb/camera_info', '/rgbd_camera/camera_info'),
        ],
        output='screen',
    )

    # 5. Semantic mapping
    mapping_node = Node(
        package='nextrones_mapping',
        executable='semantic_map_node',
        name='semantic_map_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 6. Nav goal sender
    nav_goal_node = Node(
        package='nextrones_navigation',
        executable='nav_goal_sender_node',
        name='nav_goal_sender_node',
        parameters=[
            {'use_sim_time': True},
            {'target_label': LaunchConfiguration('target_object')},
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_object',
            default_value='chair',
            description='Object label for the robot to navigate toward',
        ),
        gazebo_sim,
        nav2_sim,
        depth_to_scan,
        yolo_node,
        localization_node,
        mapping_node,
        nav_goal_node,
    ])
