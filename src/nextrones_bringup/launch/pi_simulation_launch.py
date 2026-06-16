"""
Pi simulation launch — mirrors real_robot_launch.py as closely as possible
but runs in Gazebo instead of on hardware.

Differences from simulation_launch.py:
  - No RViz2 (Pi won't have a display; view from PC with: rviz2)
  - NCNN model auto-selected if present (same as Pi)
  - depth_to_scan on /scan_kinect with same params as real robot
  - hardware_monitor_node runs (harmless in sim — USB checks will report NOT FOUND, which is expected)
  - Same max_depth=7.9 for Gazebo camera range

Run this to verify Pi-ready code before deploying to hardware:
  ros2 launch nextrones_bringup pi_simulation_launch.py target_object:=chair
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_yolo_model():
    base = os.path.expanduser('~/ros_final')
    ncnn = os.path.join(base, 'yolov8n_ncnn_model')
    pt = os.path.join(base, 'yolov8n.pt')
    if os.path.isdir(ncnn):
        return ncnn
    return pt


def generate_launch_description():
    kobuki_dir = get_package_share_directory('kobuki')
    bringup_dir = get_package_share_directory('nextrones_bringup')

    # 1. Gazebo + Kobuki robot
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kobuki_dir, 'launch', 'simulation.launch.py')),
    )

    # 2. Nav2 using /scan_kinect (same as real robot — no LiDAR)
    nav2_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kobuki_dir, 'launch', 'navigation_sim.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam': 'False',
            'params_file': os.path.join(bringup_dir, 'config', 'kinect_sim_nav_params.yaml'),
            'rviz': 'False',
        }.items(),
    )

    # 3. Depth → scan (same params as real robot, adapted for Gazebo range)
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth',            '/rgbd_camera/depth_image'),
            ('depth_camera_info', '/rgbd_camera/camera_info'),
            ('scan',             '/scan_kinect'),
        ],
        parameters=[{
            'use_sim_time': True,
            'output_frame': 'camera_link',
            'range_min': 0.3,
            'range_max': 7.9,
        }],
        output='screen',
    )

    # 4. YOLO — same model selection logic as real robot
    yolo_node = Node(
        package='nextrones_vision',
        executable='yolo_detection_node',
        name='yolo_detection_node',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/rgbd_camera/image',
            'model': _find_yolo_model(),
        }],
        output='screen',
    )

    # 5. Object localization (Gazebo camera range = 7.9m)
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

    # 6. Semantic map
    mapping_node = Node(
        package='nextrones_mapping',
        executable='semantic_map_node',
        name='semantic_map_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 7. Navigation goal sender
    nav_goal_node = Node(
        package='nextrones_navigation',
        executable='nav_goal_sender_node',
        name='nav_goal_sender_node',
        parameters=[{
            'use_sim_time': True,
            'target_label': LaunchConfiguration('target_object'),
        }],
        output='screen',
    )

    # 8. Hardware monitor — USB checks show NOT FOUND in simulation, that is expected
    hardware_monitor = Node(
        package='nextrones_diagnostics',
        executable='hardware_monitor_node',
        name='hardware_monitor_node',
        parameters=[{'use_sim_time': True, 'check_interval': 10.0}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('target_object', default_value='chair'),
        gazebo_sim,
        nav2_sim,
        depth_to_scan,
        yolo_node,
        localization_node,
        mapping_node,
        nav_goal_node,
        hardware_monitor,
    ])
