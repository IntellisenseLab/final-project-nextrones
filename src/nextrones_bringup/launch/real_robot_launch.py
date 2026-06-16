import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_yolo_model():
    """Return NCNN model path if available, otherwise .pt — NCNN is ~3x faster on ARM."""
    base = os.path.expanduser('~/ros_final')
    ncnn = os.path.join(base, 'yolov8n_ncnn_model')
    pt = os.path.join(base, 'yolov8n.pt')
    if os.path.isdir(ncnn):
        return ncnn
    return pt


def generate_launch_description():
    kobuki_dir = get_package_share_directory('kobuki')
    bringup_dir = get_package_share_directory('nextrones_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 1. Kobuki base only (NO xtion/astra — we start Kinect separately below)
    kobuki_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kobuki_dir, 'launch', 'kobuki.launch.py')),
        launch_arguments={'xtion': 'false', 'astra': 'false'}.items()
    )

    # 2. Kinect v1 driver — uses libfreenect Python bindings
    #    Publishes: /camera/rgb/image_raw, /camera/depth/image_raw, camera_info topics
    #    Requires: sudo apt install libfreenect-dev python3-freenect
    kinect_driver = Node(
        package='nextrones_vision',
        executable='kinect_driver_node',
        name='kinect_driver_node',
        output='screen',
    )

    # 3. Convert Kinect depth → fake laser scan (/scan_filtered)
    #    Replaces physical LiDAR — Nav2 subscribes to /scan_filtered
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        remappings=[
            ('depth',             '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan',              '/scan_filtered'),
        ],
        parameters=[{
            'output_frame': 'camera_depth_frame',
            'range_min': 0.45,
            'range_max': 4.5,
        }],
        output='screen',
    )

    # 3.5 Camera TF chain and Map->Odom dummy TF
    def stf(name, args):
        return Node(package='tf2_ros', executable='static_transform_publisher',
                    name=name, arguments=args, output='screen')

    cam_tf = [
        # Base to Camera RGB
        stf('tf_base_camrgb',   ['0.10', '0', '0.20', '0', '0', '0', 'base_link', 'camera_rgb_frame']),
        stf('tf_camrgb_optical', ['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_rgb_frame', 'camera_rgb_optical_frame']),
        # Camera RGB to Camera Depth
        stf('tf_camrgb_camdepth', ['0', '-0.025', '0', '0', '0', '0', 'camera_rgb_frame', 'camera_depth_frame']),
        stf('tf_camdepth_optical', ['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_depth_frame', 'camera_depth_optical_frame']),
        # Dummy Map to Odom (since we removed RTAB-Map)
        stf('tf_map_odom', ['0', '0', '0', '0', '0', '0', 'map', 'odom'])
    ]

    # 4. RTAB-Map SLAM (REMOVED to save CPU for the bottle test)
    # Nav2 uses this /map for the global costmap static layer, but we provide a dummy TF above.
    # rtabmap = IncludeLaunchDescription(...)

    # 5. Nav2 navigation stack only (no AMCL/map_server — RTAB-Map provides localization+map)
    #    navigation_launch.py manages: controller, planner, bt_navigator, behaviors, etc.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file':  os.path.join(bringup_dir, 'config', 'real_robot_nav2_params.yaml'),
        }.items(),
    )

    # 6. YOLO detection — auto-selects NCNN (fast) or .pt (fallback)
    yolo_node = Node(
        package='nextrones_vision',
        executable='yolo_detection_node',
        name='yolo_detection_node',
        parameters=[{
            'image_topic': '/camera/rgb/image_raw',
            'model': _find_yolo_model(),
        }],
        output='screen',
    )

    # 7. Object localization (Kinect max reliable range = 4.5m)
    localization_node = Node(
        package='nextrones_localization',
        executable='object_localization_node',
        name='object_localization_node',
        parameters=[{'max_depth': 4.5}],
        output='screen',
    )

    # 8. Semantic map
    mapping_node = Node(
        package='nextrones_mapping',
        executable='semantic_map_node',
        name='semantic_map_node',
        output='screen',
    )

    # 9. Navigation goal sender
    nav_goal_node = Node(
        package='nextrones_navigation',
        executable='nav_goal_sender_node',
        name='nav_goal_sender_node',
        parameters=[{'target_label': LaunchConfiguration('target_object')}],
        output='screen',
    )

    # 10. Hardware monitor — checks Kobuki USB, Kinect USB, RAM, CPU temp, topics
    hardware_monitor = Node(
        package='nextrones_diagnostics',
        executable='hardware_monitor_node',
        name='hardware_monitor_node',
        parameters=[{'check_interval': 10.0}],
        output='screen',
    )

    # Create RAM disk directories (lost on reboot, /dev/shm is tmpfs)
    mkdir_shm = ExecuteProcess(
        cmd=['bash', '-c', 'mkdir -p /dev/shm/rtabmap /dev/shm/ros_logs'],
        output='screen',
    )

    # Bring up the Kinect + base + scan FIRST and give the camera ~12 s to
    # establish its USB stream. Then start the heavy CPU consumers (RTAB-Map,
    # Nav2, YOLO). Launching everything at once starves the Kinect's first
    # sync_get_*() on the 4-core Pi and it never produces a frame.
    delayed_heavy = TimerAction(
        period=12.0,
        actions=[
            # rtabmap, (Removed)
            nav2,
            yolo_node,
            localization_node,
            mapping_node,
            nav_goal_node,
            hardware_monitor,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('target_object', default_value='bottle'),
        mkdir_shm,
        kobuki_base,
        kinect_driver,
        *cam_tf,
        depth_to_scan,
        delayed_heavy,
    ])
