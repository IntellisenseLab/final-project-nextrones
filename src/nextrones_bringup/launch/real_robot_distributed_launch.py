"""
DISTRIBUTED launch (Pi side) — full semantic stack EXCEPT YOLO.

YOLO (PyTorch, ~2GB) runs on the PC instead and subscribes to this Pi's
/camera/rgb/image_raw over the ROS 2 network (same ROS_DOMAIN_ID).  This
offloads the only OOM-causing node off the 4GB Pi.

  PC  runs:  yolo_detection_node   (publishes /nextrones/detections)
  Pi  runs:  everything else (this file)

Network traffic: RGB image Pi->PC (~5 MB/s) + tiny detections PC->Pi.
Depth stays local on the Pi (object_localization reads it here).

Start the PC side with:
  ros2 run nextrones_vision yolo_detection_node --ros-args \
      -p image_topic:=/camera/rgb/image_raw -p model:=$HOME/ros_final/yolov8n.pt

Based on nav_only_launch.py: wheel odometry, camera TFs, staggered startup.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            ExecuteProcess, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    kobuki_dir = get_package_share_directory('kobuki')
    bringup_dir = get_package_share_directory('nextrones_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    mkdir_shm = ExecuteProcess(
        cmd=['bash', '-c', 'mkdir -p /dev/shm/rtabmap /dev/shm/ros_logs'],
        output='screen',
    )

    kobuki_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kobuki_dir, 'launch', 'kobuki.launch.py')),
        launch_arguments={'xtion': 'false', 'astra': 'false'}.items()
    )

    kinect_driver = Node(
        package='nextrones_vision',
        executable='kinect_driver_node',
        name='kinect_driver_node',
        output='screen',
    )

    def stf(name, args):
        return Node(package='tf2_ros', executable='static_transform_publisher',
                    name=name, arguments=args, output='screen')

    cam_tf = [
        stf('tf_base_camrgb',   ['0.10', '0', '0.20', '0', '0', '0',
                                 'base_link', 'camera_rgb_frame']),
        stf('tf_camrgb_optical', ['0', '0', '0', '-1.5708', '0', '-1.5708',
                                  'camera_rgb_frame', 'camera_rgb_optical_frame']),
        stf('tf_camrgb_camdepth', ['0', '-0.025', '0', '0', '0', '0',
                                   'camera_rgb_frame', 'camera_depth_frame']),
        stf('tf_camdepth_optical', ['0', '0', '0', '-1.5708', '0', '-1.5708',
                                    'camera_depth_frame', 'camera_depth_optical_frame']),
    ]

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
            'output_frame': 'camera_depth_optical_frame',
            'range_min': 0.45,
            'range_max': 4.5,
        }],
        output='screen',
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rtabmap_launch'),
                         'launch', 'rtabmap.launch.py')),
        launch_arguments={
            # Lightweight 2D-only mapping to stop the Pi OOMing:
            #   Grid/3D=false        -> no octomap / no 3D point-cloud map (huge RAM saver)
            #   Mem/STMSize=5        -> small short-term memory
            #   RGBD/*Update=0.1     -> only add a map node after real motion (fewer nodes)
            #   Rtabmap/DetectionRate=1 -> process at 1 Hz (default, keep CPU low)
            'rtabmap_args':      '--delete_db_on_start '
                                 '--Grid/3D false '
                                 '--Mem/STMSize 5 '
                                 '--RGBD/LinearUpdate 0.1 '
                                 '--RGBD/AngularUpdate 0.1 '
                                 '--Rtabmap/DetectionRate 1',
            'rgb_topic':         '/camera/rgb/image_raw',
            'depth_topic':       '/camera/depth/image_raw',
            'camera_info_topic': '/camera/rgb/camera_info',
            'frame_id':          'base_footprint',
            'odom_topic':        '/odom',
            'visual_odometry':   'false',
            'approx_sync':       'true',
            'database_path':     '/dev/shm/rtabmap/rtabmap.db',
            'rtabmap_viz':       'false',
            'rviz':              'false',
        }.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file':  os.path.join(bringup_dir, 'config', 'real_robot_nav2_params.yaml'),
        }.items(),
    )

    # Lightweight semantic nodes — run on the Pi (need local depth/camera_info).
    localization_node = Node(
        package='nextrones_localization',
        executable='object_localization_node',
        name='object_localization_node',
        parameters=[{'max_depth': 4.5}],
        output='screen',
    )

    mapping_node = Node(
        package='nextrones_mapping',
        executable='semantic_map_node',
        name='semantic_map_node',
        output='screen',
    )

    nav_goal_node = Node(
        package='nextrones_navigation',
        executable='nav_goal_sender_node',
        name='nav_goal_sender_node',
        parameters=[{'target_label': LaunchConfiguration('target_object')}],
        output='screen',
    )

    # NOTE: YOLO does NOT run here. PyTorch browns out the Pi's power supply
    # (under-voltage red LED) and the ncnn pip build is broken on this ARM Pi.
    # YOLO runs on the PC instead (scripts/run_pc_yolo.sh), subscribing to the
    # JPEG-compressed /camera/rgb/image_raw/compressed topic over WiFi.

    # Camera first, then the heavy/dependent nodes ~12 s later.
    delayed = TimerAction(period=12.0, actions=[
        rtabmap, nav2, localization_node, mapping_node, nav_goal_node,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('target_object', default_value='bottle'),
        mkdir_shm,
        kobuki_base,
        kinect_driver,
        *cam_tf,
        depth_to_scan,
        delayed,
    ])
