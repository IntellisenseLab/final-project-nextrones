"""
PC SIDE — ALL compute.  Runs on the powerful PC, talks to the Pi's drivers
over the ROS 2 network (same ROS_DOMAIN_ID).  Keeps the 4GB Pi from OOMing.

Runs:  RTAB-Map SLAM (wheel odom)  + Nav2  + YOLO detection
       + object localization + semantic map + nav goal sender

Subscribes to the Pi's /camera/*, /odom, /scan_filtered, /tf.
Publishes back map->odom (RTAB-Map) and /cmd_vel (Nav2) to drive the Kobuki.

Run pi_drivers_launch.py on the Pi FIRST, then this on the PC.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            ExecuteProcess, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_yolo_model():
    base = os.path.expanduser('~/ros_final')
    ncnn = os.path.join(base, 'yolov8n_ncnn_model')
    pt = os.path.join(base, 'yolov8n.pt')
    return ncnn if os.path.isdir(ncnn) else pt


def generate_launch_description():
    bringup_dir = get_package_share_directory('nextrones_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    mkdir_shm = ExecuteProcess(
        cmd=['bash', '-c', 'mkdir -p /dev/shm/rtabmap /dev/shm/ros_logs'],
        output='screen',
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rtabmap_launch'),
                         'launch', 'rtabmap.launch.py')),
        launch_arguments={
            'rtabmap_args':      '--delete_db_on_start',
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

    # Give RTAB-Map/Nav2 a few seconds to latch onto the Pi's topics before
    # the semantic layer and YOLO start.
    delayed = TimerAction(period=5.0, actions=[
        yolo_node, localization_node, mapping_node, nav_goal_node,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('target_object', default_value='bottle'),
        mkdir_shm,
        rtabmap,
        nav2,
        delayed,
    ])
