"""
PI SIDE — hardware drivers ONLY.  Tiny CPU/RAM load so the 4GB Pi won't OOM.

Runs:  Kobuki base (+ /odom, odom->base_footprint TF)
       Kinect driver (RGB + depth + camera_info)
       camera_*_frame static TFs
       depth -> /scan_filtered

All compute (RTAB-Map SLAM, Nav2, YOLO, localization, mapping, goals) runs on
the PC via pc_compute_launch.py, over the ROS 2 network (same ROS_DOMAIN_ID).
The PC publishes map->odom and /cmd_vel; the Kobuki here consumes /cmd_vel.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    kobuki_dir = get_package_share_directory('kobuki')

    mkdir_shm = ExecuteProcess(
        cmd=['bash', '-c', 'mkdir -p /dev/shm/ros_logs'],
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

    return LaunchDescription([
        mkdir_shm,
        kobuki_base,
        kinect_driver,
        *cam_tf,
        depth_to_scan,
    ])
