"""
NAV-ONLY launch — stripped-down stack for a low-resource Pi / live demo.

Drops the heavy / fragile pieces that were OOM-crashing the 4GB Pi:
  * NO YOLO  (PyTorch alone needs ~1.5-2GB — this was the crash cause)
  * NO semantic localization / mapping / goal-sender / hardware-monitor
  * NO visual odometry  (rgbd_odometry is CPU-heavy and needs camera TF) —
    RTAB-Map uses the Kobuki WHEEL odometry (/odom) instead.

What runs:  Kobuki base + Kinect driver + depth->laserscan + RTAB-Map (mapping
            on wheel odom) + Nav2.  Drive it with RViz "2D Goal Pose" or
            `ros2 topic pub /goal_pose ...`.

The four camera_*_frame transforms are published here as static TFs because
kobuki's URDF only emits them when a camera is enabled (we run xtion:=false).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    kobuki_dir = get_package_share_directory('kobuki')
    bringup_dir = get_package_share_directory('nextrones_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    mkdir_shm = ExecuteProcess(
        cmd=['bash', '-c', 'mkdir -p /dev/shm/rtabmap /dev/shm/ros_logs'],
        output='screen',
    )

    # 1. Kobuki base only (publishes /odom + odom->base_footprint TF)
    kobuki_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kobuki_dir, 'launch', 'kobuki.launch.py')),
        launch_arguments={'xtion': 'false', 'astra': 'false'}.items()
    )

    # 2. Kinect v1 driver (libfreenect, dedicated capture thread)
    kinect_driver = Node(
        package='nextrones_vision',
        executable='kinect_driver_node',
        name='kinect_driver_node',
        output='screen',
    )

    # 3. Camera TF chain — kobuki URDF omits these when no camera is enabled.
    #    base_link -> camera_rgb_frame -> {optical, depth_frame -> depth_optical}
    #    Optical frames use the standard (-90,0,-90) ROS optical rotation.
    def stf(name, args):
        return Node(package='tf2_ros', executable='static_transform_publisher',
                    name=name, arguments=args, output='screen')

    cam_tf = [
        # x fwd 0.10 m, z up 0.20 m on the Kobuki top plate (approx)
        stf('tf_base_camrgb',   ['0.10', '0', '0.20', '0', '0', '0',
                                 'base_link', 'camera_rgb_frame']),
        stf('tf_camrgb_optical', ['0', '0', '0', '-1.5708', '0', '-1.5708',
                                  'camera_rgb_frame', 'camera_rgb_optical_frame']),
        # Kinect depth sensor sits ~2.5 cm left of the RGB sensor
        stf('tf_camrgb_camdepth', ['0', '-0.025', '0', '0', '0', '0',
                                   'camera_rgb_frame', 'camera_depth_frame']),
        stf('tf_camdepth_optical', ['0', '0', '0', '-1.5708', '0', '-1.5708',
                                    'camera_depth_frame', 'camera_depth_optical_frame']),
    ]

    # 4. Depth -> fake laser scan for Nav2 / RTAB-Map
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

    # 5. RTAB-Map — mapping using WHEEL odometry (no visual odometry node).
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
            'visual_odometry':   'false',   # use Kobuki wheel odom (lighter, robust)
            'approx_sync':       'true',
            'database_path':     '/dev/shm/rtabmap/rtabmap.db',
            'rtabmap_viz':       'false',
            'rviz':              'false',
        }.items()
    )

    # 6. Nav2 navigation stack (no AMCL/map_server — RTAB-Map provides /map)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file':  os.path.join(bringup_dir, 'config', 'real_robot_nav2_params.yaml'),
        }.items(),
    )

    # Give the Kinect ~12 s to establish its USB stream before the heavy
    # RTAB-Map / Nav2 processes start competing for CPU.
    delayed_heavy = TimerAction(period=12.0, actions=[rtabmap, nav2])

    return LaunchDescription([
        mkdir_shm,
        kobuki_base,
        kinect_driver,
        *cam_tf,
        depth_to_scan,
        delayed_heavy,
    ])
