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

        # 4. RTAB-Map SLAM (Optional - Heavy)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_rtabmap_launch, 'launch', 'rtabmap.launch.py')
            ]),
            condition=IfCondition(LaunchConfiguration('use_rtabmap')),
            launch_arguments={
                'use_sim_time': 'false',
                'rgb_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/depth/image_raw',
                'camera_info_topic': '/camera/color/camera_info',
                'approx_sync': 'true',
                'visual_odometry': 'true',
                'rtabmap_viz': 'false',
                'rtabmap_args': '--delete_db_on_start'
            }.items()

        ),

        # 5. Nav2 Stack (Optional - Heavy)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
            ]),
            condition=IfCondition(LaunchConfiguration('use_nav2')),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': params_file,
                'autostart': 'true'
            }.items()
        ),

        # 6. Semantic Pipeline (Brain - AI & Logic)
        Node(
            package='yolo_detection',
            executable='yolo_detection_node',
            name='yolo_node',
            parameters=[{
                'model_path': '/home/yasiru/yolov8n.pt',
                'debug_view': True
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
