import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', 
                                      default=os.path.join(pkg_bringup, 'config', 'navigation_params.yaml'))
    map_yaml_file = LaunchConfiguration('map', 
                                        default=os.path.join(pkg_bringup, 'config', 'map.yaml'))

    # Load URDF for TF
    urdf_file = os.path.join(pkg_bringup, 'urdf', 'kobuki.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    return LaunchDescription([
        # 0. Launch Arguments
        DeclareLaunchArgument('map', default_value=map_yaml_file, description='Full path to map yaml file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # 1. Kobuki Driver (Serial)
        Node(
            package='robot_bringup',
            executable='kobuki_driver',
            name='kobuki_driver',
            output='screen'
        ),

        # 2. Kinect Bridge (Python/Ctypes)
        Node(
            package='robot_bringup',
            executable='kinect_bridge',
            name='kinect_bridge',
            output='screen'
        ),

        # 3. TF Management
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        ),

        # 4. Laser Scan from Depth (for Nav2)
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            parameters=[{
                'output_frame': 'base_link',
                'range_min': 0.45,
                'range_max': 5.0,
                'scan_height': 10
            }],
            remappings=[
                ('depth', '/camera/depth/image_raw'),
                ('depth_camera_info', '/camera/depth/camera_info'),
                ('scan', '/scan')
            ]
        ),

        # 5. Nav2 Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
            ]),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true'
            }.items()
        ),

        # 6. AI & Autonomous Logic
        Node(
            package='robot_bringup',
            executable='yolo_detection_node',
            name='yolo_3d_node',
            output='screen'
        ),
        Node(
            package='robot_bringup',
            executable='task_manager',
            name='task_manager',
            output='screen',
            parameters=[{'target_object': 'bottle'}]
        ),
    ])
