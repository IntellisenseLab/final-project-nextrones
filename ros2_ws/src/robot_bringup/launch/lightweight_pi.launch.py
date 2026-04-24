import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')
    
    params_file = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')
    mapper_params = os.path.join(pkg_bringup, 'config', 'mapper_params_online_async.yaml')
    urdf_file = os.path.join(pkg_bringup, 'urdf', 'kobuki.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument('target_object', default_value='bottle'),
        DeclareLaunchArgument('inference_fps', default_value='2.0'),

        # 1. Hardware & Transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config}]
        ),
        Node(
            package='robot_bringup',
            executable='kobuki_driver',
            name='kobuki_driver',
            output='screen',
            parameters=[{'port': '/dev/ttyUSB0'}]
        ),
        
        # 2. Kinect Bridge (Bash Wrapper for environment)
        Node(
            package='robot_bringup',
            executable='/usr/bin/bash',
            name='kinect_bridge',
            arguments=['-c', 
                'source /opt/ros/jazzy/setup.bash && '
                'source /home/yasiru/ros2_ws/install/setup.bash && '
                'python3 /home/yasiru/ros2_ws/src/robot_bringup/robot_bringup/kinect_bridge.py'
            ],
            output='screen'
        ),

        # 3. 2D Sensor Stack (Depth to Laser Scan)
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[
                ('depth', '/camera/depth/image_raw'),
                ('depth_camera_info', '/camera/depth/camera_info'),
                ('scan', '/scan')
            ],
            parameters=[{
                'output_frame': 'camera_depth_optical_frame', 
                'range_min': 0.45,
                'range_max': 5.0,
                'scan_height': 10
            }]
        ),

        # 4. SLAM Stack
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[os.path.join(pkg_bringup, 'config', 'mapper_params_online_async.yaml'), {'use_sim_time': False}],
            output='screen'
        ),
        # 5. Nav2 Stack (individual nodes, NO collision_monitor)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[os.path.join(pkg_bringup, 'config', 'navigation_params.yaml')]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[os.path.join(pkg_bringup, 'config', 'navigation_params.yaml')]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[os.path.join(pkg_bringup, 'config', 'navigation_params.yaml')]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[os.path.join(pkg_bringup, 'config', 'navigation_params.yaml')]
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[os.path.join(pkg_bringup, 'config', 'navigation_params.yaml')]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'slam_toolbox',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]
            }]
        ),

        # 5. Zero-Stream Perception (Direct Script Execution)
        Node(
            package='yolo_detection',
            executable='/usr/bin/bash',
            name='yolo_node',
            arguments=['-c', 
                'source /opt/ros/jazzy/setup.bash && '
                'source /home/yasiru/ros2_ws/install/setup.bash && '
                'ULTRALYTICS_CONFIG_DIR=/tmp/ultralytics_config '
                'python3 /home/yasiru/ros2_ws/src/yolo_detection/yolo_detection/yolo_detection_node.py '
                '--ros-args -p model_path:=/home/yasiru/yolov8n.pt -p debug_view:=False -p publish_debug:=False -p inference_fps:=2.0'
            ],
            output='screen'
        ),

        # 6. Semantic Pipeline (Local)
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
        )
    ])
