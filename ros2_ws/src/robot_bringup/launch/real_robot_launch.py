import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    target_object_arg = DeclareLaunchArgument(
        'target_object',
        default_value='bottle',
        description='Target object for semantic navigation'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    target_object = LaunchConfiguration('target_object')
    use_rviz = LaunchConfiguration('rviz')

    # Paths
    bringup_dir = get_package_share_directory('robot_bringup')
    rviz_config_path = os.path.join(bringup_dir, 'config', 'semantic_navigation.rviz')
    
    # Parameters
    # Using nav2_params.yaml from robot_bringup/config instead of the missing my_robot_controller
    nav2_params_path = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
    
    # 1. Nav2 Bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py'
        )),
        launch_arguments={
            'params_file': nav2_params_path,
            'use_sim_time': 'false'
        }.items()
    )

    # 2. Kobuki Base
    kobuki_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('kobuki_node'), 'launch', 'kobuki_node-launch.py'
        ))
    )

    # 3. Perception & Semantic Pipeline
    yolo_node = Node(
        package='yolo_detection',
        executable='yolo_detection_node',
        name='yolo_detection_node',
        parameters=[{'model_path': '/home/hasini/yolov8n.pt', 'debug_view': False}]
    )

    localization_node = Node(
        package='object_localization',
        executable='object_localization_node',
        name='object_localization_node'
    )

    semantic_map_node = Node(
        package='semantic_map',
        executable='semantic_map_node',
        name='semantic_map_node'
    )

    nav_goal_sender_node = Node(
        package='nav_goal_sender',
        executable='nav_goal_sender_node',
        name='nav_goal_sender_node',
        parameters=[{'target_object': target_object}]
    )

    # 4. Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        # Arguments
        target_object_arg,
        use_rviz_arg,

        # Hardware & Navigation
        kobuki_launch,
        nav2_launch,
        
        # Semantic Pipeline
        yolo_node,
        localization_node,
        semantic_map_node,
        nav_goal_sender_node,

        # RViz
        rviz_node
    ])