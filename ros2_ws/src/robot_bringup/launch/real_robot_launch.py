import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 1. Path to your custom Nav2 params
    nav2_params_path = os.path.join(
        get_package_share_directory('my_robot_controller'),
        'config',
        'nav2_params.yaml'
    )

    # 2. Include the standard Nav2 Bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'),
            'launch',
            'bringup_launch.py'
        )),
        launch_arguments={
            'params_file': nav2_params_path,
            'use_sim_time': 'false'
        }.items()
    )

    # 3. Include Kobuki Launch (Nextrones Phase 1)
    kobuki_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('kobuki_node'),
            'launch',
            'kobuki_node-launch.py'
        ))
    )

    return LaunchDescription([
        kobuki_launch,
        nav2_launch
    ])