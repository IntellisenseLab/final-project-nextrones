import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. Include all launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        
        # 2. Include all config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.rviz'))),

        # 3. Include URDF
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        
        # 4. Include scripts
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*'))),
    ],
    install_requires=['setuptools', 'pyserial', 'numpy'],
    zip_safe=True,
    maintainer='san',
    maintainer_email='divyangiwas.23@uom.lk',
    description='Semantic Navigation Robot Bringup',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kobuki_driver = robot_bringup.kobuki_driver:main',
            'kinect_bridge = robot_bringup.kinect_bridge:main',
            'yolo_detection_node = robot_bringup.yolo_detection_node:main',
            'task_manager = robot_bringup.task_manager_node:main',
            'cmd_vel_test = robot_bringup.cmd_vel_test:main',
            'system_monitor = robot_bringup.system_monitor_node:main',
            'visual_tracker = robot_bringup.visual_tracker_node:main',
        ],

    },
)