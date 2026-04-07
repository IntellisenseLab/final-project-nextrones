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
        
        # 1. Include all launch files so ros2 launch can find them
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        
        # 2. Include your Nav2 params and other configuration files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        
        # 3. Include the test scripts folder added by your friend
        (os.path.join('share', package_name, 'scripts'), glob(os.path.join('scripts', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='san',
    maintainer_email='divyangiwas.23@uom.lk',
    description='Robot bringup and configuration for Semantic Navigation project - Nextrones',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # If you write a python node here later, add it like this:
            # 'test_node = robot_bringup.test_node:main',
        ],
    },
)