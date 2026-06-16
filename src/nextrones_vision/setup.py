from setuptools import find_packages, setup

package_name = 'nextrones_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yasiru',
    maintainer_email='bandarakgpy.23@uom.lk',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detection_node = nextrones_vision.yolo_detection_node:main',
            'kinect_driver_node = nextrones_vision.kinect_driver_node:main',
        ],
    },
)
