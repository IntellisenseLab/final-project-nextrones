from setuptools import find_packages, setup

package_name = 'yolo_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'cv_bridge', 'ultralytics'],
    zip_safe=True,
    maintainer='san',
    maintainer_email='divyangiwas.23@uom.lk',
    description='YOLOv8 Object Detection for Semantic Navigation',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detection_node = yolo_detection.yolo_detection_node:main',
        ],
    },
)
