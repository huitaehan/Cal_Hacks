import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Huitae Han',
    maintainer_email='huitae@calhacks.local',
    description='Computer vision, victim identification, and sensor processing for Cal Hacks',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'vision_node = robot_vision.vision_node:main',
            'visual_wall_detector = robot_vision.visual_wall_detector:main',
            'mock_camera = robot_vision.mock_camera_publisher:main',
            'ultrasonic_node = robot_vision.ultrasonic_node:main',
            'pi_streamer = robot_vision.pi_client:main',
            'test_camera = robot_vision.test_camera:main',
        ],
    },
)
