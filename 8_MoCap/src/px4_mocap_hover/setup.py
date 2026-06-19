from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'px4_mocap_hover'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description=(
        'PX4 ROS 2 motion-capture takeoff, trajectory, and landing missions.'
    ),
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mocap_arm_test = px4_mocap_hover.arm_test_node:main',
            'mocap_bridge = px4_mocap_hover.mocap_bridge_node:main',
            'mocap_circle = px4_mocap_hover.circle_node:main',
            'mocap_hover = px4_mocap_hover.node:main',
            (
                'mocap_trajectory_replay = '
                'px4_mocap_hover.trajectory_replay_node:main'
            ),
            (
                'mocap_trajectory_recorder = '
                'px4_mocap_hover.mocap_trajectory_recorder_node:main'
            ),
            'mocap_tracker = px4_mocap_hover.tracker_node:main',
        ],
    },
)
