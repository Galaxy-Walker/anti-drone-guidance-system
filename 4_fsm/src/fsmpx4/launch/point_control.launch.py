#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('fsmpx4')
    config_file = os.path.join(pkg_share, 'config', 'fsm.yaml')

    return LaunchDescription([
        Node(
            package='fsmpx4',
            executable='point_control_node',
            name='px4ctrl_fsm',
            output='screen',
            parameters=[config_file]
        )
    ])
