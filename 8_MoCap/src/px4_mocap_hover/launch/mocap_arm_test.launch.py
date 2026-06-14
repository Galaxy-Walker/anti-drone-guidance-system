"""Launch the PX4 arm-only test."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description."""
    return LaunchDescription([
        Node(
            package='px4_mocap_hover',
            executable='mocap_arm_test',
            name='px4_mocap_arm_test',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'mocap_topic': '/fmu/in/vehicle_visual_odometry',
                'mocap_timeout': 0.5,
                'control_rate': 10.0,
                'log_rate': 1.0,
                'command_retry_interval': 1.0,
                'required_mocap_samples': 20,
                'armed_duration': 3.0,
                'arm_request_timeout': 10.0,
            }],
        ),
    ])
