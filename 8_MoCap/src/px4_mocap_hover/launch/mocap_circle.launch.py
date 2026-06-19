"""Launch the PX4 motion-capture circle mission."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description."""
    return LaunchDescription([
        Node(
            package='px4_mocap_hover',
            executable='mocap_circle',
            name='px4_mocap_circle',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'mocap_topic': '/fmu/in/vehicle_visual_odometry',
                'takeoff_height': 1.0,
                'hover_duration': 3.0,
                'circle_radius': 0.5,
                'circle_duration': 10.0,
                'mocap_timeout': 0.5,
                'local_position_timeout': 0.5,
                'control_rate': 10.0,
                'log_rate': 1.0,
                'prearm_stable_duration': 10.0,
                'prearm_position_tolerance': 0.15,
                'position_tolerance': 0.15,
                'stable_duration': 1.0,
                'prestream_duration': 1.0,
                'command_retry_interval': 1.0,
                'required_mocap_samples': 20,
            }],
        ),
    ])
