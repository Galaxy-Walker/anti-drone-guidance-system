"""Launch the standalone VRPN-to-PX4 motion-capture bridge."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description."""
    return LaunchDescription([
        Node(
            package='px4_mocap_hover',
            executable='mocap_bridge',
            name='px4_mocap_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'mocap_topic': '/vrpn_mocap/RigidBody/pose',
                'px4_topic': '/fmu/in/vehicle_visual_odometry',
                'position_variance': 0.0025,
                'orientation_variance': 0.01,
                'quality': 100,
            }],
        ),
    ])
