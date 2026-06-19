"""Launch the PX4 motion-capture horizontal tracking mission."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description."""
    return LaunchDescription([
        Node(
            package='px4_mocap_hover',
            executable='mocap_tracker',
            name='px4_mocap_tracker',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'self_odometry_topic':
                    '/fmu/in/vehicle_visual_odometry',
                'target_mocap_topic':
                    '/vrpn_mocap/RigidBody_002/pose',
                'takeoff_height': 1.2,
                'mocap_timeout': 0.5,
                'target_mocap_timeout': 0.5,
                'local_position_timeout': 0.5,
                'target_reacquire_samples': 5,
                'xy_kp': 0.8,
                'xy_deadband': 0.10,
                'max_xy_speed': 0.5,
                'max_xy_acceleration': 0.5,
                'z_kp': 1.0,
                'max_z_speed': 0.3,
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
