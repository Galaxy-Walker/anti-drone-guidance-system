"""Launch the target motion-capture trajectory CSV recorder."""

from launch import LaunchDescription
from launch_ros.actions import Node


def prompt_mocap_topic():
    """Prompt until the user selects a supported motion-capture topic."""
    topics = {
        '1': '/vrpn_mocap/RigidBody_002/pose',
        '2': '/vrpn_mocap/RigidBody/pose',
    }
    while True:
        try:
            selection = input(
                '请选择要记录的动捕话题：\n'
                '1: /vrpn_mocap/RigidBody_002/pose\n'
                '2: /vrpn_mocap/RigidBody/pose\n'
                '> '
            ).strip()
        except EOFError as error:
            raise RuntimeError(
                '无法从终端读取动捕话题选择；请在交互式终端中启动'
            ) from error

        mocap_topic = topics.get(selection)
        if mocap_topic is not None:
            return mocap_topic

        print('输入无效，请输入 1 或 2。')


def generate_launch_description():
    """Build the launch description."""
    mocap_topic = prompt_mocap_topic()
    return LaunchDescription([
        Node(
            package='px4_mocap_hover',
            executable='mocap_trajectory_recorder',
            name='mocap_trajectory_recorder',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'mocap_topic': mocap_topic,
                'output_directory': '/home/nvidia/ws_ros2/trajectory_csv',
                'file_prefix': 'target_trajectory',
            }],
        ),
    ])
