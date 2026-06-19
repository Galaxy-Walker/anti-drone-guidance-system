"""Launch guarded replay of an absolute-NED trajectory CSV."""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node


def prompt_trajectory_file():
    """Prompt until the user provides an existing trajectory CSV file."""
    while True:
        try:
            value = input(
                '请输入要回放的 CSV 文件路径，然后按 Enter：\n> '
            ).strip()
        except EOFError as error:
            raise RuntimeError(
                '无法从终端读取 CSV 文件路径；请在交互式终端中启动'
            ) from error

        if not value:
            print('CSV 文件路径不能为空，请重新输入。')
            continue

        path = Path(value).expanduser()
        if not path.is_file():
            print(f'文件不存在或不是普通文件：{path}')
            continue

        return str(path.resolve())


def generate_launch_description():
    """Build the launch description."""
    trajectory_file = prompt_trajectory_file()
    return LaunchDescription([
        Node(
            package='px4_mocap_hover',
            executable='mocap_trajectory_replay',
            name='px4_mocap_trajectory_replay',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'mocap_topic': '/fmu/in/vehicle_visual_odometry',
                'trajectory_file': trajectory_file,
                'takeoff_height': 1.2,
                'hover_duration': 3.0,
                'mocap_timeout': 0.5,
                'local_position_timeout': 0.5,
                'control_rate': 60.0,
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
