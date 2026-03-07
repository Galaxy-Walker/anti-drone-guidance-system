"""
比例导引节点启动文件

用法:
  # 使用默认参数（仿真模式）
  ros2 launch ros2_guidance_system pn_guidance_launch.py

  # 使用自定义配置文件
  ros2 launch ros2_guidance_system pn_guidance_launch.py config_file:=/path/to/custom.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_default_config_path() -> str:
    """优先使用 src 下的配置文件；若不存在则回退到 install/share 下配置。"""
    pkg_dir = get_package_share_directory('ros2_guidance_system')
    install_config = os.path.join(pkg_dir, 'config', 'pn_guidance_params.yaml')

    marker = '/install/'
    if marker in pkg_dir:
        workspace_root = pkg_dir.split(marker, 1)[0]
        src_config = os.path.join(
            workspace_root,
            'src',
            'ros2_guidance_system',
            'config',
            'pn_guidance_params.yaml',
        )
        if os.path.exists(src_config):
            return src_config

    return install_config


def generate_launch_description():
    default_config = _resolve_default_config_path()

    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='参数配置文件路径'
    )


    # 创建节点
    pn_guidance_node = Node(
        package='ros2_guidance_system',
        executable='pn_guidance_node',
        name='pn_guidance_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
        ],
    )

    return LaunchDescription([
        config_file_arg,
        pn_guidance_node,
    ])
