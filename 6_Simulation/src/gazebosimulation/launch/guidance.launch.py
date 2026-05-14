"""双机 PX4/Gazebo 导引桥接节点的 launch 文件。

该 launch 文件先加载 `config/default.yaml` 中的包默认值，再用 launch 参数覆盖
指定字段。这样既能把通用默认值保存在 YAML 中，也能从命令行快速切换算法、
场景或 namespace。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    """创建 launch 参数并启动 `gazebosimulation.guidance_node`。"""
    # 这里使用安装后的 package share 路径，而不是源码路径；因此 setup.py 必须把
    # config/default.yaml 安装到 share/gazebosimulation/config。
    config_file = PathJoinSubstitution([FindPackageShare("gazebosimulation"), "config", "default.yaml"])

    arguments = [
        # 算法和场景选项会由节点根据 pythonsimulation.config.ALGORITHMS/SCENARIOS 校验。
        DeclareLaunchArgument("algorithm", default_value="pn_fov_mppi"),
        DeclareLaunchArgument("scenario", default_value="circle"),

        # namespace 必须匹配 SITL 创建的两个 PX4 uXRCE-DDS 话题 namespace；默认假设
        # `/px4_1` 是追踪机，`/px4_2` 是目标机。
        DeclareLaunchArgument("control_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("pursuer_namespace", default_value="/px4_1"),
        DeclareLaunchArgument("target_namespace", default_value="/px4_2"),

        # 调试时可以关闭这些自动命令，改为在 QGroundControl 中手动解锁和切换模式。
        DeclareLaunchArgument("auto_arm", default_value="true"),
        DeclareLaunchArgument("auto_offboard", default_value="true"),
        DeclareLaunchArgument("offboard_warmup_cycles", default_value="20"),

        # sim_time 和 dt 会传入 SimulationConfig，使 Gazebo 行为使用与离线 Python 对比仿真
        # 相同的时间假设。
        DeclareLaunchArgument("sim_time", default_value="40.0"),
        DeclareLaunchArgument("dt", default_value="0.05"),

        # MAVLink system id 必须匹配接收 VehicleCommand 的两个 PX4 实例。
        DeclareLaunchArgument("pursuer_system_id", default_value="1"),
        DeclareLaunchArgument("target_system_id", default_value="2"),

        # 记录 Gazebo/PX4 话题数据；节点只保存 CSV，图片由普通 Python 脚本后处理。
        DeclareLaunchArgument("record_data", default_value="true"),
        DeclareLaunchArgument("record_output_dir", default_value="outputs/gazebo"),
    ]

    node = Node(
        package="gazebosimulation",
        executable="guidance_node",
        name="guidance_node",
        output="screen",
        parameters=[
            config_file,
            # LaunchConfiguration 会覆盖上面的 YAML 默认值，因此 `algorithm:=pn` 等命令行
            # 参数无需修改 YAML 就能生效。
            {
                "algorithm": LaunchConfiguration("algorithm"),
                "scenario": LaunchConfiguration("scenario"),
                "control_rate_hz": LaunchConfiguration("control_rate_hz"),
                "pursuer_namespace": LaunchConfiguration("pursuer_namespace"),
                "target_namespace": LaunchConfiguration("target_namespace"),
                "auto_arm": LaunchConfiguration("auto_arm"),
                "auto_offboard": LaunchConfiguration("auto_offboard"),
                "offboard_warmup_cycles": LaunchConfiguration("offboard_warmup_cycles"),
                "sim_time": LaunchConfiguration("sim_time"),
                "dt": LaunchConfiguration("dt"),
                "pursuer_system_id": LaunchConfiguration("pursuer_system_id"),
                "target_system_id": LaunchConfiguration("target_system_id"),
                "record_data": LaunchConfiguration("record_data"),
                "record_output_dir": LaunchConfiguration("record_output_dir"),
            },
        ],
    )

    return LaunchDescription([*arguments, node])
