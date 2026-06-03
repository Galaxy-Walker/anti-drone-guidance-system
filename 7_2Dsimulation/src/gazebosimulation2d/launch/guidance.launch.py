"""2D 定高 PX4/Gazebo 导引桥接节点的 launch 文件。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = PathJoinSubstitution([FindPackageShare("gazebosimulation2d"), "config", "default.yaml"])

    arguments = [
        DeclareLaunchArgument("algorithm", default_value="pn_mppi"),
        DeclareLaunchArgument("scenario", default_value="circle"),
        DeclareLaunchArgument("control_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("pursuer_namespace", default_value="/px4_1"),
        DeclareLaunchArgument("target_namespace", default_value="/px4_2"),
        DeclareLaunchArgument("auto_arm", default_value="true"),
        DeclareLaunchArgument("auto_offboard", default_value="true"),
        DeclareLaunchArgument("offboard_warmup_cycles", default_value="20"),
        DeclareLaunchArgument("sim_time", default_value="40.0"),
        DeclareLaunchArgument("dt", default_value="0.05"),
        DeclareLaunchArgument("pursuer_fixed_altitude", default_value="8.0"),
        DeclareLaunchArgument("target_base_altitude", default_value="1.0"),
        DeclareLaunchArgument("target_start_position_tolerance", default_value="0.75"),
        DeclareLaunchArgument("target_start_velocity_tolerance", default_value="0.75"),
        DeclareLaunchArgument("pursuer_system_id", default_value="1"),
        DeclareLaunchArgument("target_system_id", default_value="2"),
        DeclareLaunchArgument("record_data", default_value="true"),
        DeclareLaunchArgument("record_output_dir", default_value="outputs/gazebo2d"),
    ]

    node = Node(
        package="gazebosimulation2d",
        executable="guidance_node",
        name="guidance_node_2d",
        output="screen",
        parameters=[
            config_file,
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
                "pursuer_fixed_altitude": LaunchConfiguration("pursuer_fixed_altitude"),
                "target_base_altitude": LaunchConfiguration("target_base_altitude"),
                "target_start_position_tolerance": LaunchConfiguration("target_start_position_tolerance"),
                "target_start_velocity_tolerance": LaunchConfiguration("target_start_velocity_tolerance"),
                "pursuer_system_id": LaunchConfiguration("pursuer_system_id"),
                "target_system_id": LaunchConfiguration("target_system_id"),
                "record_data": LaunchConfiguration("record_data"),
                "record_output_dir": LaunchConfiguration("record_output_dir"),
            },
        ],
    )

    return LaunchDescription([*arguments, node])
