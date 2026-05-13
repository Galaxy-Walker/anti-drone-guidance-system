from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    config_file = PathJoinSubstitution([FindPackageShare("gazebosimulation"), "config", "default.yaml"])

    arguments = [
        DeclareLaunchArgument("algorithm", default_value="pn_fov_mppi"),
        DeclareLaunchArgument("scenario", default_value="circle"),
        DeclareLaunchArgument("control_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("pursuer_namespace", default_value="/px4_1"),
        DeclareLaunchArgument("target_namespace", default_value="/px4_2"),
        DeclareLaunchArgument("auto_arm", default_value="true"),
        DeclareLaunchArgument("auto_offboard", default_value="true"),
        DeclareLaunchArgument("offboard_warmup_cycles", default_value="20"),
        DeclareLaunchArgument("sim_time", default_value="40.0"),
        DeclareLaunchArgument("dt", default_value="0.05"),
        DeclareLaunchArgument("pursuer_system_id", default_value="1"),
        DeclareLaunchArgument("target_system_id", default_value="2"),
    ]

    node = Node(
        package="gazebosimulation",
        executable="guidance_node",
        name="guidance_node",
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
                "pursuer_system_id": LaunchConfiguration("pursuer_system_id"),
                "target_system_id": LaunchConfiguration("target_system_id"),
            },
        ],
    )

    return LaunchDescription([*arguments, node])
