"""2D 定高 PX4/Gazebo 导引桥接节点的 launch 文件。

默认 `enable_camera:=false`、`vision_source:=off`，只启动原导引节点，启动方式
与加入视觉旁路之前一致。`vision_source:=truth` 时额外启动旁路 `vision_adapter`；
`enable_camera:=true` 时才启动仓库内的 ros_gz_bridge 相机桥接。
QGC、PX4 SITL、Gazebo 和 MicroXRCE-DDS Agent 仍由使用者在外部终端启动。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_file = PathJoinSubstitution([FindPackageShare("gazebosimulation2d"), "config", "default.yaml"])
    camera_bridge_file = PathJoinSubstitution(
        [FindPackageShare("gazebosimulation2d"), "config", "camera_bridge.yaml"]
    )

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
        DeclareLaunchArgument("pursuer_takeoff_position_tolerance", default_value="0.75"),
        DeclareLaunchArgument("pursuer_takeoff_velocity_tolerance", default_value="0.75"),
        DeclareLaunchArgument("pursuer_system_id", default_value="1"),
        DeclareLaunchArgument("target_system_id", default_value="2"),
        DeclareLaunchArgument("record_data", default_value="true"),
        DeclareLaunchArgument("record_output_dir", default_value="outputs/gazebo2d"),
        DeclareLaunchArgument("debug_log", default_value="false"),
        DeclareLaunchArgument("debug_log_period_s", default_value="0.2"),
        DeclareLaunchArgument("startup_log_period_s", default_value="1.0"),
        # 视觉旁路：默认全部关闭。enable_camera 只控制桥接，不控制 Gazebo 相机渲染。
        DeclareLaunchArgument("enable_camera", default_value="false"),
        DeclareLaunchArgument("vision_source", default_value="off"),
        DeclareLaunchArgument("camera_frame_id", default_value="camera_link_optical"),
        DeclareLaunchArgument("camera_mount_xyz", default_value="[0.0, 0.0, 0.10]"),
        DeclareLaunchArgument("camera_mount_rpy_deg", default_value="[0.0, 90.0, 0.0]"),
        DeclareLaunchArgument("truth_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("pose_timeout_s", default_value="0.2"),
        DeclareLaunchArgument("pose_pair_tolerance_s", default_value="0.05"),
        DeclareLaunchArgument("pixel_noise_px", default_value="3.0"),
        DeclareLaunchArgument("target_plane_sigma_m", default_value="0.1"),
        DeclareLaunchArgument("vision_record_data", default_value="true"),
        DeclareLaunchArgument("vision_record_output_dir", default_value="outputs/gazebo2d_vision"),
        # 视觉节点参数名与导引节点重名，launch 参数用 vision_ 前缀区分，避免相互覆盖。
        DeclareLaunchArgument("vision_debug_log", default_value="false"),
        DeclareLaunchArgument("vision_debug_log_period_s", default_value="0.5"),
    ]

    guidance_node = Node(
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
                "pursuer_takeoff_position_tolerance": LaunchConfiguration("pursuer_takeoff_position_tolerance"),
                "pursuer_takeoff_velocity_tolerance": LaunchConfiguration("pursuer_takeoff_velocity_tolerance"),
                "pursuer_system_id": LaunchConfiguration("pursuer_system_id"),
                "target_system_id": LaunchConfiguration("target_system_id"),
                "record_data": LaunchConfiguration("record_data"),
                "record_output_dir": LaunchConfiguration("record_output_dir"),
                "debug_log": LaunchConfiguration("debug_log"),
                "debug_log_period_s": LaunchConfiguration("debug_log_period_s"),
                "startup_log_period_s": LaunchConfiguration("startup_log_period_s"),
            },
        ],
    )

    # 仓库内固定桥接；条件为 false 时不解析 ros_gz_bridge 资源或 config_file。
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        output="screen",
        parameters=[{"config_file": camera_bridge_file}],
        condition=IfCondition(LaunchConfiguration("enable_camera")),
    )

    # vision_source=truth 时启动旁路适配器；off 时不创建条件节点。
    vision_enabled = PythonExpression(["'", LaunchConfiguration("vision_source"), "' != 'off'"])
    vision_adapter = Node(
        package="gazebosimulation2d",
        executable="vision_adapter",
        name="vision_adapter",
        output="screen",
        parameters=[
            config_file,
            {
                "vision_source": LaunchConfiguration("vision_source"),
                "pursuer_namespace": LaunchConfiguration("pursuer_namespace"),
                "target_namespace": LaunchConfiguration("target_namespace"),
                "camera_frame_id": LaunchConfiguration("camera_frame_id"),
                "target_base_altitude": LaunchConfiguration("target_base_altitude"),
                "camera_mount_xyz": LaunchConfiguration("camera_mount_xyz"),
                "camera_mount_rpy_deg": LaunchConfiguration("camera_mount_rpy_deg"),
                "truth_rate_hz": LaunchConfiguration("truth_rate_hz"),
                "pose_timeout_s": LaunchConfiguration("pose_timeout_s"),
                "pose_pair_tolerance_s": LaunchConfiguration("pose_pair_tolerance_s"),
                "pixel_noise_px": LaunchConfiguration("pixel_noise_px"),
                "target_plane_sigma_m": LaunchConfiguration("target_plane_sigma_m"),
                "vision_record_data": LaunchConfiguration("vision_record_data"),
                "vision_record_output_dir": LaunchConfiguration("vision_record_output_dir"),
                "debug_log": LaunchConfiguration("vision_debug_log"),
                "debug_log_period_s": LaunchConfiguration("vision_debug_log_period_s"),
            },
        ],
        condition=IfCondition(vision_enabled),
    )

    return LaunchDescription([*arguments, guidance_node, camera_bridge, vision_adapter])
