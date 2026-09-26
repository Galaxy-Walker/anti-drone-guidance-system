"""2D 定高 PX4/Gazebo 导引桥接节点的 launch 文件。

默认 `enable_camera:=false`、`vision_source:=off`、`target_source:=odometry`，
只启动原导引节点，启动方式与加入视觉闭环之前一致。

- `enable_camera:=true` 启动仓库内的 ros_gz_bridge 相机桥接（含 `/clock`）；
- `vision_source:=truth` 只额外启动 `vision_adapter`（odometry 伪检测旁路）；
- `vision_source:=yolo` 启动 `vision_detector` + `vision_adapter`，需要
  `yolo_python` / `yolo_model_path` 和 `use_sim_time:=true`；
- `target_source:=vision` 让导引节点消费 `/vision/target_pose`。

QGC、PX4 SITL、Gazebo 和 MicroXRCE-DDS Agent 仍由使用者在外部终端启动。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
        # 视觉闭环通用参数：视觉链路三节点统一 use_sim_time，/clock 由相机桥接提供。
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_camera", default_value="false"),
        DeclareLaunchArgument("vision_source", default_value="off"),
        DeclareLaunchArgument("target_source", default_value="odometry"),
        DeclareLaunchArgument("vision_topic", default_value="/vision/target_pose"),
        DeclareLaunchArgument("vision_fallback", default_value="none"),
        DeclareLaunchArgument("vision_max_age_s", default_value="0.5"),
        DeclareLaunchArgument("vision_alpha", default_value="0.85"),
        DeclareLaunchArgument("vision_beta", default_value="0.25"),
        DeclareLaunchArgument("vision_accel_tau_s", default_value="0.5"),
        DeclareLaunchArgument("vision_gate_sigma", default_value="0.0"),
        DeclareLaunchArgument("vision_coast_s", default_value="0.3"),
        DeclareLaunchArgument("vision_loss_s", default_value="1.0"),
        DeclareLaunchArgument("vision_hold_on_loss", default_value="true"),
        DeclareLaunchArgument("min_dt_s", default_value="0.01"),
        DeclareLaunchArgument("max_dt_s", default_value="0.5"),
        # vision_adapter 参数：truth/yolo 共用；与导引节点重名的参数加 vision_ 前缀。
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
        DeclareLaunchArgument("vision_debug_log", default_value="false"),
        DeclareLaunchArgument("vision_debug_log_period_s", default_value="0.5"),
        DeclareLaunchArgument("min_score", default_value="0.25"),
        DeclareLaunchArgument("pose_match_tolerance_s", default_value="0.05"),
        DeclareLaunchArgument("pose_cache_max_age_s", default_value="0.5"),
        DeclareLaunchArgument("pose_cache_interpolate", default_value="true"),
        DeclareLaunchArgument("extrapolate_pose", default_value="false"),
        DeclareLaunchArgument("record_dataset", default_value="false"),
        DeclareLaunchArgument("vision_dataset_output_dir", default_value="outputs/gazebo2d_vision/dataset"),
        DeclareLaunchArgument("dataset_label_box_size_m", default_value="0.35"),
        # vision_detector 参数：launch 参数统一加 yolo_ 前缀，避免与导引/适配节点重名。
        DeclareLaunchArgument("yolo_python", default_value=""),
        DeclareLaunchArgument("yolo_worker_script", default_value=""),
        DeclareLaunchArgument("yolo_model_path", default_value=""),
        DeclareLaunchArgument("yolo_model_fallback_path", default_value=""),
        DeclareLaunchArgument("yolo_imgsz", default_value="640"),
        DeclareLaunchArgument("yolo_conf", default_value="0.25"),
        DeclareLaunchArgument("yolo_iou", default_value="0.7"),
        DeclareLaunchArgument("yolo_device", default_value="0"),
        DeclareLaunchArgument("yolo_half", default_value="true"),
        DeclareLaunchArgument("yolo_max_det", default_value="5"),
        DeclareLaunchArgument("yolo_class_name", default_value="uav"),
        DeclareLaunchArgument("yolo_process_hz", default_value="10.0"),
        DeclareLaunchArgument("yolo_max_frame_age_s", default_value="0.2"),
        DeclareLaunchArgument("yolo_frame_format", default_value="raw"),
        DeclareLaunchArgument("yolo_inference_timeout_s", default_value="1.0"),
        DeclareLaunchArgument("yolo_worker_startup_timeout_s", default_value="30.0"),
        DeclareLaunchArgument("yolo_worker_restart_limit", default_value="3"),
        DeclareLaunchArgument("yolo_save_frame_hz", default_value="0.0"),
        DeclareLaunchArgument("yolo_stats_csv", default_value="outputs/gazebo2d_vision/yolo_detections.csv"),
        DeclareLaunchArgument("yolo_debug_log", default_value="false"),
        DeclareLaunchArgument("yolo_debug_log_period_s", default_value="1.0"),
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
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "target_source": LaunchConfiguration("target_source"),
                "vision_topic": LaunchConfiguration("vision_topic"),
                "vision_fallback": LaunchConfiguration("vision_fallback"),
                "vision_max_age_s": LaunchConfiguration("vision_max_age_s"),
                "vision_alpha": LaunchConfiguration("vision_alpha"),
                "vision_beta": LaunchConfiguration("vision_beta"),
                "vision_accel_tau_s": LaunchConfiguration("vision_accel_tau_s"),
                "vision_gate_sigma": LaunchConfiguration("vision_gate_sigma"),
                "vision_coast_s": LaunchConfiguration("vision_coast_s"),
                "vision_loss_s": LaunchConfiguration("vision_loss_s"),
                "vision_hold_on_loss": LaunchConfiguration("vision_hold_on_loss"),
                "min_dt_s": LaunchConfiguration("min_dt_s"),
                "max_dt_s": LaunchConfiguration("max_dt_s"),
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

    # vision_source != off 时启动适配器；truth 用伪检测，yolo 消费 /camera/detections。
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
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "min_score": LaunchConfiguration("min_score"),
                "pose_match_tolerance_s": LaunchConfiguration("pose_match_tolerance_s"),
                "pose_cache_max_age_s": LaunchConfiguration("pose_cache_max_age_s"),
                "pose_cache_interpolate": LaunchConfiguration("pose_cache_interpolate"),
                "extrapolate_pose": LaunchConfiguration("extrapolate_pose"),
                "record_dataset": LaunchConfiguration("record_dataset"),
                "dataset_output_dir": LaunchConfiguration("vision_dataset_output_dir"),
                "dataset_label_box_size_m": LaunchConfiguration("dataset_label_box_size_m"),
            },
        ],
        condition=IfCondition(vision_enabled),
    )

    # vision_source=yolo 时启动检测节点；truth/off 不创建。
    yolo_enabled = PythonExpression(["'", LaunchConfiguration("vision_source"), "' == 'yolo'"])
    vision_detector = Node(
        package="gazebosimulation2d",
        executable="vision_detector",
        name="vision_detector",
        output="screen",
        parameters=[
            config_file,
            {
                "camera_frame_id": LaunchConfiguration("camera_frame_id"),
                "yolo_python": LaunchConfiguration("yolo_python"),
                "yolo_worker_script": LaunchConfiguration("yolo_worker_script"),
                "model_path": LaunchConfiguration("yolo_model_path"),
                "model_fallback_path": LaunchConfiguration("yolo_model_fallback_path"),
                "imgsz": LaunchConfiguration("yolo_imgsz"),
                "conf": LaunchConfiguration("yolo_conf"),
                "iou": LaunchConfiguration("yolo_iou"),
                # launch 会按 YAML 规则把 "0" 解析成 int；device 是字符串参数，这里强制成 str。
                "device": ParameterValue(LaunchConfiguration("yolo_device"), value_type=str),
                "half": LaunchConfiguration("yolo_half"),
                "max_det": LaunchConfiguration("yolo_max_det"),
                "class_name": LaunchConfiguration("yolo_class_name"),
                "process_hz": LaunchConfiguration("yolo_process_hz"),
                "max_frame_age_s": LaunchConfiguration("yolo_max_frame_age_s"),
                "frame_format": LaunchConfiguration("yolo_frame_format"),
                "inference_timeout_s": LaunchConfiguration("yolo_inference_timeout_s"),
                "worker_startup_timeout_s": LaunchConfiguration("yolo_worker_startup_timeout_s"),
                "worker_restart_limit": LaunchConfiguration("yolo_worker_restart_limit"),
                "save_frame_hz": LaunchConfiguration("yolo_save_frame_hz"),
                "dataset_output_dir": LaunchConfiguration("vision_dataset_output_dir"),
                "stats_csv": LaunchConfiguration("yolo_stats_csv"),
                "debug_log": LaunchConfiguration("yolo_debug_log"),
                "debug_log_period_s": LaunchConfiguration("yolo_debug_log_period_s"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        condition=IfCondition(yolo_enabled),
    )

    return LaunchDescription(
        [*arguments, guidance_node, camera_bridge, vision_adapter, vision_detector]
    )
