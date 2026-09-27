"""视觉量测适配节点：把图像检测/truth 参考投影成 `/vision/target_pose`。

支持三种 `vision_source`：

- `off`：不启动（launch 不会创建该节点）。
- `truth`：不订阅图像；用目标 odometry 参考位置投影成伪检测，再由同一反投影链路
  生成量测（P3 旁路，行为与旧版一致）。
- `yolo`：订阅 `/camera/detections`（`vision_detector` 的输出），按图像 `header.stamp`
  在追踪机位姿缓存里插值取出相机位姿，再把 bbox 中心反投影到固定目标平面。

时间语义（`yolo` 模式）：

- 视觉链路统一 `use_sim_time=true`；位姿缓存键是**收到 odometry 时的仿真时间**
  （不假设 PX4 `timestamp_sample` 与 Gazebo/ROS 时钟同源），量测按图像 stamp 插值；
- 位姿缓存超出 `pose_match_tolerance_s`、缓存样本超出 `pose_cache_max_age_s` 或图像
  落在缓存两端之外（future）时拒绝量测并记录原因，不允许静默退回“最新位姿”；
- 残余同步误差（odometry 接收时刻 ≠ 采样时刻）在 `pose_match_dt_ms` 中量化。

数据集录制（`record_dataset`，默认关闭）供 P3 门槛评估与 P7 微调：`vision_detector`
按 `save_frame_hz` 落盘 `frames/<stamp_ns>.jpg`，本节点在真值投影有效时落盘
`labels/<stamp_ns>.txt`（YOLO 归一化格式，目标盒假设 0.35 m 立方体并加 8% margin）。
"""

from __future__ import annotations

import bisect
import csv
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def _ensure_pythonsimulation2d_on_path() -> None:
    """开发阶段未安装包时，让同级 `pythonsimulation2d` 可导入。"""
    for parent in Path(__file__).resolve().parents:
        src_candidate = parent / "src"
        if (src_candidate / "pythonsimulation2d").is_dir():
            sys.path.insert(0, str(src_candidate))
            return
        if (parent / "pythonsimulation2d").is_dir():
            sys.path.insert(0, str(parent))
            return


_ensure_pythonsimulation2d_on_path()


import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import VehicleOdometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from pythonsimulation2d.camera_geometry import (
    CameraIntrinsics,
    CameraPose,
    ground_to_pixel,
    pixel_to_ground,
    validate_intrinsics,
)

from gazebosimulation2d.coordinates import camera_pose_from_odometry
from gazebosimulation2d.recording_paths import resolve_recording_path
from gazebosimulation2d.sim_clock import SimClockGuard, create_sim_clock_guard_timer

# 伪检测的 bbox 只是像素级占位，不冒充真实目标框尺寸；中心才是有效信息。
TRUTH_BBOX_SIZE_PX = 4.0

# 本轮不提供姿态观测，用大而有限的方差明确表示“未知”，而不是全零。
POSE_UNKNOWN_VARIANCE = 1.0e6

# off 不应创建节点；truth/yolo 是两种实际运行模式。
SUPPORTED_VISION_SOURCES = ("off", "truth", "yolo")

# 目标盒假设：Det-Fly/仿真目标约 0.35 m 宽（计划 §3.3），仅用于离线标注。
DEFAULT_LABEL_BOX_SIZE_M = 0.35
# 标注框在真值 AABB 基础上加 8% margin（每边 4%）。
LABEL_MARGIN_FRACTION = 0.08

CSV_FIELDS = (
    # 旧字段：truth 行语义不变，yolo 行复用同一批诊断列。
    "elapsed_s",
    "stamp_s",
    "valid",
    "invalid_reason",
    "pursuer_timestamp_sample_us",
    "target_timestamp_sample_us",
    "pursuer_age_ms",
    "target_age_ms",
    "pose_pair_delta_ms",
    "u_ref",
    "v_ref",
    "target_x_est",
    "target_y_est",
    "target_x_ref",
    "target_y_ref",
    "target_z_odom",
    "target_plane_z",
    "position_roundtrip_error_m",
    # 视觉链路新增字段；truth 模式全部为 NaN。
    "source",
    "image_stamp_s",
    "detection_age_ms",
    "pose_match_dt_ms",
    "pose_interpolated",
    "score",
    "bbox_w",
    "bbox_h",
    "n_detections",
    "pixel_error_vs_truth_px",
    "position_error_vs_odom_m",
)


@dataclass(slots=True)
class _PoseSample:
    """位姿缓存条目；时间键是收到 odometry 时的仿真时间。"""

    t_sim_ns: int
    position_ned: np.ndarray
    quaternion_wxyz: np.ndarray
    velocity_ned: np.ndarray


@dataclass(slots=True)
class _SampleRecord:
    """一次量测结果；无效时用 `invalid_reason` 说明拒绝原因。"""

    stamp_ns: int
    elapsed_s: float
    source: str = "truth"
    valid: bool = False
    invalid_reason: str = ""
    pursuer_timestamp_sample_us: int | None = None
    target_timestamp_sample_us: int | None = None
    pursuer_age_ms: float = math.nan
    target_age_ms: float = math.nan
    pose_pair_delta_ms: float = math.nan
    u_ref: float = math.nan
    v_ref: float = math.nan
    target_x_est: float = math.nan
    target_y_est: float = math.nan
    target_x_ref: float = math.nan
    target_y_ref: float = math.nan
    target_z_odom: float = math.nan
    target_plane_z: float = math.nan
    position_roundtrip_error_m: float = math.nan
    image_stamp_s: float = math.nan
    detection_age_ms: float = math.nan
    pose_match_dt_ms: float = math.nan
    pose_interpolated: float = math.nan
    score: float = math.nan
    bbox_w: float = math.nan
    bbox_h: float = math.nan
    n_detections: float = math.nan
    pixel_error_vs_truth_px: float = math.nan
    position_error_vs_odom_m: float = math.nan


def _select_drone_detection(message: Detection2DArray) -> tuple[Detection2D, float] | None:
    """按契约挑选有效 drone 检测中分数最高的一条，返回 `(detection, score)`。"""
    best: Detection2D | None = None
    best_score = 0.0
    for detection in message.detections:
        for result in detection.results:
            if result.hypothesis.class_id != "drone":
                continue
            score = float(result.hypothesis.score)
            if not math.isfinite(score) or not 0.0 <= score <= 1.0:
                continue
            if best is None or score > best_score:
                best = detection
                best_score = score
    if best is None:
        return None
    return best, best_score


def _slerp_quaternion(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    """四元数最短弧插值（输入/输出均为 `[w, x, y, z]`）。"""
    a = np.asarray(q0, dtype=float)[:4]
    b = np.asarray(q1, dtype=float)[:4]
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b = -b
        dot = -dot
    if dot > 0.9995:
        result = a + alpha * (b - a)
        return result / np.linalg.norm(result)
    theta = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta = math.sin(theta)
    return (math.sin((1.0 - alpha) * theta) * a + math.sin(alpha * theta) * b) / sin_theta


class VisionAdapter(Node):
    """订阅相机内参/检测/两机 odometry，发布独立的位置量测。"""

    def __init__(self, parameter_overrides: list[Parameter] | None = None) -> None:
        super().__init__("vision_adapter", parameter_overrides=parameter_overrides)
        self._declare_parameters()
        self._load_parameters()

        self._start_mono_ns = time.monotonic_ns()
        # yolo 模式要求 use_sim_time=true：/clock 缺失时明确报错退出，而不是静默拒绝全部量测。
        self._sim_clock_guard = SimClockGuard() if self._vision_source == "yolo" else None
        if self._sim_clock_guard is not None:
            create_sim_clock_guard_timer(self, self._sim_clock_guard)
        self._pursuer_odometry: VehicleOdometry | None = None
        self._target_odometry: VehicleOdometry | None = None
        self._pursuer_received_ns: int | None = None
        self._target_received_ns: int | None = None
        self._camera_intrinsics: CameraIntrinsics | None = None
        self._camera_info_reason = "no_camera_info"
        self._pose_cache: list[_PoseSample] = []
        self._records: list[_SampleRecord] = []
        self._last_logged_reason: str | None = None
        self._last_debug_log_ns: int | None = None
        self._label_written = 0
        self._label_skipped = 0

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        position_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            VehicleOdometry,
            f"{self._pursuer_namespace}/fmu/out/vehicle_odometry",
            self._pursuer_odometry_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleOdometry,
            f"{self._target_namespace}/fmu/out/vehicle_odometry",
            self._target_odometry_callback,
            px4_qos,
        )
        self.create_subscription(CameraInfo, "/camera/camera_info", self._camera_info_callback, camera_qos)
        if self._vision_source == "yolo":
            self.create_subscription(Detection2DArray, "/camera/detections", self._detections_callback, detection_qos)
        else:
            # truth 伪检测使用独立定时器，不要求与图像同频。
            self.create_timer(1.0 / self._truth_rate_hz, self._on_timer)

        self._detections_pub = self.create_publisher(Detection2DArray, "/camera/detections_truth", detection_qos)
        self._target_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/vision/target_pose", position_qos)

        self.get_logger().info(
            f"vision_adapter ready: source={self._vision_source}, "
            f"pursuer={self._pursuer_namespace}, target={self._target_namespace}, "
            f"camera_frame={self._camera_frame_id}, truth_rate_hz={self._truth_rate_hz:.1f}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("vision_source", "off")
        self.declare_parameter("pursuer_namespace", "/px4_1")
        self.declare_parameter("target_namespace", "/px4_2")
        self.declare_parameter("camera_frame_id", "camera_link_optical")
        self.declare_parameter("target_base_altitude", 1.0)
        self.declare_parameter("camera_mount_xyz", [0.0, 0.0, 0.10])
        self.declare_parameter("camera_mount_rpy_deg", [0.0, 90.0, 0.0])
        self.declare_parameter("truth_rate_hz", 10.0)
        self.declare_parameter("pose_timeout_s", 0.2)
        self.declare_parameter("pose_pair_tolerance_s", 0.05)
        self.declare_parameter("pixel_noise_px", 3.0)
        self.declare_parameter("target_plane_sigma_m", 0.1)
        self.declare_parameter("vision_record_data", True)
        self.declare_parameter("vision_record_output_dir", "outputs/gazebo2d_vision")
        self.declare_parameter("debug_log", False)
        self.declare_parameter("debug_log_period_s", 0.5)
        # yolo 模式新增参数。
        self.declare_parameter("min_score", 0.25)
        self.declare_parameter("pose_match_tolerance_s", 0.05)
        self.declare_parameter("pose_cache_max_age_s", 0.5)
        self.declare_parameter("pose_cache_interpolate", True)
        self.declare_parameter("extrapolate_pose", False)
        self.declare_parameter("record_dataset", False)
        self.declare_parameter("dataset_output_dir", "outputs/gazebo2d_vision/dataset")
        self.declare_parameter("dataset_label_box_size_m", DEFAULT_LABEL_BOX_SIZE_M)

    def _load_parameters(self) -> None:
        self._vision_source = str(self.get_parameter("vision_source").value)
        if self._vision_source not in SUPPORTED_VISION_SOURCES:
            raise ValueError(
                f"Unsupported vision_source {self._vision_source!r}; expected one of {SUPPORTED_VISION_SOURCES}"
            )
        if self._vision_source == "off":
            raise ValueError("vision_adapter 只在 vision_source=truth/yolo 时启动；off 模式不应创建该节点")
        if self._vision_source == "yolo" and not _as_bool(self.get_parameter("use_sim_time").value):
            raise ValueError(
                "vision_source=yolo 要求 use_sim_time=true：图像 stamp 与位姿缓存必须使用同一仿真时间基准"
            )

        self._pursuer_namespace = "/" + str(self.get_parameter("pursuer_namespace").value).strip("/")
        self._target_namespace = "/" + str(self.get_parameter("target_namespace").value).strip("/")
        self._camera_frame_id = str(self.get_parameter("camera_frame_id").value)
        if not self._camera_frame_id:
            raise ValueError("camera_frame_id must not be empty")

        self._target_base_altitude = float(self.get_parameter("target_base_altitude").value)
        if not math.isfinite(self._target_base_altitude):
            raise ValueError("target_base_altitude must be finite")

        self._camera_mount_xyz = self._float_vector("camera_mount_xyz")
        mount_rpy_deg = self._float_vector("camera_mount_rpy_deg")
        self._camera_mount_rpy_rad = tuple(math.radians(value) for value in mount_rpy_deg)

        self._truth_rate_hz = self._positive_float("truth_rate_hz")
        self._pose_timeout_s = self._positive_float("pose_timeout_s")
        self._pose_pair_tolerance_s = self._positive_float("pose_pair_tolerance_s")
        self._pixel_noise_px = self._positive_float("pixel_noise_px")
        self._target_plane_sigma_m = self._positive_float("target_plane_sigma_m")
        self._debug_log_period_s = self._positive_float("debug_log_period_s")

        self._vision_record_data = _as_bool(self.get_parameter("vision_record_data").value)
        self._vision_record_output_dir = resolve_recording_path(
            str(self.get_parameter("vision_record_output_dir").value)
        )
        self._debug_log = _as_bool(self.get_parameter("debug_log").value)

        self._min_score = self._bounded_float("min_score", 0.0, 1.0)
        self._pose_match_tolerance_s = self._positive_float("pose_match_tolerance_s")
        self._pose_cache_max_age_s = self._positive_float("pose_cache_max_age_s")
        self._pose_cache_interpolate = _as_bool(self.get_parameter("pose_cache_interpolate").value)
        self._extrapolate_pose = _as_bool(self.get_parameter("extrapolate_pose").value)
        self._record_dataset = _as_bool(self.get_parameter("record_dataset").value)
        self._dataset_output_dir = Path(str(self.get_parameter("dataset_output_dir").value)).expanduser()
        self._label_box_size_m = self._positive_float("dataset_label_box_size_m")

    def _positive_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be a positive finite number")
        return value

    def _bounded_float(self, name: str, lower: float, upper: float) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(f"{name} must be in [{lower}, {upper}]")
        return value

    def _float_vector(self, name: str) -> tuple[float, float, float]:
        raw = self.get_parameter(name).value
        if isinstance(raw, str):
            try:
                raw = [float(item) for item in raw.strip().strip("[]").split(",")]
            except ValueError as exc:
                raise ValueError(f"{name} must be a 3-element numeric array") from exc
        values = np.asarray(raw, dtype=float)
        if values.shape != (3,) or not np.all(np.isfinite(values)):
            raise ValueError(f"{name} must be a 3-element finite array")
        return float(values[0]), float(values[1]), float(values[2])

    # ------------------------------------------------------------ odometry/内参

    def _sim_now_ns(self) -> int:
        """仿真时间当前值；yolo 模式的时间基准，测试可覆盖该方法。"""
        return int(self.get_clock().now().nanoseconds)

    def _pursuer_odometry_callback(self, message: VehicleOdometry) -> None:
        self._pursuer_odometry = message
        self._pursuer_received_ns = time.monotonic_ns()
        if self._vision_source == "yolo":
            self._cache_pursuer_pose(message)

    def _target_odometry_callback(self, message: VehicleOdometry) -> None:
        self._target_odometry = message
        self._target_received_ns = time.monotonic_ns()

    def _cache_pursuer_pose(self, message: VehicleOdometry) -> None:
        position = np.asarray(message.position, dtype=float)
        quaternion = np.asarray(message.q, dtype=float)
        velocity = np.asarray(message.velocity, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            return
        if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)) or np.linalg.norm(quaternion) < 1e-9:
            return
        if velocity.shape != (3,) or not np.all(np.isfinite(velocity)):
            velocity = np.zeros(3, dtype=float)

        now_ns = self._sim_now_ns()
        sample = _PoseSample(
            t_sim_ns=now_ns,
            position_ned=position.copy(),
            quaternion_wxyz=quaternion.copy(),
            velocity_ned=velocity.copy(),
        )
        bisect.insort(self._pose_cache, sample, key=lambda item: item.t_sim_ns)

        # 只保留 pose_cache_max_age_s 内的样本，避免用过期位姿插值。
        cutoff_ns = now_ns - int(self._pose_cache_max_age_s * 1e9)
        while self._pose_cache and self._pose_cache[0].t_sim_ns < cutoff_ns:
            self._pose_cache.pop(0)

    def _camera_info_callback(self, message: CameraInfo) -> None:
        intrinsics, reason = self._validate_camera_info(message)
        self._camera_intrinsics = intrinsics
        self._camera_info_reason = reason or ""

    def _validate_camera_info(self, message: CameraInfo) -> tuple[CameraIntrinsics | None, str | None]:
        """校验并缓存内参；返回 `(intrinsics, None)` 或 `(None, 拒绝原因)`。"""
        if message.width <= 0 or message.height <= 0:
            return None, "invalid_camera_info"
        k = np.asarray(message.k, dtype=float)
        if k.shape != (9,) or not np.all(np.isfinite(k)):
            return None, "invalid_intrinsics"

        intrinsics = CameraIntrinsics(
            width=int(message.width),
            height=int(message.height),
            fx=float(k[0]),
            fy=float(k[4]),
            cx=float(k[2]),
            cy=float(k[5]),
        )
        if validate_intrinsics(intrinsics) is not None:
            return None, "invalid_intrinsics"

        distortion = np.asarray(message.d, dtype=float)
        # 本轮只接受零畸变；非零或非有限畸变必须显式拒绝，不能静默忽略。
        if distortion.size > 0 and not np.allclose(distortion, 0.0):
            return None, "unsupported_distortion"
        if message.header.frame_id != self._camera_frame_id:
            return None, "invalid_frame"
        return intrinsics, None

    # ------------------------------------------------------------------ truth

    def _on_timer(self) -> None:
        stamp_ns = int(self.get_clock().now().nanoseconds)
        now_mono_ns = time.monotonic_ns()
        record = _SampleRecord(
            stamp_ns=stamp_ns,
            elapsed_s=(now_mono_ns - self._start_mono_ns) * 1e-9,
            source="truth",
        )
        self._compute_truth_measurement(now_mono_ns, record)
        self._records.append(record)
        self._log_reason_change(record)
        self._maybe_log_debug(record)

    def _compute_truth_measurement(self, now_mono_ns: int, record: _SampleRecord) -> None:
        intrinsics = self._camera_intrinsics
        if intrinsics is None:
            record.invalid_reason = self._camera_info_reason or "no_camera_info"
            return

        reason = self._paired_odometry(now_mono_ns, record)
        if reason is not None:
            record.invalid_reason = reason
            return

        pose_result = camera_pose_from_odometry(
            self._pursuer_odometry.position,
            self._pursuer_odometry.q,
            self._camera_mount_xyz,
            self._camera_mount_rpy_rad,
        )
        if pose_result is None:
            record.invalid_reason = "invalid_pursuer_odometry"
            return
        pose = CameraPose(position_enu=pose_result[0], rotation_world_from_optical=pose_result[1])

        reference_enu = self._target_reference_enu(record)
        if reference_enu is None:
            record.invalid_reason = "invalid_target_odometry"
            return

        pixel = ground_to_pixel(reference_enu, pose, intrinsics)
        if pixel is None:
            record.invalid_reason = "projection_failed"
            return

        detections = self._build_detections(pixel, record.stamp_ns)
        self._detections_pub.publish(detections)
        # truth 直接调用共享处理函数，不自订阅自己的检测话题。
        self._finalize_measurement(pixel, pose, intrinsics, reference_enu, record)

    def _paired_odometry(self, now_mono_ns: int, record: _SampleRecord) -> str | None:
        """校验两机 odometry 的 frame、新鲜度和配对容差，并把诊断写进 record。"""
        if self._pursuer_odometry is None or self._pursuer_received_ns is None:
            return "no_pursuer_odometry"
        if self._target_odometry is None or self._target_received_ns is None:
            return "no_target_odometry"
        if self._pursuer_odometry.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return "unsupported_pursuer_frame"
        if self._target_odometry.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return "unsupported_target_frame"

        record.pursuer_timestamp_sample_us = int(self._pursuer_odometry.timestamp_sample)
        record.target_timestamp_sample_us = int(self._target_odometry.timestamp_sample)
        record.pursuer_age_ms = (now_mono_ns - self._pursuer_received_ns) * 1e-6
        record.target_age_ms = (now_mono_ns - self._target_received_ns) * 1e-6
        pair_delta_ns = abs(self._pursuer_received_ns - self._target_received_ns)
        record.pose_pair_delta_ms = pair_delta_ns * 1e-6

        if record.pursuer_age_ms > self._pose_timeout_s * 1e3:
            return "stale_pursuer_odometry"
        if record.target_age_ms > self._pose_timeout_s * 1e3:
            return "stale_target_odometry"
        if pair_delta_ns * 1e-9 > self._pose_pair_tolerance_s:
            return "pose_pair_delta_too_large"
        return None

    def _target_reference_enu(self, record: _SampleRecord) -> np.ndarray | None:
        """目标 odometry 位置按参考 XY 和固定目标平面高度合成投影输入。

        truth 模式必须存在（否则拒绝）；yolo 模式只用于误差诊断，可以为空。
        """
        if self._target_odometry is None:
            return None
        position_ned = np.asarray(self._target_odometry.position, dtype=float)
        if position_ned.shape != (3,) or not np.all(np.isfinite(position_ned)):
            return None

        position_enu = np.array([position_ned[1], position_ned[0], -position_ned[2]], dtype=float)
        record.target_x_ref = float(position_enu[0])
        record.target_y_ref = float(position_enu[1])
        record.target_z_odom = float(position_enu[2])
        record.target_plane_z = float(self._target_base_altitude)
        return np.array([position_enu[0], position_enu[1], self._target_base_altitude], dtype=float)

    def _build_detections(self, pixel: np.ndarray, stamp_ns: int) -> Detection2DArray:
        message = Detection2DArray()
        self._stamp_header(message.header, stamp_ns)
        message.header.frame_id = self._camera_frame_id

        detection = Detection2D()
        detection.header = message.header
        detection.bbox.center.position.x = float(pixel[0])
        detection.bbox.center.position.y = float(pixel[1])
        detection.bbox.center.theta = 0.0
        detection.bbox.size_x = TRUTH_BBOX_SIZE_PX
        detection.bbox.size_y = TRUTH_BBOX_SIZE_PX

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = "drone"
        hypothesis.hypothesis.score = 1.0
        detection.results.append(hypothesis)

        message.detections.append(detection)
        return message

    # ------------------------------------------------------------------- yolo

    def _detections_callback(self, message: Detection2DArray) -> None:
        """yolo 模式入口：每次检测回调产生一行记录（含空检测/拒绝）。"""
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        now_mono_ns = time.monotonic_ns()
        record = _SampleRecord(
            stamp_ns=stamp_ns,
            elapsed_s=(now_mono_ns - self._start_mono_ns) * 1e-9,
            source="yolo",
        )
        record.image_stamp_s = stamp_ns * 1e-9
        record.n_detections = float(len(message.detections))
        record.target_plane_z = float(self._target_base_altitude)
        self._compute_yolo_measurement(message, record)
        self._records.append(record)
        self._log_reason_change(record)
        self._maybe_log_debug(record)

    def _compute_yolo_measurement(self, message: Detection2DArray, record: _SampleRecord) -> None:
        intrinsics = self._camera_intrinsics
        if intrinsics is None:
            record.invalid_reason = self._camera_info_reason or "no_camera_info"
            return

        now_sim_ns = self._sim_now_ns()
        record.detection_age_ms = (now_sim_ns - record.stamp_ns) * 1e-6

        pose, reason, match_dt_ms, interpolated = self._pose_at(record.stamp_ns)
        if pose is None:
            record.invalid_reason = reason
            return
        record.pose_match_dt_ms = match_dt_ms
        record.pose_interpolated = 1.0 if interpolated else 0.0

        selection = _select_drone_detection(message)
        if selection is None:
            record.invalid_reason = "no_drone_detection"
            return
        detection, score = selection
        record.score = score
        record.bbox_w = float(detection.bbox.size_x)
        record.bbox_h = float(detection.bbox.size_y)
        if score < self._min_score:
            record.invalid_reason = "low_score"
            return

        pixel = np.array(
            [detection.bbox.center.position.x, detection.bbox.center.position.y],
            dtype=float,
        )
        # 目标 odometry 在 yolo 模式只是诊断参考，不参与量测本身。
        reference_enu = self._target_reference_enu(record)
        self._finalize_measurement(pixel, pose, intrinsics, reference_enu, record)

    def _pose_at(self, stamp_ns: int) -> tuple[CameraPose | None, str, float, bool]:
        """按图像 stamp 从位姿缓存取相机位姿。

        返回 `(pose, reason, match_dt_ms, interpolated)`；拒绝时 pose 为 None，
        reason 属于 `pose_cache_miss/pose_cache_stale/pose_cache_future/invalid_pursuer_odometry`。
        """
        cache = self._pose_cache
        if not cache:
            return None, "no_pursuer_odometry", math.nan, False

        tolerance_ns = int(self._pose_match_tolerance_s * 1e9)
        first, last = cache[0], cache[-1]
        if stamp_ns < first.t_sim_ns - tolerance_ns:
            return None, "pose_cache_stale", math.nan, False
        if stamp_ns > last.t_sim_ns + tolerance_ns:
            if not self._extrapolate_pose:
                return None, "pose_cache_future", math.nan, False
            if stamp_ns - last.t_sim_ns > int(self._pose_cache_max_age_s * 1e9):
                return None, "pose_cache_future", math.nan, False
            position = last.position_ned + last.velocity_ned * (stamp_ns - last.t_sim_ns) * 1e-9
            pose = self._compose_pose(position, last.quaternion_wxyz)
            if pose is None:
                return None, "invalid_pursuer_odometry", math.nan, False
            return pose, "", (stamp_ns - last.t_sim_ns) * 1e-6, False

        index = bisect.bisect_left(cache, stamp_ns, key=lambda sample: sample.t_sim_ns)
        if self._pose_cache_interpolate and 0 < index < len(cache):
            left, right = cache[index - 1], cache[index]
            if right.t_sim_ns - left.t_sim_ns > tolerance_ns:
                # 中间有 odometry 空洞，插值会假装位姿连续，必须拒绝。
                return None, "pose_cache_miss", math.nan, False
            if right.t_sim_ns == left.t_sim_ns:
                alpha = 0.0
            else:
                alpha = (stamp_ns - left.t_sim_ns) / (right.t_sim_ns - left.t_sim_ns)
            position = left.position_ned + alpha * (right.position_ned - left.position_ned)
            quaternion = _slerp_quaternion(left.quaternion_wxyz, right.quaternion_wxyz, alpha)
            pose = self._compose_pose(position, quaternion)
            if pose is None:
                return None, "invalid_pursuer_odometry", math.nan, False
            return pose, "", 0.0, True

        candidates = [cache[max(0, index - 1)], cache[min(len(cache) - 1, index)]]
        nearest = min(candidates, key=lambda sample: abs(sample.t_sim_ns - stamp_ns))
        match_dt_ns = stamp_ns - nearest.t_sim_ns
        if abs(match_dt_ns) > tolerance_ns:
            return None, "pose_cache_miss", math.nan, False
        pose = self._compose_pose(nearest.position_ned, nearest.quaternion_wxyz)
        if pose is None:
            return None, "invalid_pursuer_odometry", math.nan, False
        return pose, "", match_dt_ns * 1e-6, False

    def _compose_pose(self, position_ned: np.ndarray, quaternion_wxyz: np.ndarray) -> CameraPose | None:
        pose_result = camera_pose_from_odometry(
            position_ned,
            quaternion_wxyz,
            self._camera_mount_xyz,
            self._camera_mount_rpy_rad,
        )
        if pose_result is None:
            return None
        return CameraPose(position_enu=pose_result[0], rotation_world_from_optical=pose_result[1])

    # ------------------------------------------------------------ 共享量测处理

    def _finalize_measurement(
        self,
        pixel: np.ndarray,
        pose: CameraPose,
        intrinsics: CameraIntrinsics,
        reference_enu: np.ndarray | None,
        record: _SampleRecord,
    ) -> None:
        """共享反投影链路：像素 → 目标平面交点 → 发布 `/vision/target_pose`。

        truth 伪检测和 yolo 检测都必须走这里，禁止再写第二套反投影。
        `reference_enu` 只用于误差诊断和数据集标注，不参与量测本身。
        """
        record.u_ref = float(pixel[0])
        record.v_ref = float(pixel[1])

        result = pixel_to_ground(pixel, pose, intrinsics, self._target_base_altitude)
        if result is None:
            record.invalid_reason = "backprojection_failed"
            return

        point, jacobian = result
        record.target_x_est = float(point[0])
        record.target_y_est = float(point[1])
        if reference_enu is not None:
            record.position_roundtrip_error_m = float(np.linalg.norm(point - reference_enu))
            if record.source == "yolo":
                record.position_error_vs_odom_m = float(np.linalg.norm(point[:2] - reference_enu[:2]))
                truth_pixel = ground_to_pixel(reference_enu, pose, intrinsics)
                if truth_pixel is not None:
                    record.pixel_error_vs_truth_px = float(np.linalg.norm(pixel - truth_pixel))

        record.valid = True
        self._target_pose_pub.publish(self._build_position_message(point, jacobian, record.stamp_ns))

        if record.source == "yolo" and self._record_dataset and reference_enu is not None:
            self._write_dataset_label(record.stamp_ns, pose, intrinsics, reference_enu)

    def _build_position_message(
        self,
        point_enu: np.ndarray,
        jacobian: np.ndarray,
        stamp_ns: int,
    ) -> PoseWithCovarianceStamped:
        """位置为反投影交点；姿态填单位四元数但明确“不提供姿态观测”。

        6×6 行优先协方差的 XY 块索引是 0、1、6、7；本轮只传播像素噪声，
        z 用固定平面高度不确定度，姿态对角线用大而有限的方差。
        """
        message = PoseWithCovarianceStamped()
        self._stamp_header(message.header, stamp_ns)
        message.header.frame_id = "enu"

        message.pose.pose.position.x = float(point_enu[0])
        message.pose.pose.position.y = float(point_enu[1])
        message.pose.pose.position.z = float(point_enu[2])
        message.pose.pose.orientation.w = 1.0

        covariance = np.zeros((6, 6), dtype=float)
        xy_covariance = (self._pixel_noise_px ** 2) * (jacobian @ jacobian.T)
        covariance[0, 0] = xy_covariance[0, 0]
        covariance[0, 1] = xy_covariance[0, 1]
        covariance[1, 0] = xy_covariance[1, 0]
        covariance[1, 1] = xy_covariance[1, 1]
        covariance[2, 2] = self._target_plane_sigma_m ** 2
        covariance[3, 3] = POSE_UNKNOWN_VARIANCE
        covariance[4, 4] = POSE_UNKNOWN_VARIANCE
        covariance[5, 5] = POSE_UNKNOWN_VARIANCE
        message.pose.covariance = covariance.reshape(-1).tolist()
        return message

    @staticmethod
    def _stamp_header(header, stamp_ns: int) -> None:
        header.stamp.sec = int(stamp_ns // 1_000_000_000)
        header.stamp.nanosec = int(stamp_ns % 1_000_000_000)

    # ------------------------------------------------------------- 数据集标注

    def _write_dataset_label(
        self,
        stamp_ns: int,
        pose: CameraPose,
        intrinsics: CameraIntrinsics,
        reference_enu: np.ndarray,
    ) -> None:
        """把目标真值盒投影成 YOLO 归一化标注；缺帧时留空（视为背景负样本）。

        目标盒中心取目标 odometry 位置（含真实高度），边长 `dataset_label_box_size_m`；
        8 个角点投影后取 AABB，加 8% margin 并夹到图像范围内。
        """
        half = self._label_box_size_m * 0.5
        offsets = [
            (dx, dy, dz)
            for dx in (-half, half)
            for dy in (-half, half)
            for dz in (-half, half)
        ]
        center = np.array(
            [reference_enu[0], reference_enu[1], self._target_odom_z_or(reference_enu)],
            dtype=float,
        )
        pixels: list[np.ndarray] = []
        for offset in offsets:
            pixel = ground_to_pixel(center + np.array(offset), pose, intrinsics)
            if pixel is not None:
                pixels.append(pixel)
        if len(pixels) < 4:
            self._label_skipped += 1
            return

        stacked = np.array(pixels, dtype=float)
        u_min, v_min = stacked.min(axis=0)
        u_max, v_max = stacked.max(axis=0)
        width = max(1e-6, u_max - u_min)
        height = max(1e-6, v_max - v_min)
        u_min -= width * LABEL_MARGIN_FRACTION * 0.5
        u_max += width * LABEL_MARGIN_FRACTION * 0.5
        v_min -= height * LABEL_MARGIN_FRACTION * 0.5
        v_max += height * LABEL_MARGIN_FRACTION * 0.5

        u_min = max(0.0, min(float(intrinsics.width - 1), u_min))
        u_max = max(0.0, min(float(intrinsics.width - 1), u_max))
        v_min = max(0.0, min(float(intrinsics.height - 1), v_min))
        v_max = max(0.0, min(float(intrinsics.height - 1), v_max))
        if u_max <= u_min or v_max <= v_min:
            self._label_skipped += 1
            return

        center_u = (u_min + u_max) * 0.5 / intrinsics.width
        center_v = (v_min + v_max) * 0.5 / intrinsics.height
        norm_w = (u_max - u_min) / intrinsics.width
        norm_h = (v_max - v_min) / intrinsics.height
        path = self._dataset_output_dir / "labels" / f"{stamp_ns}.txt"
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(
                f"0 {center_u:.6f} {center_v:.6f} {norm_w:.6f} {norm_h:.6f}\n",
                encoding="utf-8",
            )
            self._label_written += 1
        except OSError as exc:
            self.get_logger().warning(f"数据集标注写入失败：{exc}", throttle_duration_sec=5.0)

    def _target_odom_z_or(self, reference_enu: np.ndarray) -> float:
        """标注盒中心高度：优先用目标 odometry 的真实高度，缺失时退回参考平面。"""
        if self._target_odometry is not None:
            position_ned = np.asarray(self._target_odometry.position, dtype=float)
            if position_ned.shape == (3,) and np.all(np.isfinite(position_ned)):
                return float(-position_ned[2])
        return float(reference_enu[2])

    # ------------------------------------------------------------ 日志/记录

    def _log_reason_change(self, record: _SampleRecord) -> None:
        reason = record.invalid_reason if not record.valid else ""
        if reason == self._last_logged_reason:
            return
        self._last_logged_reason = reason
        if record.valid:
            self.get_logger().info(f"vision {record.source} measurement output is valid")
        else:
            self.get_logger().info(f"vision {record.source} waiting/rejected: {reason}")

    def _maybe_log_debug(self, record: _SampleRecord) -> None:
        if not self._debug_log:
            return
        now_mono_ns = time.monotonic_ns()
        if self._last_debug_log_ns is not None:
            since_last_s = (now_mono_ns - self._last_debug_log_ns) * 1e-9
            if since_last_s < self._debug_log_period_s:
                return
        self._last_debug_log_ns = now_mono_ns
        self.get_logger().info(
            f"vision_{record.source} "
            f"valid={record.valid} reason={record.invalid_reason or '-'} "
            f"elapsed={record.elapsed_s:.2f}s "
            f"ages_ms={record.pursuer_age_ms:.1f}/{record.target_age_ms:.1f} "
            f"pair_ms={record.pose_pair_delta_ms:.1f} "
            f"uv=({record.u_ref:.1f}, {record.v_ref:.1f}) "
            f"est=({record.target_x_est:.3f}, {record.target_y_est:.3f}) "
            f"ref=({record.target_x_ref:.3f}, {record.target_y_ref:.3f}) "
            f"z_odom={record.target_z_odom:.3f} plane={record.target_plane_z:.3f} "
            f"img_age_ms={record.detection_age_ms:.1f} match_dt_ms={record.pose_match_dt_ms:.1f} "
            f"score={record.score:.3f} n={record.n_detections:.0f} "
            f"px_err={record.pixel_error_vs_truth_px:.2f} xy_err={record.position_error_vs_odom_m:.3f} "
            f"roundtrip_err={record.position_roundtrip_error_m:.4f}m"
        )

    def save_recording(self) -> None:
        if not self._vision_record_data:
            return
        if not self._records:
            self._log_shutdown_safe("warn", "vision_record_data is enabled, but no vision samples were collected")
            return

        try:
            path = self._vision_record_output_dir / "vision_samples.csv"
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("w", newline="", encoding="utf-8") as file:
                writer = csv.DictWriter(file, fieldnames=CSV_FIELDS)
                writer.writeheader()
                for record in self._records:
                    writer.writerow(_record_to_row(record))
            self._log_shutdown_safe("info", f"saved vision samples CSV to {path}")
            if self._record_dataset:
                self._log_shutdown_safe(
                    "info",
                    f"dataset labels written={self._label_written}, skipped={self._label_skipped} "
                    f"(skipped 帧按背景负样本处理)",
                )
        except Exception as exc:  # noqa: BLE001
            self._log_shutdown_safe("error", f"failed to save vision samples: {exc}")

    def _log_shutdown_safe(self, level: str, message: str) -> None:
        if rclpy.ok():
            getattr(self.get_logger(), level)(message)
            return
        stream = sys.stderr if level == "error" else sys.stdout
        print(f"[{level.upper()}] [vision_adapter]: {message}", file=stream)


def _record_to_row(record: _SampleRecord) -> dict[str, object]:
    return {
        "elapsed_s": record.elapsed_s,
        "stamp_s": record.stamp_ns * 1e-9,
        "valid": int(record.valid),
        "invalid_reason": record.invalid_reason,
        "pursuer_timestamp_sample_us": "" if record.pursuer_timestamp_sample_us is None
        else record.pursuer_timestamp_sample_us,
        "target_timestamp_sample_us": "" if record.target_timestamp_sample_us is None
        else record.target_timestamp_sample_us,
        "pursuer_age_ms": record.pursuer_age_ms,
        "target_age_ms": record.target_age_ms,
        "pose_pair_delta_ms": record.pose_pair_delta_ms,
        "u_ref": record.u_ref,
        "v_ref": record.v_ref,
        "target_x_est": record.target_x_est,
        "target_y_est": record.target_y_est,
        "target_x_ref": record.target_x_ref,
        "target_y_ref": record.target_y_ref,
        "target_z_odom": record.target_z_odom,
        "target_plane_z": record.target_plane_z,
        "position_roundtrip_error_m": record.position_roundtrip_error_m,
        "source": record.source,
        "image_stamp_s": record.image_stamp_s,
        "detection_age_ms": record.detection_age_ms,
        "pose_match_dt_ms": record.pose_match_dt_ms,
        "pose_interpolated": record.pose_interpolated,
        "score": record.score,
        "bbox_w": record.bbox_w,
        "bbox_h": record.bbox_h,
        "n_detections": record.n_detections,
        "pixel_error_vs_truth_px": record.pixel_error_vs_truth_px,
        "position_error_vs_odom_m": record.position_error_vs_odom_m,
    }


def _as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "on"}
    return bool(value)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    try:
        node = VisionAdapter()
    except ValueError as exc:
        print(f"[ERROR] [vision_adapter]: {exc}", file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.save_recording()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
