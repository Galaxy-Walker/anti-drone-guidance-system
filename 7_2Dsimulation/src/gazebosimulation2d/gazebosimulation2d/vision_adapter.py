"""纯旁路的视觉 truth 适配节点。

本轮只把相机内参和两机 odometry 转换成独立位置量测：

- 订阅 `/camera/camera_info`，缓存并校验内参和 frame。
- 用追踪机 odometry 的完整姿态加安装外参构造相机位姿。
- 把目标机 odometry 参考位置投影成伪检测，发布 `/camera/detections_truth`。
- 同一处理函数直接把伪检测反投影到固定目标平面，发布 `/vision/target_pose` 并记录 CSV。

节点不订阅图像、不解码图像、不生成标注图，也不控制飞机；输出不被现有导引消费。
时间语义、拒绝条件和验收边界见 `docs/camera_vision_integration_plan.md` 的 P3 章节。
"""

from __future__ import annotations

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

# 伪检测的 bbox 只是像素级占位，不冒充真实目标框尺寸；中心才是有效信息。
TRUTH_BBOX_SIZE_PX = 4.0

# 本轮不提供姿态观测，用大而有限的方差明确表示“未知”，而不是全零。
POSE_UNKNOWN_VARIANCE = 1.0e6

# 本轮只支持 off/truth；yolo 等模式需先完成时钟映射和标定。
SUPPORTED_VISION_SOURCES = ("off", "truth")

CSV_FIELDS = (
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
)


@dataclass(slots=True)
class _SampleRecord:
    """一次定时器周期的量测结果；无效时用 `invalid_reason` 说明拒绝原因。"""

    stamp_ns: int
    elapsed_s: float
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


def _select_drone_detection(message: Detection2DArray) -> Detection2D | None:
    """按契约挑选有效 drone 检测中分数最高的一条。"""
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
    return best


class VisionAdapter(Node):
    """订阅相机内参与两机 odometry，发布 truth 伪检测和位置量测。"""

    def __init__(self, parameter_overrides: list[Parameter] | None = None) -> None:
        super().__init__("vision_adapter", parameter_overrides=parameter_overrides)
        self._declare_parameters()
        self._load_parameters()

        self._start_mono_ns = time.monotonic_ns()
        self._pursuer_odometry: VehicleOdometry | None = None
        self._target_odometry: VehicleOdometry | None = None
        self._pursuer_received_ns: int | None = None
        self._target_received_ns: int | None = None
        self._camera_intrinsics: CameraIntrinsics | None = None
        self._camera_info_reason = "no_camera_info"
        self._records: list[_SampleRecord] = []
        self._last_logged_reason: str | None = None
        self._last_debug_log_ns: int | None = None

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

        self._detections_pub = self.create_publisher(Detection2DArray, "/camera/detections_truth", detection_qos)
        self._target_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/vision/target_pose", position_qos)

        self.create_timer(1.0 / self._truth_rate_hz, self._on_timer)
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

    def _load_parameters(self) -> None:
        self._vision_source = str(self.get_parameter("vision_source").value)
        if self._vision_source not in SUPPORTED_VISION_SOURCES:
            raise ValueError(
                f"Unsupported vision_source {self._vision_source!r}; expected one of {SUPPORTED_VISION_SOURCES}"
            )
        if self._vision_source != "truth":
            raise ValueError("vision_adapter 只在 vision_source=truth 时启动；off 模式不应创建该节点")

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
        self._vision_record_output_dir = Path(
            str(self.get_parameter("vision_record_output_dir").value)
        ).expanduser()
        self._debug_log = _as_bool(self.get_parameter("debug_log").value)

    def _positive_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be a positive finite number")
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

    def _pursuer_odometry_callback(self, message: VehicleOdometry) -> None:
        self._pursuer_odometry = message
        self._pursuer_received_ns = time.monotonic_ns()

    def _target_odometry_callback(self, message: VehicleOdometry) -> None:
        self._target_odometry = message
        self._target_received_ns = time.monotonic_ns()

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

    def _on_timer(self) -> None:
        stamp_ns = int(self.get_clock().now().nanoseconds)
        now_mono_ns = time.monotonic_ns()
        record = _SampleRecord(
            stamp_ns=stamp_ns,
            elapsed_s=(now_mono_ns - self._start_mono_ns) * 1e-9,
        )
        self._compute_measurement(now_mono_ns, record)
        self._records.append(record)
        self._log_reason_change(record)
        self._maybe_log_debug(record)

    def _compute_measurement(self, now_mono_ns: int, record: _SampleRecord) -> None:
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
        self._handle_detections(detections, pose, intrinsics, reference_enu, record)

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
        """目标 odometry 位置按参考 XY 和固定目标平面高度合成投影输入。"""
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

    def _handle_detections(
        self,
        detections: Detection2DArray,
        pose: CameraPose,
        intrinsics: CameraIntrinsics,
        reference_enu: np.ndarray,
        record: _SampleRecord,
    ) -> None:
        """共享检测处理：像素 → 目标平面交点 → 位置量测。

        truth 生成伪检测后直接调用本函数；后续 YOLO 模式应复用同一函数，
        不要再写第二套反投影。
        """
        detection = _select_drone_detection(detections)
        if detection is None:
            record.invalid_reason = "no_drone_detection"
            return

        pixel = np.array(
            [detection.bbox.center.position.x, detection.bbox.center.position.y],
            dtype=float,
        )
        record.u_ref = float(pixel[0])
        record.v_ref = float(pixel[1])

        result = pixel_to_ground(pixel, pose, intrinsics, self._target_base_altitude)
        if result is None:
            record.invalid_reason = "backprojection_failed"
            return

        point, jacobian = result
        record.target_x_est = float(point[0])
        record.target_y_est = float(point[1])
        record.position_roundtrip_error_m = float(np.linalg.norm(point - reference_enu))
        record.valid = True
        self._target_pose_pub.publish(self._build_position_message(point, jacobian, record.stamp_ns))

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

    def _log_reason_change(self, record: _SampleRecord) -> None:
        reason = record.invalid_reason if not record.valid else ""
        if reason == self._last_logged_reason:
            return
        self._last_logged_reason = reason
        if record.valid:
            self.get_logger().info("vision truth measurement output is valid")
        else:
            self.get_logger().info(f"vision truth waiting/rejected: {reason}")

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
            "vision_truth "
            f"valid={record.valid} reason={record.invalid_reason or '-'} "
            f"elapsed={record.elapsed_s:.2f}s "
            f"ages_ms={record.pursuer_age_ms:.1f}/{record.target_age_ms:.1f} "
            f"pair_ms={record.pose_pair_delta_ms:.1f} "
            f"uv=({record.u_ref:.1f}, {record.v_ref:.1f}) "
            f"est=({record.target_x_est:.3f}, {record.target_y_est:.3f}) "
            f"ref=({record.target_x_ref:.3f}, {record.target_y_ref:.3f}) "
            f"z_odom={record.target_z_odom:.3f} plane={record.target_plane_z:.3f} "
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
