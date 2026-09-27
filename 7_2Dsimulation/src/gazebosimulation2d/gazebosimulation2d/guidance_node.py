"""把 2D 定高导引算法接入 PX4/Gazebo 的 ROS 2 节点。

节点沿用 6_Simulation 的双 PX4 实例接口：

- pursuer：追踪机，使用 `pythonsimulation2d` 的导引算法，并发布 XY 位置 setpoint + 固定高度。
- target：目标机，沿用 6 的目标机/话题/模型，按合成目标参考轨迹飞行。

导引、距离和记录指标都按 XY 平面计算；高度只用于 Gazebo/PX4 setpoint。
`target_source=vision` 时追踪机在准备阶段先飞至场景起点上方（目标机同时停在该起点），
保证开始跟踪时目标已在相机视野内，不依赖两机的 spawn 位置。
"""

from __future__ import annotations

import csv
import math
import sys
import time
from dataclasses import dataclass, field
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


def _as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "on"}
    return bool(value)


import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from pythonsimulation2d.config import ALGORITHMS, SCENARIOS, SimulationConfig
from pythonsimulation2d.guidance import GuidanceMemory, compute_guidance
from pythonsimulation2d.math_utils import clamp_norm_xy, norm_xy
from pythonsimulation2d.state import PursuerState, SimulationResult, TargetState
from pythonsimulation2d.target import target_state
from pythonsimulation2d.target_filter import (
    STATE_LOST,
    TargetFilterConfig,
    VisionTargetTracker,
)

from gazebosimulation2d.recording_paths import resolve_recording_path
from gazebosimulation2d.coordinates import (
    enu_to_ned_list,
    ned_to_enu_vector,
    yaw_enu_to_ned,
    yaw_from_quaternion_ned,
    yaw_to_target_ned,
)
from gazebosimulation2d.px4_utils import (
    arm_command,
    namespaced_topic,
    offboard_control_mode,
    offboard_mode_command,
    timestamp_us,
    trajectory_setpoint,
)
from gazebosimulation2d.sim_clock import SimClockGuard, create_sim_clock_guard_timer

# 视觉量测的 z 固定为目标平面高度；只估计 XY。
VISION_UNKNOWN = math.nan


@dataclass(slots=True)
class _TrackingSample:
    """一个控制周期的完整记录行；视觉列在 odometry 模式下为 NaN。"""

    elapsed_s: float
    pursuer_position: np.ndarray
    pursuer_velocity: np.ndarray
    target_position: np.ndarray
    target_velocity: np.ndarray
    acceleration: np.ndarray
    yaw: float
    distance: float
    target_source: str
    vision_valid: float = VISION_UNKNOWN
    vision_age_s: float = VISION_UNKNOWN
    vision_latency_s: float = VISION_UNKNOWN
    vision_measurements: int = 0
    target_est_x: float = VISION_UNKNOWN
    target_est_y: float = VISION_UNKNOWN
    target_est_vx: float = VISION_UNKNOWN
    target_est_vy: float = VISION_UNKNOWN
    target_est_ax: float = VISION_UNKNOWN
    target_est_ay: float = VISION_UNKNOWN
    vision_error_xy: float = VISION_UNKNOWN


@dataclass(slots=True)
class _VisionSnapshot:
    """一个控制周期使用的视觉估计快照，用于记录与调试日志。"""

    valid: bool = False
    state: str = STATE_LOST
    age_s: float = VISION_UNKNOWN
    latency_s: float = VISION_UNKNOWN
    measurements: int = 0
    position_xy: np.ndarray = field(default_factory=lambda: np.full(2, VISION_UNKNOWN))
    velocity_xy: np.ndarray = field(default_factory=lambda: np.full(2, VISION_UNKNOWN))
    acceleration_xy: np.ndarray = field(default_factory=lambda: np.full(2, VISION_UNKNOWN))


class GuidanceNode(Node):
    """面向一架追踪机和一架目标机的 2D PX4 Offboard 桥接节点。"""

    def __init__(self, parameter_overrides: list[Parameter] | None = None) -> None:
        super().__init__("guidance_node_2d", parameter_overrides=parameter_overrides)
        self._declare_parameters()
        self._load_parameters()

        # use_sim_time=true 但 /clock 缺失时，ROS 定时器不会触发，必须用墙钟定时器报错退出。
        self._sim_clock_guard = (
            SimClockGuard() if _as_bool(self.get_parameter("use_sim_time").value) else None
        )
        if self._sim_clock_guard is not None:
            create_sim_clock_guard_timer(self, self._sim_clock_guard)
        self._vision_tracker = (
            VisionTargetTracker(self._vision_filter_config) if self._target_source == "vision" else None
        )
        self._latest_vision: tuple[float, np.ndarray, np.ndarray] | None = None
        self._consumed_vision_stamp_s: float | None = None
        self._vision_received = 0
        self._vision_measurements = 0
        self._vision_filter_rejects = 0
        self._vision_stale_drops = 0

        self._memory = GuidanceMemory()
        self._pursuer: PursuerState | None = None
        self._target: TargetState | None = None
        self._active_start_ns: int | None = None
        self._last_debug_log_ns: int | None = None
        self._last_startup_log_ns: int | None = None
        self._last_hold_log_ns: int | None = None
        self._pursuer_takeoff_position: np.ndarray | None = None
        self._tracking_started = False

        self._target_offboard_cycles = 0
        self._pursuer_offboard_cycles = 0
        self._target_arm_sent = False
        self._target_offboard_sent = False
        self._pursuer_arm_sent = False
        self._pursuer_offboard_sent = False
        self._target_ready = False
        self._pursuer_ready = False
        self._target_ready_logged = False
        self._pursuer_ready_logged = False

        self._target_yaw_enu = 0.0
        self._waiting_logged = False
        self._record_samples: list[_TrackingSample] = []

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pursuer_offboard_pub = self.create_publisher(
            OffboardControlMode,
            namespaced_topic(self._pursuer_namespace, "fmu/in/offboard_control_mode"),
            qos,
        )
        self._pursuer_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            namespaced_topic(self._pursuer_namespace, "fmu/in/trajectory_setpoint"),
            qos,
        )
        self._pursuer_command_pub = self.create_publisher(
            VehicleCommand,
            namespaced_topic(self._pursuer_namespace, "fmu/in/vehicle_command"),
            qos,
        )

        self._target_offboard_pub = self.create_publisher(
            OffboardControlMode,
            namespaced_topic(self._target_namespace, "fmu/in/offboard_control_mode"),
            qos,
        )
        self._target_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            namespaced_topic(self._target_namespace, "fmu/in/trajectory_setpoint"),
            qos,
        )
        self._target_command_pub = self.create_publisher(
            VehicleCommand,
            namespaced_topic(self._target_namespace, "fmu/in/vehicle_command"),
            qos,
        )

        self.create_subscription(
            VehicleOdometry,
            namespaced_topic(self._pursuer_namespace, "fmu/out/vehicle_odometry"),
            self._pursuer_odometry_callback,
            qos,
        )
        self.create_subscription(
            VehicleOdometry,
            namespaced_topic(self._target_namespace, "fmu/out/vehicle_odometry"),
            self._target_odometry_callback,
            qos,
        )
        if self._target_source == "vision":
            # 与 vision_adapter 的发布端一致：RELIABLE + VOLATILE。
            vision_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self.create_subscription(
                PoseWithCovarianceStamped,
                self._vision_topic,
                self._vision_pose_callback,
                vision_qos,
            )

        self.create_timer(1.0 / self._control_rate_hz, self._timer_callback)
        self.get_logger().info(
            f"guidance_node_2d ready: algorithm={self._algorithm}, scenario={self._scenario}, "
            f"pursuer={self._pursuer_namespace}, target={self._target_namespace}, "
            f"pursuer_fixed_altitude={self._pursuer_fixed_altitude:.2f}m, "
            f"target_source={self._target_source}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("algorithm", "pn_mppi")
        self.declare_parameter("scenario", "circle")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("pursuer_namespace", "/px4_1")
        self.declare_parameter("target_namespace", "/px4_2")
        self.declare_parameter("auto_arm", True)
        self.declare_parameter("auto_offboard", True)
        self.declare_parameter("offboard_warmup_cycles", 20)
        self.declare_parameter("sim_time", 40.0)
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("pursuer_fixed_altitude", 8.0)
        self.declare_parameter("target_base_altitude", 1.0)
        self.declare_parameter("target_start_position_tolerance", 0.75)
        self.declare_parameter("target_start_velocity_tolerance", 0.75)
        self.declare_parameter("pursuer_takeoff_position_tolerance", 0.75)
        self.declare_parameter("pursuer_takeoff_velocity_tolerance", 0.75)
        self.declare_parameter("pursuer_system_id", 1)
        self.declare_parameter("target_system_id", 2)
        self.declare_parameter("record_data", True)
        self.declare_parameter("record_output_dir", "outputs/gazebo2d")
        self.declare_parameter("debug_log", False)
        self.declare_parameter("debug_log_period_s", 0.2)
        self.declare_parameter("startup_log_period_s", 1.0)
        # 导引输入来源与视觉估计参数；默认 odometry，行为与旧版一致。
        self.declare_parameter("target_source", "odometry")
        self.declare_parameter("vision_topic", "/vision/target_pose")
        self.declare_parameter("vision_fallback", "none")
        self.declare_parameter("vision_max_age_s", 0.5)
        self.declare_parameter("vision_alpha", 0.85)
        self.declare_parameter("vision_beta", 0.25)
        self.declare_parameter("vision_accel_tau_s", 0.5)
        self.declare_parameter("vision_gate_sigma", 0.0)
        self.declare_parameter("vision_coast_s", 0.3)
        self.declare_parameter("vision_loss_s", 1.0)
        self.declare_parameter("vision_hold_on_loss", True)
        self.declare_parameter("min_dt_s", 0.01)
        self.declare_parameter("max_dt_s", 0.5)

    def _load_parameters(self) -> None:
        self._algorithm = str(self.get_parameter("algorithm").value)
        self._scenario = str(self.get_parameter("scenario").value)
        if self._algorithm not in ALGORITHMS:
            raise ValueError(f"Unknown algorithm {self._algorithm!r}; expected one of {ALGORITHMS}")
        if self._scenario not in SCENARIOS:
            raise ValueError(f"Unknown scenario {self._scenario!r}; expected one of {SCENARIOS}")

        self._control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        if self._control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be positive")

        dt = float(self.get_parameter("dt").value)
        sim_time = float(self.get_parameter("sim_time").value)
        self._pursuer_fixed_altitude = float(self.get_parameter("pursuer_fixed_altitude").value)
        self._config = SimulationConfig(dt=dt, sim_time=sim_time)
        self._config.pursuer.fixed_altitude = self._pursuer_fixed_altitude
        self._config.pursuer.initial_position[2] = self._pursuer_fixed_altitude

        self._target_base_altitude = float(self.get_parameter("target_base_altitude").value)
        self._load_vision_parameters()
        self._pursuer_namespace = str(self.get_parameter("pursuer_namespace").value)
        self._target_namespace = str(self.get_parameter("target_namespace").value)
        self._auto_arm = _as_bool(self.get_parameter("auto_arm").value)
        self._auto_offboard = _as_bool(self.get_parameter("auto_offboard").value)
        self._offboard_warmup_cycles = int(self.get_parameter("offboard_warmup_cycles").value)
        self._target_start_position_tolerance = float(self.get_parameter("target_start_position_tolerance").value)
        self._target_start_velocity_tolerance = float(self.get_parameter("target_start_velocity_tolerance").value)
        self._pursuer_takeoff_position_tolerance = float(
            self.get_parameter("pursuer_takeoff_position_tolerance").value
        )
        self._pursuer_takeoff_velocity_tolerance = float(
            self.get_parameter("pursuer_takeoff_velocity_tolerance").value
        )
        self._pursuer_system_id = int(self.get_parameter("pursuer_system_id").value)
        self._target_system_id = int(self.get_parameter("target_system_id").value)
        self._record_data = _as_bool(self.get_parameter("record_data").value)
        self._record_output_dir = resolve_recording_path(str(self.get_parameter("record_output_dir").value))
        self._debug_log = _as_bool(self.get_parameter("debug_log").value)
        self._debug_log_period_s = float(self.get_parameter("debug_log_period_s").value)
        self._startup_log_period_s = float(self.get_parameter("startup_log_period_s").value)
        if self._pursuer_takeoff_position_tolerance <= 0.0:
            raise ValueError("pursuer_takeoff_position_tolerance must be positive")
        if self._pursuer_takeoff_velocity_tolerance < 0.0:
            raise ValueError("pursuer_takeoff_velocity_tolerance must be non-negative")
        if self._debug_log_period_s <= 0.0:
            raise ValueError("debug_log_period_s must be positive")
        if self._startup_log_period_s <= 0.0:
            raise ValueError("startup_log_period_s must be positive")

    def _load_vision_parameters(self) -> None:
        """校验视觉导引参数并构造 α-β 滤波器配置（odometry 模式也要求参数合法）。"""
        self._target_source = str(self.get_parameter("target_source").value)
        if self._target_source not in ("odometry", "vision"):
            raise ValueError(f"target_source 必须是 odometry 或 vision，收到 {self._target_source!r}")

        self._vision_topic = str(self.get_parameter("vision_topic").value)
        if not self._vision_topic.startswith("/"):
            raise ValueError("vision_topic 必须是绝对话题名")

        self._vision_fallback = str(self.get_parameter("vision_fallback").value)
        if self._vision_fallback not in ("none", "odometry"):
            raise ValueError(f"vision_fallback 必须是 none 或 odometry，收到 {self._vision_fallback!r}")
        if self._vision_fallback != "none":
            # 本轮不允许静默退回 odometry；保留参数是为了未来远距离捕获场景。
            raise ValueError("vision_fallback=odometry 预留给后续远距离捕获/视觉移交，本轮只支持 none")

        self._vision_max_age_s = self._finite_float("vision_max_age_s")
        if self._vision_max_age_s <= 0.0:
            raise ValueError("vision_max_age_s 必须为正")

        alpha = self._finite_float("vision_alpha")
        beta = self._finite_float("vision_beta")
        if not 0.0 < alpha <= 1.0:
            raise ValueError("vision_alpha 必须落在 (0, 1]")
        # α-β 滤波器稳定域：0 < beta < 4 - 2*alpha。
        if not 0.0 < beta < 4.0 - 2.0 * alpha:
            raise ValueError(f"vision_beta 必须落在 (0, {4.0 - 2.0 * alpha:.2f}) 内以保证 α-β 稳定")

        accel_tau_s = self._finite_float("vision_accel_tau_s")
        if accel_tau_s < 0.0:
            raise ValueError("vision_accel_tau_s 必须非负（0 表示关闭加速度前馈）")
        gate_sigma = self._finite_float("vision_gate_sigma")
        if gate_sigma < 0.0:
            raise ValueError("vision_gate_sigma 必须非负（0 表示关闭门控）")

        coast_s = self._finite_float("vision_coast_s")
        loss_s = self._finite_float("vision_loss_s")
        if coast_s <= 0.0 or loss_s <= coast_s:
            raise ValueError("要求 0 < vision_coast_s < vision_loss_s")

        min_dt_s = self._finite_float("min_dt_s")
        max_dt_s = self._finite_float("max_dt_s")
        if min_dt_s <= 0.0 or max_dt_s < min_dt_s:
            raise ValueError("要求 0 < min_dt_s <= max_dt_s")

        self._vision_hold_on_loss = _as_bool(self.get_parameter("vision_hold_on_loss").value)
        self._vision_filter_config = TargetFilterConfig(
            alpha=alpha,
            beta=beta,
            accel_tau_s=accel_tau_s,
            gate_sigma=gate_sigma,
            coast_s=coast_s,
            loss_s=loss_s,
            min_dt_s=min_dt_s,
            max_dt_s=max_dt_s,
            plane_z=self._target_base_altitude,
        )

    def _finite_float(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            raise ValueError(f"{name} 必须是有限数")
        return value

    def _pursuer_odometry_callback(self, message: VehicleOdometry) -> None:
        position = ned_to_enu_vector(message.position)
        velocity = ned_to_enu_vector(message.velocity)
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            return

        self._pursuer = PursuerState(
            position=position,
            velocity=velocity,
            acceleration=self._memory.previous_acceleration.copy(),
            yaw=yaw_from_quaternion_ned(message.q),
        )

    def _target_odometry_callback(self, message: VehicleOdometry) -> None:
        position = ned_to_enu_vector(message.position)
        velocity = ned_to_enu_vector(message.velocity)
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            return

        self._target = TargetState(position=position, velocity=velocity, acceleration=np.zeros(3))

    def _vision_pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        stamp_s = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        position = np.array(
            [message.pose.pose.position.x, message.pose.pose.position.y],
            dtype=float,
        )
        if not np.all(np.isfinite(position)):
            return
        covariance = np.asarray(message.pose.covariance, dtype=float)
        if covariance.shape == (36,):
            covariance_xy = covariance.reshape(6, 6)[np.ix_([0, 1], [0, 1])]
        else:
            covariance_xy = None
        self._latest_vision = (stamp_s, position, covariance_xy)
        self._vision_received += 1

    def _clock_now_s(self) -> float:
        """当前节点时间（秒）；视觉模式要求 use_sim_time=true，测试可覆盖。"""
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _timer_callback(self) -> None:
        if self._pursuer is None or self._target is None:
            if not self._waiting_logged:
                self.get_logger().info("waiting for pursuer and target VehicleOdometry before publishing setpoints")
                self._waiting_logged = True
            return

        now_us = timestamp_us()
        target_start = self._target_start_reference()

        if not self._startup_ready():
            self._prepare_vehicles_for_tracking(now_us, target_start)
            return

        if not self._tracking_started:
            self._start_tracking()

        self._run_tracking_cycle(now_us)

    def _startup_ready(self) -> bool:
        return self._target_ready and self._pursuer_ready

    def _prepare_vehicles_for_tracking(self, timestamp: int, target_start: TargetState) -> None:
        self._ensure_pursuer_takeoff_position()

        self._publish_target_setpoint(timestamp, target_start)
        self._publish_pursuer_takeoff_setpoint(timestamp, target_start.position)
        self._publish_target_mode_commands(timestamp)
        self._publish_pursuer_mode_commands(timestamp)
        self._update_startup_readiness(target_start)
        self._maybe_log_startup_status(target_start)

    def _run_tracking_cycle(self, timestamp: int) -> None:
        elapsed = self._elapsed_seconds()
        target_reference = self._gazebo_target_reference(elapsed)

        if self._target_source == "vision":
            target_state, snapshot = self._vision_target_state()
        else:
            target_state, snapshot = self._target, _VisionSnapshot()
        if target_state is None:
            self._run_hold_cycle(timestamp, elapsed, snapshot)
            return

        guidance = compute_guidance(
            self._algorithm,
            self._pursuer,
            target_state,
            self._memory,
            self._config,
            self._config.dt,
        )

        applied_acceleration = clamp_norm_xy(guidance.acceleration, self._config.pursuer.a_max)
        self._memory.previous_acceleration = applied_acceleration.copy()
        self._record_sample(elapsed, applied_acceleration, snapshot)

        self._publish_target_setpoint(timestamp, target_reference)
        desired_velocity, acceleration_setpoint, pursuer_yaw_ned = self._publish_pursuer_setpoint(
            timestamp,
            applied_acceleration,
            guidance.look_at_position,
        )
        self._maybe_log_debug(
            elapsed,
            target_reference,
            guidance.acceleration,
            applied_acceleration,
            desired_velocity,
            acceleration_setpoint,
            pursuer_yaw_ned,
            snapshot,
        )

    def _vision_target_state(self) -> tuple[TargetState | None, _VisionSnapshot]:
        """消费最新视觉量测并外推到当前时刻；返回 `(导引目标状态, 快照)`。

        `lost` 且 `vision_hold_on_loss=true` 时返回 None，由调用方进入 hold；
        本轮不允许回退 odometry，`vision_fallback` 只接受 none。
        """
        assert self._vision_tracker is not None
        now_s = self._clock_now_s()
        snapshot = _VisionSnapshot()

        if self._latest_vision is not None:
            stamp_s, position, covariance = self._latest_vision
            snapshot.latency_s = now_s - stamp_s
            if stamp_s != self._consumed_vision_stamp_s:
                self._consumed_vision_stamp_s = stamp_s
                if snapshot.latency_s > self._vision_max_age_s:
                    self._vision_stale_drops += 1
                elif self._vision_tracker.update(stamp_s, position, covariance):
                    self._vision_measurements += 1
                else:
                    self._vision_filter_rejects += 1

        estimate = self._vision_tracker.predict(now_s)
        snapshot.state = estimate.state
        snapshot.age_s = estimate.age_s
        snapshot.measurements = self._vision_measurements
        snapshot.valid = estimate.initialized and estimate.state != STATE_LOST
        if estimate.initialized:
            snapshot.position_xy = estimate.position[:2].copy()
            snapshot.velocity_xy = estimate.velocity[:2].copy()
            snapshot.acceleration_xy = estimate.acceleration[:2].copy()
        if not estimate.initialized:
            return None, snapshot
        if estimate.state == STATE_LOST and self._vision_hold_on_loss:
            return None, snapshot

        target_state = TargetState(
            np.array([estimate.position[0], estimate.position[1], self._target_base_altitude], dtype=float),
            np.array([estimate.velocity[0], estimate.velocity[1], 0.0], dtype=float),
            np.array([estimate.acceleration[0], estimate.acceleration[1], 0.0], dtype=float),
        )
        return target_state, snapshot

    def _run_hold_cycle(self, timestamp: int, elapsed: float, snapshot: _VisionSnapshot) -> None:
        """丢失/未初始化时的悬停：零速零加速度 setpoint，保持当前 yaw。"""
        self._record_sample(elapsed, np.zeros(3), snapshot)
        self._publish_target_setpoint(timestamp, self._gazebo_target_reference(elapsed))

        yaw_ned = yaw_enu_to_ned(self._pursuer.yaw)
        self._pursuer_offboard_pub.publish(offboard_control_mode(timestamp, velocity=True, acceleration=True))
        self._pursuer_setpoint_pub.publish(
            trajectory_setpoint(
                timestamp,
                velocity=enu_to_ned_list(np.zeros(3)),
                acceleration=enu_to_ned_list(np.zeros(3)),
                yaw=yaw_ned,
            )
        )
        self._maybe_log_hold(elapsed, snapshot)

    def _maybe_log_hold(self, elapsed: float, snapshot: _VisionSnapshot) -> None:
        now_ns = time.monotonic_ns()
        if self._last_hold_log_ns is not None:
            if (now_ns - self._last_hold_log_ns) * 1e-9 < self._startup_log_period_s:
                return
        first = self._last_hold_log_ns is None
        self._last_hold_log_ns = now_ns
        message = (
            f"vision hold: t={elapsed:.2f}s state={snapshot.state} "
            f"age={snapshot.age_s:.3f}s latency={snapshot.latency_s:.3f}s "
            f"accepted={snapshot.measurements} received={self._vision_received} "
            f"stale={self._vision_stale_drops} gated={self._vision_filter_rejects}"
        )
        if first:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def _target_start_reference(self) -> TargetState:
        start = self._gazebo_target_reference(0.0)
        return TargetState(start.position.copy(), np.zeros(3), np.zeros(3))

    def _gazebo_target_reference(self, elapsed: float) -> TargetState:
        """目标 XY 使用 7 的 2D 轨迹，高度固定在离地 1m。"""
        reference = target_state(self._scenario, elapsed, self._config)
        position = reference.position.copy()
        velocity = reference.velocity.copy()
        acceleration = reference.acceleration.copy()

        position[2] = self._target_base_altitude
        velocity[2] = 0.0
        acceleration[2] = 0.0

        return TargetState(position, velocity, acceleration)

    def _ensure_pursuer_takeoff_position(self) -> None:
        if self._pursuer_takeoff_position is not None:
            return

        self._pursuer_takeoff_position = self._pursuer.position.copy()
        if self._target_source == "vision":
            # 视觉闭环的初始捕获：目标机准备阶段停在场景起点，追踪机先飞至其上方再开始跟踪。
            # 否则目标可能在窄视场（8 m 高度下目标平面足印约 17 x 12 m）之外，永远等不到第一帧量测。
            # 场景起点是仿真的先验线索（代替外部引导/视觉移交），不依赖 spawn 位置。
            target_start = self._target_start_reference().position
            self._pursuer_takeoff_position[:2] = target_start[:2]
        self._pursuer_takeoff_position[2] = self._pursuer_fixed_altitude
        self.get_logger().info(
            f"locked pursuer takeoff setpoint: p={self._format_vector(self._pursuer_takeoff_position)} "
            f"(target_source={self._target_source})"
        )

    def _update_startup_readiness(self, target_start: TargetState) -> None:
        if not self._target_ready:
            self._target_ready = self._target_commands_done() and self._target_at_start(target_start.position)
            if self._target_ready and not self._target_ready_logged:
                self._target_ready_logged = True
                self.get_logger().info("target ready at scenario start")

        if not self._pursuer_ready:
            self._pursuer_ready = self._pursuer_commands_done() and self._pursuer_at_takeoff_position()
            if self._pursuer_ready and not self._pursuer_ready_logged:
                self._pursuer_ready_logged = True
                self.get_logger().info("pursuer ready at fixed takeoff altitude")

    def _start_tracking(self) -> None:
        self._tracking_started = True
        self._active_start_ns = None
        self._last_debug_log_ns = None
        self._last_startup_log_ns = None
        self._last_hold_log_ns = None
        self._memory.previous_acceleration = np.zeros(3)
        if self._vision_tracker is not None:
            # 追踪开始前的量测不参与闭环，重置估计器和消费指针，等第一个新量测。
            self._vision_tracker.reset()
            self._latest_vision = None
            self._consumed_vision_stamp_s = None
        self.get_logger().info(
            f"both vehicles ready; starting 2D tracking and data recording (target_source={self._target_source})"
        )

    def _record_sample(
        self,
        elapsed: float,
        acceleration_enu: np.ndarray,
        snapshot: _VisionSnapshot,
    ) -> None:
        if not self._record_data:
            return
        if self._record_samples and elapsed <= self._record_samples[-1].elapsed_s:
            return

        distance_xy = norm_xy(self._target.position - self._pursuer.position)
        sample = _TrackingSample(
            elapsed_s=float(elapsed),
            pursuer_position=self._pursuer.position.copy(),
            pursuer_velocity=self._pursuer.velocity.copy(),
            target_position=self._target.position.copy(),
            target_velocity=self._target.velocity.copy(),
            acceleration=acceleration_enu.copy(),
            yaw=float(self._pursuer.yaw),
            distance=float(distance_xy),
            target_source=self._target_source,
        )
        if self._target_source == "vision":
            sample.vision_valid = 1.0 if snapshot.valid else 0.0
            sample.vision_age_s = snapshot.age_s
            sample.vision_latency_s = snapshot.latency_s
            sample.vision_measurements = snapshot.measurements
            sample.target_est_x = float(snapshot.position_xy[0])
            sample.target_est_y = float(snapshot.position_xy[1])
            sample.target_est_vx = float(snapshot.velocity_xy[0])
            sample.target_est_vy = float(snapshot.velocity_xy[1])
            sample.target_est_ax = float(snapshot.acceleration_xy[0])
            sample.target_est_ay = float(snapshot.acceleration_xy[1])
            if snapshot.valid:
                sample.vision_error_xy = float(norm_xy(snapshot.position_xy - self._target.position[:2]))
        self._record_samples.append(sample)

    def save_recording(self) -> None:
        if not self._record_data:
            return
        if not self._record_samples:
            self._log_shutdown_safe("warn", "record_data is enabled, but no Gazebo 2D samples were collected")
            return

        try:
            result = self._recording_result()
            output_dir = self._record_output_dir / self._scenario / self._algorithm
            output_dir.mkdir(parents=True, exist_ok=True)
            self._write_recording_csv(result, output_dir / "gazebo_samples.csv")
            self._log_shutdown_safe("info", f"saved Gazebo 2D recording CSV to {output_dir / 'gazebo_samples.csv'}")
        except Exception as exc:  # noqa: BLE001
            self._log_shutdown_safe("error", f"failed to save Gazebo 2D recording: {exc}")

    def _log_shutdown_safe(self, level: str, message: str) -> None:
        if rclpy.ok():
            getattr(self.get_logger(), level)(message)
            return
        stream = sys.stderr if level == "error" else sys.stdout
        print(f"[{level.upper()}] [guidance_node_2d]: {message}", file=stream)

    def _recording_result(self) -> SimulationResult:
        return SimulationResult(
            scenario=self._scenario,
            algorithm=self._algorithm,
            time=np.array([sample.elapsed_s for sample in self._record_samples], dtype=float),
            pursuer_position=np.array([sample.pursuer_position for sample in self._record_samples]),
            pursuer_velocity=np.array([sample.pursuer_velocity for sample in self._record_samples]),
            target_position=np.array([sample.target_position for sample in self._record_samples]),
            target_velocity=np.array([sample.target_velocity for sample in self._record_samples]),
            acceleration=np.array([sample.acceleration for sample in self._record_samples]),
            yaw=np.array([sample.yaw for sample in self._record_samples], dtype=float),
            distance=np.array([sample.distance for sample in self._record_samples], dtype=float),
        )

    def _write_recording_csv(self, result: SimulationResult, path: Path) -> None:
        fieldnames = (
            "time",
            "pursuer_x",
            "pursuer_y",
            "pursuer_z",
            "pursuer_vx",
            "pursuer_vy",
            "pursuer_vz",
            "target_x",
            "target_y",
            "target_z",
            "target_vx",
            "target_vy",
            "target_vz",
            "acceleration_x",
            "acceleration_y",
            "acceleration_z",
            "yaw",
            "distance_xy",
            # 视觉闭环新增列：target_x/y 仍是 odometry 真值，估计值单独成列。
            "guidance_target_source",
            "vision_valid",
            "vision_age_s",
            "vision_latency_s",
            "vision_measurements",
            "target_est_x",
            "target_est_y",
            "target_est_vx",
            "target_est_vy",
            "target_est_ax",
            "target_est_ay",
            "vision_error_xy",
        )
        with path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            for index, time_value in enumerate(result.time):
                sample = self._record_samples[index]
                writer.writerow(
                    {
                        "time": float(time_value),
                        "pursuer_x": float(result.pursuer_position[index, 0]),
                        "pursuer_y": float(result.pursuer_position[index, 1]),
                        "pursuer_z": float(result.pursuer_position[index, 2]),
                        "pursuer_vx": float(result.pursuer_velocity[index, 0]),
                        "pursuer_vy": float(result.pursuer_velocity[index, 1]),
                        "pursuer_vz": float(result.pursuer_velocity[index, 2]),
                        "target_x": float(result.target_position[index, 0]),
                        "target_y": float(result.target_position[index, 1]),
                        "target_z": float(result.target_position[index, 2]),
                        "target_vx": float(result.target_velocity[index, 0]),
                        "target_vy": float(result.target_velocity[index, 1]),
                        "target_vz": float(result.target_velocity[index, 2]),
                        "acceleration_x": float(result.acceleration[index, 0]),
                        "acceleration_y": float(result.acceleration[index, 1]),
                        "acceleration_z": float(result.acceleration[index, 2]),
                        "yaw": float(result.yaw[index]),
                        "distance_xy": float(result.distance[index]),
                        "guidance_target_source": sample.target_source,
                        "vision_valid": sample.vision_valid,
                        "vision_age_s": sample.vision_age_s,
                        "vision_latency_s": sample.vision_latency_s,
                        "vision_measurements": sample.vision_measurements,
                        "target_est_x": sample.target_est_x,
                        "target_est_y": sample.target_est_y,
                        "target_est_vx": sample.target_est_vx,
                        "target_est_vy": sample.target_est_vy,
                        "target_est_ax": sample.target_est_ax,
                        "target_est_ay": sample.target_est_ay,
                        "vision_error_xy": sample.vision_error_xy,
                    }
                )

    def _elapsed_seconds(self) -> float:
        now_ns = self.get_clock().now().nanoseconds
        if self._active_start_ns is None:
            self._active_start_ns = now_ns
        elapsed = (now_ns - self._active_start_ns) * 1e-9
        if self._config.sim_time > 0.0:
            return min(elapsed, self._config.sim_time)
        return elapsed

    def _publish_target_setpoint(self, timestamp: int, target_reference: TargetState) -> None:
        speed_xy = float(np.linalg.norm(target_reference.velocity[:2]))
        if speed_xy > 0.05:
            self._target_yaw_enu = float(np.arctan2(target_reference.velocity[1], target_reference.velocity[0]))

        self._target_offboard_pub.publish(offboard_control_mode(timestamp, position=True, velocity=True, acceleration=False))
        self._target_setpoint_pub.publish(
            trajectory_setpoint(
                timestamp,
                position=enu_to_ned_list(target_reference.position),
                velocity=enu_to_ned_list(target_reference.velocity),
                yaw=yaw_enu_to_ned(self._target_yaw_enu),
            )
        )

    def _publish_pursuer_setpoint(
        self,
        timestamp: int,
        acceleration_enu: np.ndarray,
        look_at_position_enu: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """把 2D guidance 加速度转换成追踪机速度 + 加速度 setpoint。"""
        acceleration_enu = acceleration_enu.copy()
        acceleration_enu[2] = 0.0

        desired_velocity = self._pursuer.velocity.copy()
        desired_velocity[2] = 0.0
        desired_velocity = clamp_norm_xy(desired_velocity + acceleration_enu * self._config.dt, self._config.pursuer.v_max)

        yaw_ned = yaw_to_target_ned(self._pursuer.position, look_at_position_enu)
        self._pursuer_offboard_pub.publish(offboard_control_mode(timestamp, velocity=True, acceleration=True))
        self._pursuer_setpoint_pub.publish(
            trajectory_setpoint(
                timestamp,
                velocity=enu_to_ned_list(desired_velocity),
                acceleration=enu_to_ned_list(acceleration_enu),
                yaw=yaw_ned,
            )
        )
        return desired_velocity.copy(), acceleration_enu.copy(), yaw_ned

    def _publish_pursuer_takeoff_setpoint(self, timestamp: int, look_at_position_enu: np.ndarray) -> None:
        if self._pursuer_takeoff_position is None:
            raise RuntimeError("pursuer takeoff position has not been initialized")

        yaw_ned = yaw_to_target_ned(self._pursuer.position, look_at_position_enu)
        self._pursuer_offboard_pub.publish(offboard_control_mode(timestamp, position=True, velocity=True, acceleration=False))
        self._pursuer_setpoint_pub.publish(
            trajectory_setpoint(
                timestamp,
                position=enu_to_ned_list(self._pursuer_takeoff_position),
                velocity=enu_to_ned_list(np.zeros(3)),
                yaw=yaw_ned,
            )
        )

    def _maybe_log_debug(
        self,
        elapsed: float,
        target_reference: TargetState,
        guidance_acceleration_enu: np.ndarray,
        applied_acceleration_enu: np.ndarray,
        desired_velocity_enu: np.ndarray,
        acceleration_setpoint_enu: np.ndarray,
        pursuer_yaw_ned: float,
        snapshot: _VisionSnapshot,
    ) -> None:
        if not self._debug_log:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self._last_debug_log_ns is not None:
            since_last = (now_ns - self._last_debug_log_ns) * 1e-9
            if since_last < self._debug_log_period_s:
                return
        self._last_debug_log_ns = now_ns

        self.get_logger().info(
            "debug_2d "
            f"t={elapsed:.2f}s | "
            f"target_odom p={self._format_vector(self._target.position)} "
            f"v={self._format_vector(self._target.velocity)} "
            f"a={self._format_vector(self._target.acceleration)} | "
            f"target_cmd p={self._format_vector(target_reference.position)} "
            f"v={self._format_vector(target_reference.velocity)} "
            f"a={self._format_vector(target_reference.acceleration)} "
            f"yaw_ned={yaw_enu_to_ned(self._target_yaw_enu):+.3f} | "
            f"pursuer_odom p={self._format_vector(self._pursuer.position)} "
            f"v={self._format_vector(self._pursuer.velocity)} "
            f"a={self._format_vector(self._pursuer.acceleration)} | "
            "pursuer_cmd mode=velocity+acceleration "
            f"v_sp={self._format_vector(desired_velocity_enu)} "
            f"a_sp={self._format_vector(acceleration_setpoint_enu)} "
            f"raw_a={self._format_vector(guidance_acceleration_enu)} "
            f"applied_a={self._format_vector(applied_acceleration_enu)} "
            f"yaw_ned={pursuer_yaw_ned:+.3f} | "
            f"vision source={self._target_source} state={snapshot.state} "
            f"valid={snapshot.valid} age={snapshot.age_s:.3f}s latency={snapshot.latency_s:.3f}s "
            f"accepted={snapshot.measurements} received={self._vision_received} "
            f"stale={self._vision_stale_drops} gated={self._vision_filter_rejects} "
            f"est_p={self._format_xy(snapshot.position_xy)} "
            f"est_v={self._format_xy(snapshot.velocity_xy)}"
        )

    @staticmethod
    def _format_xy(vector: np.ndarray) -> str:
        value = np.asarray(vector, dtype=float)
        return f"[{value[0]:+.2f}, {value[1]:+.2f}]"

    def _maybe_log_startup_status(self, target_start: TargetState) -> None:
        if self._pursuer_takeoff_position is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self._last_startup_log_ns is not None:
            since_last = (now_ns - self._last_startup_log_ns) * 1e-9
            if since_last < self._startup_log_period_s:
                return
        self._last_startup_log_ns = now_ns

        target_error = float(np.linalg.norm(self._target.position - target_start.position))
        target_speed = float(np.linalg.norm(self._target.velocity))
        pursuer_error = float(np.linalg.norm(self._pursuer.position - self._pursuer_takeoff_position))
        pursuer_speed = float(np.linalg.norm(self._pursuer.velocity))
        self.get_logger().info(
            "startup_2d "
            f"target_ready={self._target_ready} "
            f"target_p={self._format_vector(self._target.position)} "
            f"target_err={target_error:.2f} target_speed={target_speed:.2f} "
            f"target_commands_done={self._target_commands_done()} | "
            f"pursuer_ready={self._pursuer_ready} "
            f"pursuer_p={self._format_vector(self._pursuer.position)} "
            f"pursuer_err={pursuer_error:.2f} pursuer_speed={pursuer_speed:.2f} "
            f"pursuer_commands_done={self._pursuer_commands_done()}"
        )

    @staticmethod
    def _format_vector(vector: np.ndarray) -> str:
        value = np.asarray(vector, dtype=float)
        return f"[{value[0]:+.2f}, {value[1]:+.2f}, {value[2]:+.2f}]"

    def _target_at_start(self, start_position_enu: np.ndarray) -> bool:
        return self._state_at_setpoint(
            self._target,
            start_position_enu,
            self._target_start_position_tolerance,
            self._target_start_velocity_tolerance,
        )

    def _pursuer_at_takeoff_position(self) -> bool:
        if self._pursuer_takeoff_position is None:
            return False
        return self._state_at_setpoint(
            self._pursuer,
            self._pursuer_takeoff_position,
            self._pursuer_takeoff_position_tolerance,
            self._pursuer_takeoff_velocity_tolerance,
        )

    def _state_at_setpoint(
        self,
        state: PursuerState | TargetState,
        desired_position: np.ndarray,
        position_tolerance: float,
        velocity_tolerance: float,
    ) -> bool:
        position_error = float(np.linalg.norm(state.position - desired_position))
        speed = float(np.linalg.norm(state.velocity))
        return position_error <= position_tolerance and speed <= velocity_tolerance

    def _target_commands_done(self) -> bool:
        return (not self._auto_arm or self._target_arm_sent) and (
            not self._auto_offboard or self._target_offboard_sent
        )

    def _pursuer_commands_done(self) -> bool:
        return (not self._auto_arm or self._pursuer_arm_sent) and (
            not self._auto_offboard or self._pursuer_offboard_sent
        )

    def _publish_target_mode_commands(self, timestamp: int) -> None:
        self._target_offboard_cycles += 1
        if self._target_offboard_cycles < self._offboard_warmup_cycles:
            return

        if self._auto_arm and not self._target_arm_sent:
            self._target_command_pub.publish(arm_command(timestamp, self._target_system_id, arm=True))
            self._target_arm_sent = True
            self.get_logger().info("sent arm command for target")

        if self._auto_offboard and not self._target_offboard_sent:
            self._target_command_pub.publish(offboard_mode_command(timestamp, self._target_system_id))
            self._target_offboard_sent = True
            self.get_logger().info("sent offboard mode command for target")

    def _publish_pursuer_mode_commands(self, timestamp: int) -> None:
        self._pursuer_offboard_cycles += 1
        if self._pursuer_offboard_cycles < self._offboard_warmup_cycles:
            return

        if self._auto_arm and not self._pursuer_arm_sent:
            self._pursuer_command_pub.publish(arm_command(timestamp, self._pursuer_system_id, arm=True))
            self._pursuer_arm_sent = True
            self.get_logger().info("sent arm command for pursuer")

        if self._auto_offboard and not self._pursuer_offboard_sent:
            self._pursuer_command_pub.publish(offboard_mode_command(timestamp, self._pursuer_system_id))
            self._pursuer_offboard_sent = True
            self.get_logger().info("sent offboard mode command for pursuer")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GuidanceNode()
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
