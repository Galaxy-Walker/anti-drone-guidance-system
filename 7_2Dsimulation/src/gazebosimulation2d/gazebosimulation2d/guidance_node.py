"""把 2D 定高导引算法接入 PX4/Gazebo 的 ROS 2 节点。

节点沿用 6_Simulation 的双 PX4 实例接口：

- pursuer：追踪机，使用 `pythonsimulation2d` 的导引算法，并发布 XY 位置 setpoint + 固定高度。
- target：目标机，沿用 6 的目标机/话题/模型，按合成目标参考轨迹飞行。

导引、距离和记录指标都按 XY 平面计算；高度只用于 Gazebo/PX4 setpoint。
"""

from __future__ import annotations

import csv
import sys
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
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from pythonsimulation2d.config import ALGORITHMS, SCENARIOS, SimulationConfig
from pythonsimulation2d.guidance import GuidanceMemory, compute_guidance
from pythonsimulation2d.math_utils import clamp_norm_xy, norm_xy
from pythonsimulation2d.state import PursuerState, SimulationResult, TargetState
from pythonsimulation2d.target import target_state

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


class GuidanceNode(Node):
    """面向一架追踪机和一架目标机的 2D PX4 Offboard 桥接节点。"""

    def __init__(self) -> None:
        super().__init__("guidance_node_2d")
        self._declare_parameters()
        self._load_parameters()

        self._memory = GuidanceMemory()
        self._pursuer: PursuerState | None = None
        self._target: TargetState | None = None
        self._active_start_ns: int | None = None

        self._target_offboard_cycles = 0
        self._pursuer_offboard_cycles = 0
        self._target_arm_sent = False
        self._target_offboard_sent = False
        self._pursuer_arm_sent = False
        self._pursuer_offboard_sent = False
        self._target_ready = False
        self._pursuer_ready = False

        self._target_yaw_enu = 0.0
        self._waiting_logged = False
        self._record_samples: list[
            tuple[
                float,
                np.ndarray,
                np.ndarray,
                np.ndarray,
                np.ndarray,
                np.ndarray,
                float,
                float,
            ]
        ] = []

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

        self.create_timer(1.0 / self._control_rate_hz, self._timer_callback)
        self.get_logger().info(
            f"guidance_node_2d ready: algorithm={self._algorithm}, scenario={self._scenario}, "
            f"pursuer={self._pursuer_namespace}, target={self._target_namespace}, "
            f"pursuer_fixed_altitude={self._pursuer_fixed_altitude:.2f}m"
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
        self.declare_parameter("pursuer_system_id", 1)
        self.declare_parameter("target_system_id", 2)
        self.declare_parameter("record_data", True)
        self.declare_parameter("record_output_dir", "outputs/gazebo2d")

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
        self._pursuer_namespace = str(self.get_parameter("pursuer_namespace").value)
        self._target_namespace = str(self.get_parameter("target_namespace").value)
        self._auto_arm = _as_bool(self.get_parameter("auto_arm").value)
        self._auto_offboard = _as_bool(self.get_parameter("auto_offboard").value)
        self._offboard_warmup_cycles = int(self.get_parameter("offboard_warmup_cycles").value)
        self._target_start_position_tolerance = float(self.get_parameter("target_start_position_tolerance").value)
        self._target_start_velocity_tolerance = float(self.get_parameter("target_start_velocity_tolerance").value)
        self._pursuer_system_id = int(self.get_parameter("pursuer_system_id").value)
        self._target_system_id = int(self.get_parameter("target_system_id").value)
        self._record_data = _as_bool(self.get_parameter("record_data").value)
        self._record_output_dir = Path(str(self.get_parameter("record_output_dir").value)).expanduser()

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

    def _timer_callback(self) -> None:
        if self._pursuer is None or self._target is None:
            if not self._waiting_logged:
                self.get_logger().info("waiting for pursuer and target VehicleOdometry before publishing setpoints")
                self._waiting_logged = True
            return

        now_us = timestamp_us(self)
        target_start = self._target_start_reference()

        if not self._target_ready:
            self._prepare_target_at_start(now_us, target_start)
            return

        if not self._pursuer_ready:
            self._prepare_pursuer_for_tracking(now_us, target_start)
            return

        elapsed = self._elapsed_seconds()
        target_reference = self._gazebo_target_reference(elapsed)
        guidance = compute_guidance(
            self._algorithm,
            self._pursuer,
            self._target,
            self._memory,
            self._config,
            self._config.dt,
        )

        applied_acceleration = clamp_norm_xy(guidance.acceleration, self._config.pursuer.a_max)
        self._memory.previous_acceleration = applied_acceleration.copy()
        self._record_sample(elapsed, applied_acceleration)

        self._publish_target_setpoint(now_us, target_reference)
        self._publish_pursuer_setpoint(now_us, applied_acceleration, guidance.look_at_position)

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

    def _prepare_target_at_start(self, timestamp: int, target_start: TargetState) -> None:
        self._publish_target_setpoint(timestamp, target_start)
        self._publish_target_mode_commands(timestamp)

        if self._target_commands_done() and self._target_at_start(target_start.position):
            self._target_ready = True
            self.get_logger().info("target reached scenario start; preparing pursuer")

    def _prepare_pursuer_for_tracking(self, timestamp: int, target_start: TargetState) -> None:
        self._publish_target_setpoint(timestamp, target_start)
        self._publish_pursuer_hold_setpoint(timestamp, target_start.position)
        self._publish_pursuer_mode_commands(timestamp)

        if self._pursuer_commands_done():
            self._pursuer_ready = True
            self._active_start_ns = None
            self.get_logger().info("pursuer ready; starting 2D tracking and data recording")

    def _record_sample(
        self,
        elapsed: float,
        acceleration_enu: np.ndarray,
    ) -> None:
        if not self._record_data:
            return
        if self._record_samples and elapsed <= self._record_samples[-1][0]:
            return

        distance_xy = norm_xy(self._target.position - self._pursuer.position)
        self._record_samples.append(
            (
                float(elapsed),
                self._pursuer.position.copy(),
                self._pursuer.velocity.copy(),
                self._target.position.copy(),
                self._target.velocity.copy(),
                acceleration_enu.copy(),
                float(self._pursuer.yaw),
                float(distance_xy),
            )
        )

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
        times = np.array([sample[0] for sample in self._record_samples], dtype=float)
        return SimulationResult(
            scenario=self._scenario,
            algorithm=self._algorithm,
            time=times,
            pursuer_position=np.array([sample[1] for sample in self._record_samples]),
            pursuer_velocity=np.array([sample[2] for sample in self._record_samples]),
            target_position=np.array([sample[3] for sample in self._record_samples]),
            target_velocity=np.array([sample[4] for sample in self._record_samples]),
            acceleration=np.array([sample[5] for sample in self._record_samples]),
            yaw=np.array([sample[6] for sample in self._record_samples], dtype=float),
            distance=np.array([sample[7] for sample in self._record_samples], dtype=float),
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
        )
        with path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            for index, time_value in enumerate(result.time):
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
    ) -> None:
        """把 2D guidance 加速度转换成追踪机 XY 位置 setpoint + 固定高度。"""
        desired_velocity = self._pursuer.velocity.copy()
        desired_velocity[2] = 0.0
        desired_velocity = clamp_norm_xy(desired_velocity + acceleration_enu * self._config.dt, self._config.pursuer.v_max)

        desired_position = self._pursuer.position.copy()
        desired_position[:2] = desired_position[:2] + desired_velocity[:2] * self._config.dt
        desired_position[2] = self._pursuer_fixed_altitude

        yaw_ned = yaw_to_target_ned(self._pursuer.position, look_at_position_enu)
        self._pursuer_offboard_pub.publish(offboard_control_mode(timestamp, position=True, velocity=True, acceleration=False))
        self._pursuer_setpoint_pub.publish(
            trajectory_setpoint(
                timestamp,
                position=enu_to_ned_list(desired_position),
                velocity=enu_to_ned_list(desired_velocity),
                yaw=yaw_ned,
            )
        )

    def _publish_pursuer_hold_setpoint(self, timestamp: int, look_at_position_enu: np.ndarray) -> None:
        hold_position = self._pursuer.position.copy()
        hold_position[2] = self._pursuer_fixed_altitude
        yaw_ned = yaw_to_target_ned(self._pursuer.position, look_at_position_enu)
        self._pursuer_offboard_pub.publish(offboard_control_mode(timestamp, position=True, velocity=True, acceleration=False))
        self._pursuer_setpoint_pub.publish(
            trajectory_setpoint(
                timestamp,
                position=enu_to_ned_list(hold_position),
                velocity=enu_to_ned_list(np.zeros(3)),
                yaw=yaw_ned,
            )
        )

    def _target_at_start(self, start_position_enu: np.ndarray) -> bool:
        position_error = float(np.linalg.norm(self._target.position - start_position_enu))
        speed = float(np.linalg.norm(self._target.velocity))
        return (
            position_error <= self._target_start_position_tolerance
            and speed <= self._target_start_velocity_tolerance
        )

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
