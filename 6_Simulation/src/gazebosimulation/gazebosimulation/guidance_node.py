from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


def _ensure_pythonsimulation_on_path() -> None:
    for parent in Path(__file__).resolve().parents:
        src_candidate = parent / "src"
        if (src_candidate / "pythonsimulation").is_dir():
            sys.path.insert(0, str(src_candidate))
            return
        if (parent / "pythonsimulation").is_dir():
            sys.path.insert(0, str(parent))
            return


_ensure_pythonsimulation_on_path()


def _as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in {"1", "true", "yes", "on"}
    return bool(value)

import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from pythonsimulation.config import ALGORITHMS, SCENARIOS, SimulationConfig
from pythonsimulation.guidance import GuidanceMemory, compute_guidance
from pythonsimulation.math_utils import clamp_norm
from pythonsimulation.state import PursuerState, TargetState
from pythonsimulation.target import target_state

from gazebosimulation.coordinates import (
    enu_to_ned_list,
    ned_to_enu_vector,
    yaw_enu_to_ned,
    yaw_pitch_from_quaternion_ned,
    yaw_to_target_ned,
)
from gazebosimulation.px4_utils import (
    arm_command,
    namespaced_topic,
    offboard_control_mode,
    offboard_mode_command,
    timestamp_us,
    trajectory_setpoint,
)


class GuidanceNode(Node):
    def __init__(self) -> None:
        super().__init__("guidance_node")
        self._declare_parameters()
        self._load_parameters()

        self._memory = GuidanceMemory()
        self._pursuer: PursuerState | None = None
        self._target: TargetState | None = None
        self._active_start_ns: int | None = None
        self._offboard_cycles = 0
        self._arm_sent = False
        self._offboard_sent = False
        self._target_yaw_enu = 0.0
        self._waiting_logged = False

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
            f"guidance_node ready: algorithm={self._algorithm}, scenario={self._scenario}, "
            f"pursuer={self._pursuer_namespace}, target={self._target_namespace}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("algorithm", "pn_fov_mppi")
        self.declare_parameter("scenario", "circle")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("pursuer_namespace", "/px4_1")
        self.declare_parameter("target_namespace", "/px4_2")
        self.declare_parameter("auto_arm", True)
        self.declare_parameter("auto_offboard", True)
        self.declare_parameter("offboard_warmup_cycles", 20)
        self.declare_parameter("sim_time", 40.0)
        self.declare_parameter("dt", 0.05)
        self.declare_parameter("pursuer_system_id", 1)
        self.declare_parameter("target_system_id", 2)

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
        self._config = SimulationConfig(dt=dt, sim_time=sim_time)
        self._pursuer_namespace = str(self.get_parameter("pursuer_namespace").value)
        self._target_namespace = str(self.get_parameter("target_namespace").value)
        self._auto_arm = _as_bool(self.get_parameter("auto_arm").value)
        self._auto_offboard = _as_bool(self.get_parameter("auto_offboard").value)
        self._offboard_warmup_cycles = int(self.get_parameter("offboard_warmup_cycles").value)
        self._pursuer_system_id = int(self.get_parameter("pursuer_system_id").value)
        self._target_system_id = int(self.get_parameter("target_system_id").value)

    def _pursuer_odometry_callback(self, message: VehicleOdometry) -> None:
        position = ned_to_enu_vector(message.position)
        velocity = ned_to_enu_vector(message.velocity)
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            return

        yaw, pitch = yaw_pitch_from_quaternion_ned(message.q)
        self._pursuer = PursuerState(
            position=position,
            velocity=velocity,
            acceleration=self._memory.previous_acceleration.copy(),
            yaw=yaw,
            pitch=pitch,
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
                self.get_logger().info("waiting for pursuer and target VehicleOdometry before publishing offboard setpoints")
                self._waiting_logged = True
            return

        now_us = timestamp_us(self)
        elapsed = self._elapsed_seconds()
        target_reference = target_state(self._scenario, elapsed, self._config)
        guidance = compute_guidance(
            self._algorithm,
            self._pursuer,
            self._target,
            self._memory,
            self._config,
            self._config.dt,
        )
        applied_acceleration = clamp_norm(guidance.acceleration, self._config.pursuer.a_max)
        self._memory.previous_acceleration = applied_acceleration.copy()

        self._publish_target_setpoint(now_us, target_reference)
        self._publish_pursuer_setpoint(now_us, applied_acceleration, guidance.look_at_position)
        self._publish_mode_commands(now_us)

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
        desired_velocity = self._pursuer.velocity + acceleration_enu * self._config.dt
        desired_velocity = clamp_norm(desired_velocity, self._config.pursuer.v_max)
        desired_velocity = self._limit_vertical_velocity(self._pursuer.position, desired_velocity)
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

    def _limit_vertical_velocity(self, position_enu: np.ndarray, velocity_enu: np.ndarray) -> np.ndarray:
        limited = velocity_enu.copy()
        next_z = position_enu[2] + limited[2] * self._config.dt
        if next_z < self._config.pursuer.z_min:
            limited[2] = max(0.0, (self._config.pursuer.z_min - position_enu[2]) / self._config.dt)
        if next_z > self._config.pursuer.z_max:
            limited[2] = min(0.0, (self._config.pursuer.z_max - position_enu[2]) / self._config.dt)
        return limited

    def _publish_mode_commands(self, timestamp: int) -> None:
        self._offboard_cycles += 1
        if self._offboard_cycles < self._offboard_warmup_cycles:
            return

        if self._auto_arm and not self._arm_sent:
            self._pursuer_command_pub.publish(arm_command(timestamp, self._pursuer_system_id, arm=True))
            self._target_command_pub.publish(arm_command(timestamp, self._target_system_id, arm=True))
            self._arm_sent = True
            self.get_logger().info("sent arm commands for pursuer and target")

        if self._auto_offboard and not self._offboard_sent:
            self._pursuer_command_pub.publish(offboard_mode_command(timestamp, self._pursuer_system_id))
            self._target_command_pub.publish(offboard_mode_command(timestamp, self._target_system_id))
            self._offboard_sent = True
            self.get_logger().info("sent offboard mode commands for pursuer and target")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GuidanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
