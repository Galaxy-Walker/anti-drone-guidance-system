"""把轻量级导引算法接入 PX4/Gazebo 的 ROS 2 节点。

该节点控制两个 PX4 实例：

- pursuer：运行所选追踪导引算法的追踪机。
- target：按照合成目标轨迹飞行的目标机。

所有算法计算都沿用 `pythonsimulation` 中的 ENU 坐标系。PX4 输入/输出只在
边界处转换，这样无需维护第二套 Gazebo 专用算法实现。
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import numpy as np


def _ensure_pythonsimulation_on_path() -> None:
    """在未安装包时，让同级 `pythonsimulation` 也能被导入。

    开发阶段可能会直接从源码树运行该 ROS 包，而不是先通过 colcon 安装。
    向上查找路径可以同时兼容 `6_Simulation/src/pythonsimulation` 和安装后的包路径。
    """
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
    """兼容 ROS launch 传入的 `true`、`false` 等字符串布尔值。"""
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

from pythonsimulation.config import ALGORITHMS, SCENARIOS, SimulationConfig
from pythonsimulation.guidance import GuidanceMemory, compute_guidance
from pythonsimulation.math_utils import clamp_norm
from pythonsimulation.state import PursuerState, SimulationResult, TargetState
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
    """面向一架追踪机和一架目标机的 PX4 Offboard 桥接节点。"""

    def __init__(self) -> None:
        super().__init__("guidance_node")
        self._declare_parameters()
        self._load_parameters()

        # GuidanceMemory 保存需要跨控制周期保留的算法状态，例如 FOV 算法的最后一次
        # 观测目标状态，以及 NMPC/MPPI 平滑代价需要的上一帧加速度。
        self._memory = GuidanceMemory()

        # 最新里程计会被转换成 pythonsimulation 的状态对象。定时器会等两架机状态都
        # 准备好后才发第一条命令，避免基于出生点假设而不是真实 PX4 状态控制。
        self._pursuer: PursuerState | None = None
        self._target: TargetState | None = None

        # 目标轨迹时钟从两路里程计都可用后才开始，避免 Gazebo/PX4 启动期间目标参考
        # 已经提前向前推进。
        self._active_start_ns: int | None = None

        # PX4 在接受 Offboard 模式切换前，需要先收到若干 OffboardControlMode 和
        # TrajectorySetpoint 消息。这些标志确保预热结束后 arm/offboard 命令只发送一次。
        self._offboard_cycles = 0
        self._arm_sent = False
        self._offboard_sent = False

        # 当目标参考速度接近 0 时复用上一帧 yaw，避免静止目标因为 atan2 数值噪声乱转。
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
                float,
                bool,
                float,
            ]
        ] = []

        # PX4 uXRCE-DDS 车辆话题通常使用 best-effort 和 transient local；发布/订阅端
        # 使用一致 QoS，可以提高不同 PX4/px4_msgs 组合下的话题匹配成功率。
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 追踪机发布器：该机接收所选导引算法的输出。
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

        # 目标机发布器：目标机同样切入 Offboard，但它跟随合成场景轨迹，而不是导引算法输出。
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

        # 追踪机导引使用目标机真实里程计，而不是理想目标参考，因此 Gazebo 跟踪误差会被算法看到。
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

        # 这里使用固定频率定时器即可：算法每个控制周期读取最新里程计并输出一条 setpoint。
        self.create_timer(1.0 / self._control_rate_hz, self._timer_callback)
        self.get_logger().info(
            f"guidance_node ready: algorithm={self._algorithm}, scenario={self._scenario}, "
            f"pursuer={self._pursuer_namespace}, target={self._target_namespace}"
        )

    def _declare_parameters(self) -> None:
        """声明可被 launch 覆盖的参数及其开发默认值。"""
        # 算法和场景名称会根据 pythonsimulation.config 做合法性校验。
        self.declare_parameter("algorithm", "pn_fov_mppi")
        self.declare_parameter("scenario", "circle")

        # 控制频率和 PX4 namespace。namespace 必须匹配 SITL 实例和 Micro XRCE-DDS 桥接话题。
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("pursuer_namespace", "/px4_1")
        self.declare_parameter("target_namespace", "/px4_2")

        # 自动命令是可选的，调试时可以关闭它们，改为在 QGC 中手动解锁/切入 Offboard。
        self.declare_parameter("auto_arm", True)
        self.declare_parameter("auto_offboard", True)
        self.declare_parameter("offboard_warmup_cycles", 20)

        # sim_time 限制目标参考时间；dt 同时用于导引计算和本桥接节点中的简单速度积分。
        self.declare_parameter("sim_time", 40.0)
        self.declare_parameter("dt", 0.05)

        # VehicleCommand 的目标 system id 必须匹配两个 PX4 实例。
        self.declare_parameter("pursuer_system_id", 1)
        self.declare_parameter("target_system_id", 2)

        # Gazebo 实飞/仿真数据记录：节点只保存原始 CSV，绘图由普通 Python 脚本后处理。
        self.declare_parameter("record_data", True)
        self.declare_parameter("record_output_dir", "outputs/gazebo")

    def _load_parameters(self) -> None:
        """读取 ROS 参数，校验选项，并创建共享配置。"""
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
        # 保持一个 SimulationConfig 实例，使 Gazebo 桥接行为尽量贴近离线 Python 对比仿真。
        self._config = SimulationConfig(dt=dt, sim_time=sim_time)
        self._pursuer_namespace = str(self.get_parameter("pursuer_namespace").value)
        self._target_namespace = str(self.get_parameter("target_namespace").value)
        self._auto_arm = _as_bool(self.get_parameter("auto_arm").value)
        self._auto_offboard = _as_bool(self.get_parameter("auto_offboard").value)
        self._offboard_warmup_cycles = int(self.get_parameter("offboard_warmup_cycles").value)
        self._pursuer_system_id = int(self.get_parameter("pursuer_system_id").value)
        self._target_system_id = int(self.get_parameter("target_system_id").value)
        self._record_data = _as_bool(self.get_parameter("record_data").value)
        self._record_output_dir = Path(str(self.get_parameter("record_output_dir").value)).expanduser()

    def _pursuer_odometry_callback(self, message: VehicleOdometry) -> None:
        """把追踪机 PX4 NED 里程计转换到 ENU 后缓存。"""
        position = ned_to_enu_vector(message.position)
        velocity = ned_to_enu_vector(message.velocity)
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            # PX4 启动阶段可能发布无效样本；直接忽略，避免把 NaN 喂给导引算法。
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
        """把目标机 PX4 NED 里程计转换到 ENU 后缓存。"""
        position = ned_to_enu_vector(message.position)
        velocity = ned_to_enu_vector(message.velocity)
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            return

        self._target = TargetState(position=position, velocity=velocity, acceleration=np.zeros(3))

    def _timer_callback(self) -> None:
        """执行一个控制周期：更新目标 setpoint、导引输出和 PX4 命令。"""
        if self._pursuer is None or self._target is None:
            if not self._waiting_logged:
                self.get_logger().info("waiting for pursuer and target VehicleOdometry before publishing offboard setpoints")
                self._waiting_logged = True
            return

        now_us = timestamp_us(self)
        elapsed = self._elapsed_seconds()

        # 目标机跟随理想场景参考；追踪机则使用下面的 PX4 目标里程计，因此目标跟踪误差会进入闭环行为。
        target_reference = target_state(self._scenario, elapsed, self._config)
        guidance = compute_guidance(
            self._algorithm,
            self._pursuer,
            self._target,
            self._memory,
            self._config,
            self._config.dt,
        )

        # 离线仿真在动力学中使用同样的加速度限制；桥接节点在转换为 PX4 setpoint 前先限幅。
        applied_acceleration = clamp_norm(guidance.acceleration, self._config.pursuer.a_max)
        self._memory.previous_acceleration = applied_acceleration.copy()
        self._record_sample(elapsed, applied_acceleration, guidance.visible, guidance.los_angle)

        self._publish_target_setpoint(now_us, target_reference)
        self._publish_pursuer_setpoint(now_us, applied_acceleration, guidance.look_at_position)
        self._publish_mode_commands(now_us)

    def _record_sample(
        self,
        elapsed: float,
        acceleration_enu: np.ndarray,
        visible: bool,
        los_angle: float,
    ) -> None:
        """按控制周期保存一帧 Gazebo/PX4 闭环数据，供退出时写入 CSV。"""
        if not self._record_data:
            return
        if self._record_samples and elapsed <= self._record_samples[-1][0]:
            return

        distance = float(np.linalg.norm(self._target.position - self._pursuer.position))
        self._record_samples.append(
            (
                float(elapsed),
                self._pursuer.position.copy(),
                self._pursuer.velocity.copy(),
                self._target.position.copy(),
                self._target.velocity.copy(),
                acceleration_enu.copy(),
                float(self._pursuer.yaw),
                float(self._pursuer.pitch),
                distance,
                bool(visible),
                float(los_angle),
            )
        )

    def save_recording(self) -> None:
        """节点退出时只把 Gazebo 话题数据保存成 CSV。"""
        if not self._record_data:
            return
        if not self._record_samples:
            self._log_shutdown_safe("warn", "record_data is enabled, but no Gazebo samples were collected")
            return

        try:
            result = self._recording_result()
            output_dir = self._record_output_dir / self._scenario / self._algorithm
            output_dir.mkdir(parents=True, exist_ok=True)

            self._write_recording_csv(result, output_dir / "gazebo_samples.csv")

            self._log_shutdown_safe("info", f"saved Gazebo recording CSV to {output_dir / 'gazebo_samples.csv'}")
        except Exception as exc:  # noqa: BLE001 - 退出阶段不能让保存失败吞掉 shutdown。
            self._log_shutdown_safe("error", f"failed to save Gazebo recording: {exc}")

    def _log_shutdown_safe(self, level: str, message: str) -> None:
        """launch 收到 Ctrl+C 时 ROS context 可能已失效，此时改用 stdout/stderr。"""
        if rclpy.ok():
            getattr(self.get_logger(), level)(message)
            return
        stream = sys.stderr if level == "error" else sys.stdout
        print(f"[{level.upper()}] [guidance_node]: {message}", file=stream)

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
            pitch=np.array([sample[7] for sample in self._record_samples], dtype=float),
            distance=np.array([sample[8] for sample in self._record_samples], dtype=float),
            visible=np.array([sample[9] for sample in self._record_samples], dtype=bool),
            los_angle=np.array([sample[10] for sample in self._record_samples], dtype=float),
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
            "pitch",
            "distance",
            "visible",
            "los_angle",
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
                        "pitch": float(result.pitch[index]),
                        "distance": float(result.distance[index]),
                        "visible": bool(result.visible[index]),
                        "los_angle": float(result.los_angle[index]),
                    }
                )

    def _elapsed_seconds(self) -> float:
        """返回场景时间，从第一个有效控制周期开始计时。"""
        now_ns = self.get_clock().now().nanoseconds
        if self._active_start_ns is None:
            self._active_start_ns = now_ns
        elapsed = (now_ns - self._active_start_ns) * 1e-9
        if self._config.sim_time > 0.0:
            return min(elapsed, self._config.sim_time)
        return elapsed

    def _publish_target_setpoint(self, timestamp: int, target_reference: TargetState) -> None:
        """发布目标机 Offboard 心跳和位置/速度 setpoint。"""
        speed_xy = float(np.linalg.norm(target_reference.velocity[:2]))
        if speed_xy > 0.05:
            self._target_yaw_enu = float(np.arctan2(target_reference.velocity[1], target_reference.velocity[0]))

        # 目标机使用位置+速度控制，尽量按照 PX4/Gazebo 能力跟随合成轨迹。
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
        """根据导引输出发布追踪机速度/加速度 setpoint。"""
        # PX4 接受速度和加速度 setpoint，而 pythonsimulation 导引输出的是加速度。
        # 这里积分一个 dt 得到兼容的速度目标，并按共享的 v_max 约束限幅。
        desired_velocity = self._pursuer.velocity + acceleration_enu * self._config.dt
        desired_velocity = clamp_norm(desired_velocity, self._config.pursuer.v_max)
        desired_velocity = self._limit_vertical_velocity(self._pursuer.position, desired_velocity)

        # 让机头/相机指向算法使用的同一个参考目标。对 FOV 方法而言，目标丢失时这里会指向最后观测预测点。
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
        """防止简单 setpoint 积分命令 z 超出高度边界。"""
        limited = velocity_enu.copy()
        next_z = position_enu[2] + limited[2] * self._config.dt
        if next_z < self._config.pursuer.z_min:
            # 如果下一步会低于下限，只允许向上运动。
            limited[2] = max(0.0, (self._config.pursuer.z_min - position_enu[2]) / self._config.dt)
        if next_z > self._config.pursuer.z_max:
            # 如果下一步会高于上限，只允许向下运动。
            limited[2] = min(0.0, (self._config.pursuer.z_max - position_enu[2]) / self._config.dt)
        return limited

    def _publish_mode_commands(self, timestamp: int) -> None:
        """PX4 收到预热 setpoint 后，一次性发送 arm/offboard 命令。"""
        self._offboard_cycles += 1
        if self._offboard_cycles < self._offboard_warmup_cycles:
            # 如果 PX4 尚未持续收到有效 OffboardControlMode/TrajectorySetpoint 消息，
            # 它会拒绝切入 Offboard 模式。
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
    """ROS 2 console script 入口。"""
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
