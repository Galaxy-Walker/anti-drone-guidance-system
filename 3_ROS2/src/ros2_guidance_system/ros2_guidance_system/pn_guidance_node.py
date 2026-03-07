#!/usr/bin/env python3
"""
比例导引(PN)节点

基于 ROS2 + PX4 Offboard 模式的无人机拦截控制节点。
通过参数化配置支持仿真测试和真实飞行两种场景。

状态机流程:
  初始化(INIT) -> 起飞(TAKEOFF) -> 导引追踪(GUIDANCE) -> 降落(LAND)

话题:
  发布:
    /fmu/in/offboard_control_mode   - 离板控制模式心跳
    /fmu/in/trajectory_setpoint     - 轨迹设定点
    /fmu/in/vehicle_command         - 飞控指令（解锁、模式切换等）
  订阅:
    /fmu/out/vehicle_local_position_v1 - 无人机本地位置
    /fmu/out/vehicle_status_v1         - 无人机状态
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
import numpy as np
import math

# 导入核心算法和目标源（完全解耦的模块）
from .pn_guidance_core import PNGuidanceCore, State, GuidanceResult
from .target_provider import SimulatedTarget, RosTopicTarget


def _get_param(node: Node, name: str):
    """读取 ROS2 参数值的辅助函数，避免 Pyright 类型警告。"""
    val = node.get_parameter(name).value
    assert val is not None, f"参数 '{name}' 不应为 None"
    return val


class PNGuidanceNode(Node):
    """
    比例导引 ROS2 节点。

    职责：
      1. 与 PX4 通信（发布指令、订阅状态）
      2. 管理飞行状态机（起飞->导引->降落）
      3. 调用解耦的导引核心和目标源模块

    所有算法参数通过 ROS2 参数声明，可在 launch 文件或命令行中配置。
    """

    def __init__(self):
        super().__init__('pn_guidance_node')

        # ============================================================
        # 声明 ROS2 参数（可通过 launch 文件或命令行覆盖）
        # ============================================================
        # 导引算法参数
        self.declare_parameter('guidance.N', 4.0)               # 导引比例系数
        self.declare_parameter('guidance.speed_min', 2.0)       # 最小速度 (m/s)
        self.declare_parameter('guidance.speed_max', 30.0)      # 最大速度 (m/s)
        self.declare_parameter('guidance.strategy', 'pursuit') # 速度策略

        # 飞行参数
        self.declare_parameter('flight.takeoff_height', 10.0)   # 起飞高度 (m)，正值
        self.declare_parameter('flight.intercept_radius', 2.0)  # 拦截判定距离 (m)
        self.declare_parameter('flight.control_freq', 20.0)     # 控制频率 (Hz)

        # 目标源参数
        self.declare_parameter('target.source', 'simulated')    # 目标源类型: simulated / ros_topic
        self.declare_parameter('target.motion_type', 'static')  # 仿真运动类型
        self.declare_parameter('target.center_x', 50.0)         # 圆周中心 X
        self.declare_parameter('target.center_y', 50.0)         # 圆周中心 Y
        self.declare_parameter('target.radius', 30.0)           # 圆周半径 (m)
        self.declare_parameter('target.omega', 0.1)             # 圆周角速度 (rad/s)
        self.declare_parameter('target.altitude', 20.0)         # 目标高度 (m)，正值
        self.declare_parameter('target.altitude_amplitude', 5.0) # 高度振荡幅度 (m)，仅 circle_altitude
        self.declare_parameter('target.altitude_omega', 0.2)    # 高度振荡角频率 (rad/s)，仅 circle_altitude
        self.declare_parameter('target.line_vx', 2.0)           # 直线运动速度 X (m/s)
        self.declare_parameter('target.line_vy', 2.0)           # 直线运动速度 Y (m/s)
        self.declare_parameter('target.line_vz', 0.0)           # 直线运动速度 Z (m/s)
        self.declare_parameter('target.start_x', 50.0)          # 初始位置 X (m)
        self.declare_parameter('target.start_y', 50.0)          # 初始位置 Y (m)
        self.declare_parameter('target.timeout', 2.0)           # ROS话题超时 (s)

        # 调试参数
        self.declare_parameter('debug.log_interval', 20)        # 日志打印间隔（每 N 个周期打印一次）
        self.declare_parameter('debug.verbose', True)          # 是否启用详细日志

        # ============================================================
        # 读取参数
        # ============================================================
        guidance_N = float(_get_param(self, 'guidance.N'))
        speed_min = float(_get_param(self, 'guidance.speed_min'))
        speed_max = float(_get_param(self, 'guidance.speed_max'))
        strategy = str(_get_param(self, 'guidance.strategy'))

        self.takeoff_height = -abs(float(_get_param(self, 'flight.takeoff_height')))  # 转为 NED（负值）
        self.intercept_radius = float(_get_param(self, 'flight.intercept_radius'))
        control_freq = float(_get_param(self, 'flight.control_freq'))
        self.dt = 1.0 / control_freq

        target_source = str(_get_param(self, 'target.source'))
        self.log_interval = int(_get_param(self, 'debug.log_interval'))
        self.verbose = bool(_get_param(self, 'debug.verbose'))

        self.get_logger().info("=" * 60)
        self.get_logger().info("  比例导引节点启动")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"当前是否开启详细日志(verbose): {self.verbose}")

        # ============================================================
        # 配置 QoS（PX4 micro-XRCE-DDS 要求的配置）
        # ============================================================
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ============================================================
        # 创建发布者
        # ============================================================
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.get_logger().info("发布者已创建: offboard_control_mode, trajectory_setpoint, vehicle_command")

        # ============================================================
        # 创建订阅者
        # ============================================================
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._on_vehicle_local_position, qos_profile)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self._on_vehicle_status, qos_profile)

        self.get_logger().info("订阅者已创建: vehicle_local_position, vehicle_status")

        # ============================================================
        # 内部状态
        # ============================================================
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        self.flight_phase = "INIT"  # 状态机: INIT -> TAKEOFF -> GUIDANCE -> LAND
        self._position_received = False  # 是否已收到位置数据

        # 追踪者（己方无人机）状态
        self.tracker_state = State()

        # ============================================================
        # 初始化导引核心（与 ROS2 解耦）
        # ============================================================
        self.guidance_core = PNGuidanceCore(
            N=guidance_N,
            speed_min=speed_min,
            speed_max=speed_max,
            strategy=strategy,
            log_func=self.get_logger().info,
        )

        # ============================================================
        # 初始化目标源（与 ROS2 解耦）
        # ============================================================
        if target_source == 'simulated':
            target_config = {
                'motion_type': str(_get_param(self, 'target.motion_type')),
                'center': [
                    float(_get_param(self, 'target.center_x')),
                    float(_get_param(self, 'target.center_y')),
                ],
                'radius': float(_get_param(self, 'target.radius')),
                'omega': float(_get_param(self, 'target.omega')),
                'altitude': -abs(float(_get_param(self, 'target.altitude'))),
                'altitude_amplitude': float(_get_param(self, 'target.altitude_amplitude')),
                'altitude_omega': float(_get_param(self, 'target.altitude_omega')),
                'line_velocity': [
                    float(_get_param(self, 'target.line_vx')),
                    float(_get_param(self, 'target.line_vy')),
                    float(_get_param(self, 'target.line_vz')),
                ],
                'start_position': [
                    float(_get_param(self, 'target.start_x')),
                    float(_get_param(self, 'target.start_y')),
                    -abs(float(_get_param(self, 'target.altitude'))),
                ],
            }
            self.target_provider = SimulatedTarget.from_config(
                target_config, log_func=self.get_logger().info,
            )
            self.get_logger().info(f"仿真目标源已初始化，运动类型: {target_config['motion_type']}")
        elif target_source == 'ros_topic':
            self.target_provider = RosTopicTarget(
                timeout_sec=float(_get_param(self, 'target.timeout')),
                log_func=self.get_logger().warn,
            )
            # TODO: 在此处创建目标话题的订阅，在回调中调用 self.target_provider.set_state(...)
            self.get_logger().warn("ROS话题目标源已启用，但目标订阅尚未实现，请根据实际传感器话题进行扩展")
        else:
            self.get_logger().error(f"未知的目标源类型: {target_source}，将使用仿真目标")
            self.target_provider = SimulatedTarget(log_func=self.get_logger().info)

        # ============================================================
        # 打印参数汇总
        # ============================================================
        self.get_logger().info(f"参数配置:")
        self.get_logger().info(f"  导引系数 N = {guidance_N}")
        self.get_logger().info(f"  速度范围 = [{speed_min}, {speed_max}] m/s")
        self.get_logger().info(f"  速度策略 = {strategy}")
        self.get_logger().info(f"  起飞高度 = {abs(self.takeoff_height):.1f} m")
        self.get_logger().info(f"  拦截判定距离 = {self.intercept_radius} m")
        self.get_logger().info(f"  控制频率 = {control_freq} Hz (dt={self.dt:.3f}s)")
        self.get_logger().info(f"  目标源 = {target_source}")
        self.get_logger().info(f"  日志间隔 = 每 {self.log_interval} 周期")

        # ============================================================
        # 启动控制回路定时器
        # ============================================================
        self.timer = self.create_timer(self.dt, self._control_loop)
        self.get_logger().info(f"控制回路定时器已启动，等待飞控连接...")

    # ================================================================
    # 订阅回调
    # ================================================================

    def _on_vehicle_local_position(self, msg: VehicleLocalPosition):
        """接收无人机本地位置数据。"""
        self.vehicle_local_position = msg
        self.tracker_state.position = np.array([msg.x, msg.y, msg.z])
        self.tracker_state.velocity = np.array([msg.vx, msg.vy, msg.vz])
        self.tracker_state.speed = float(np.linalg.norm(self.tracker_state.velocity))

        if not self._position_received:
            self._position_received = True
            self.get_logger().info(
                f"首次收到位置数据: "
                f"位置=[{msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f}], "
                f"速度=[{msg.vx:.1f}, {msg.vy:.1f}, {msg.vz:.1f}]"
            )

    def _on_vehicle_status(self, msg: VehicleStatus):
        """接收无人机状态数据。"""
        self.vehicle_status = msg
        if self.verbose:
            self.get_logger().debug(
                f"飞控状态: 导航模式={msg.nav_state}, 解锁状态={msg.arming_state}"
            )

    # ================================================================
    # PX4 指令发送
    # ================================================================

    def _publish_vehicle_command(self, command, **params):
        """发送飞控指令。"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def _arm(self):
        """发送解锁指令。"""
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('已发送解锁指令')

    def _engage_offboard_mode(self):
        """切换到离板(Offboard)控制模式。"""
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("已发送切换离板模式指令")

    def _land(self):
        """切换到降落模式。"""
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("已发送降落指令")

    # ================================================================
    # 控制信号发布
    # ================================================================

    def _publish_offboard_heartbeat(self):
        """发布离板控制模式心跳信号（必须持续发送以维持离板模式）。"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True   # 同时启用位置和速度控制
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def _publish_trajectory_setpoint(self, pos_cmd: np.ndarray, vel_cmd: np.ndarray):
        """
        发布轨迹设定点。

        参数:
            pos_cmd: 期望位置 [x, y, z] (NED)
            vel_cmd: 期望速度 [vx, vy, vz] (NED)
        """
        msg = TrajectorySetpoint()
        msg.position = [float(pos_cmd[0]), float(pos_cmd[1]), float(pos_cmd[2])]
        msg.velocity = [float(vel_cmd[0]), float(vel_cmd[1]), float(vel_cmd[2])]

        # 偏航角指向速度方向（水平分量足够大时）
        horizontal_speed = np.linalg.norm(vel_cmd[0:2])
        if horizontal_speed > 0.1:
            msg.yaw = float(math.atan2(vel_cmd[1], vel_cmd[0]))
        else:
            msg.yaw = float('nan')  # 保持当前偏航角

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    # ================================================================
    # 主控制回路（状态机）
    # ================================================================

    def _control_loop(self):
        """主控制回路回调函数，由定时器触发。"""

        # 始终发送心跳以维持离板模式
        self._publish_offboard_heartbeat()

        # ---- 启动阶段：等待足够的心跳信号后切换模式并解锁 ----
        if self.offboard_setpoint_counter == 10:
            self._engage_offboard_mode()
            self._arm()
            self.flight_phase = "TAKEOFF"
            self.get_logger().info("状态机切换: INIT -> TAKEOFF")

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
            # 初始阶段发送原点作为设定点（满足离板模式进入条件）
            self._publish_trajectory_setpoint(
                np.array([0.0, 0.0, 0.0]),
                np.array([0.0, 0.0, 0.0])
            )
            return

        # ---- 状态机主逻辑 ----
        if self.flight_phase == "TAKEOFF":
            self._handle_takeoff()
        elif self.flight_phase == "GUIDANCE":
            self._handle_guidance()
        elif self.flight_phase == "LAND":
            pass  # 降落由 PX4 自主控制，无需额外操作

    def _handle_takeoff(self):
        """处理起飞阶段：爬升到指定高度。"""
        self._publish_trajectory_setpoint(
            np.array([0.0, 0.0, self.takeoff_height]),
            np.array([0.0, 0.0, 0.0])
        )

        # NED 坐标系: Z 轴向下为正，高度越高 Z 越小（越负）
        # 判断条件：当前 Z 值（0.5m 容差）和离板模式状态
        current_z = self.vehicle_local_position.z
        is_at_height = current_z <= self.takeoff_height + 0.5
        is_offboard = self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        if self.offboard_setpoint_counter % self.log_interval == 0:
            self.get_logger().info(
                f"[起飞] 当前高度={-current_z:.1f}m, "
                f"目标高度={-self.takeoff_height:.1f}m, "
                f"离板模式={'是' if is_offboard else '否'}"
            )

        if is_at_height and is_offboard:
            self.get_logger().info(
                f"起飞完成！当前高度 {-current_z:.1f}m，开始导引追踪"
            )
            self.flight_phase = "GUIDANCE"
            self.get_logger().info("状态机切换: TAKEOFF -> GUIDANCE")

        self.offboard_setpoint_counter += 1

    def _handle_guidance(self):
        """处理导引追踪阶段：调用 PN 算法计算并发布控制指令。"""

        # 更新目标状态
        self.target_provider.update(self.dt)

        # 检查目标数据有效性
        if not self.target_provider.is_valid():
            self.get_logger().warn("目标数据无效，保持当前位置悬停")
            self._publish_trajectory_setpoint(
                self.tracker_state.position,
                np.array([0.0, 0.0, 0.0])
            )
            self.offboard_setpoint_counter += 1
            return

        target_state = self.target_provider.get_state()

        # 计算距离
        dist = float(np.linalg.norm(target_state.position - self.tracker_state.position))

        if dist < self.intercept_radius:
            # 拦截成功
            self.get_logger().info("=" * 40)
            self.get_logger().info(f"拦截成功！距离={dist:.2f}m，执行降落")
            self.get_logger().info("=" * 40)
            self.flight_phase = "LAND"
            self.get_logger().info("状态机切换: GUIDANCE -> LAND")
            self._land()
        else:
            # 调用 PN 导引核心算法
            result: GuidanceResult = self.guidance_core.calculate_guidance(
                self.tracker_state, target_state, self.dt
            )

            # 发布控制指令
            self._publish_trajectory_setpoint(result.pos_cmd, result.vel_cmd)

            # 周期性打印调试信息
            if self.offboard_setpoint_counter % self.log_interval == 0:
                self.get_logger().info(
                    f"[导引] "
                    f"距离={result.distance:.1f}m, "
                    f"接近速度={result.closing_speed:.1f}m/s, "
                    f"角速率={result.los_rate:.4f}rad/s, "
                    f"期望速率={result.desired_speed:.1f}m/s"
                )
                if self.verbose:
                    self.get_logger().info(
                        f"  追踪者: 位置=[{self.tracker_state.position[0]:.1f}, "
                        f"{self.tracker_state.position[1]:.1f}, "
                        f"{self.tracker_state.position[2]:.1f}], "
                        f"速度=[{self.tracker_state.velocity[0]:.1f}, "
                        f"{self.tracker_state.velocity[1]:.1f}, "
                        f"{self.tracker_state.velocity[2]:.1f}]"
                    )
                    self.get_logger().info(
                        f"  目标: 位置=[{target_state.position[0]:.1f}, "
                        f"{target_state.position[1]:.1f}, "
                        f"{target_state.position[2]:.1f}], "
                        f"速度=[{target_state.velocity[0]:.1f}, "
                        f"{target_state.velocity[1]:.1f}, "
                        f"{target_state.velocity[2]:.1f}]"
                    )
                    self.get_logger().info(
                        f"  指令: 位置=[{result.pos_cmd[0]:.1f}, "
                        f"{result.pos_cmd[1]:.1f}, "
                        f"{result.pos_cmd[2]:.1f}], "
                        f"速度=[{result.vel_cmd[0]:.1f}, "
                        f"{result.vel_cmd[1]:.1f}, "
                        f"{result.vel_cmd[2]:.1f}]"
                    )

        self.offboard_setpoint_counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = PNGuidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断信号 (SIGINT)，正在关闭...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
