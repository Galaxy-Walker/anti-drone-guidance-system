#!/usr/bin/env python3
"""
反无人机导引节点 — 合并 offboard_px4 安全机制与 pn_guidance 导引算法

状态机（6 状态）:
  INIT → REQUESTING_OFFBOARD → ARMING → OFFBOARD_ACTIVE → (RETURN_REQUESTED / ERROR)
  OFFBOARD_ACTIVE 内嵌导引子阶段: TAKEOFF → GUIDANCE → LAND

安全机制（来自 offboard_px4， 如果处于仿真模式，则安全检测被跳过）:
  - VehicleControlMode 订阅 + 边缘检测，确认离板模式真正生效
  - BatteryStatus 订阅 + 电量监控（低电量警告、安全状态联动）
  - threading.Lock 保护共享状态
  - 各阶段超时检测
  - monitor_safety() 检测意外退出导航模式 / 意外上锁
  - handle_error_state() 错误恢复流程
  - return_flight() RTL 请求 + 确认逻辑

导引算法（来自 pn_guidance_node）:
  - ROS2 参数声明
  - PNGuidanceCore 实例化与 calculate_guidance() 调用
  - SimulatedTarget / RosTopicTarget 工厂创建
  - 轨迹设定点同时发布 位置 + 速度
  - 拦截判定
  - log_interval 周期日志 + verbose 详细调试输出

话题:
  发布:
    /fmu/in/offboard_control_mode   - 离板控制模式心跳
    /fmu/in/trajectory_setpoint     - 轨迹设定点
    /fmu/in/vehicle_command         - 飞控指令
  订阅:
    /fmu/out/vehicle_local_position(_v1)  - 本地位置，兼容不同 PX4/px4_msgs 话题命名
    /fmu/out/vehicle_status(_v1)          - 飞控状态，兼容不同 PX4/px4_msgs 话题命名
    /fmu/out/vehicle_control_mode       - 控制模式确认
    /fmu/out/battery_status             - 电池状态
"""

from enum import Enum, auto
import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry

from px4_msgs.msg import (
    BatteryStatus,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleControlMode,
    VehicleLocalPosition,
    VehicleStatus,
)
from std_msgs.msg import String

import numpy as np
import math

from .pn_guidance_core import PNGuidanceCore, State, GuidanceResult
from .target_provider import SimulatedTarget, RosTopicTarget


LOCAL_POSITION_TOPICS = [
    '/fmu/out/vehicle_local_position_v1',
    '/fmu/out/vehicle_local_position',
]
VEHICLE_STATUS_TOPICS = [
    '/fmu/out/vehicle_status_v1',
    '/fmu/out/vehicle_status',
]


def _get_param(node: Node, name: str):
    val = node.get_parameter(name).value
    assert val is not None, f"参数 '{name}' 不应为 None"
    return val


class FlightState(Enum):
    """飞行状态机"""
    INIT = auto()
    REQUESTING_OFFBOARD = auto()
    ARMING = auto()
    OFFBOARD_ACTIVE = auto()
    RETURN_REQUESTED = auto()
    ERROR = auto()


class AntiDroneGuidanceNode(Node):
    """反无人机导引节点"""

    def __init__(self):
        super().__init__('pn_guidance_node')

        # ---- 线程锁 ----
        self._lock = threading.Lock()

        # ============================================================
        # 声明 ROS2 参数
        # ============================================================
        self.declare_parameter('guidance.N', 4.0)
        self.declare_parameter('guidance.speed_min', 2.0)
        self.declare_parameter('guidance.speed_max', 30.0)
        self.declare_parameter('guidance.strategy', 'adaptive')

        self.declare_parameter('flight.takeoff_height', 10.0)
        self.declare_parameter('flight.intercept_radius', 2.0)
        self.declare_parameter('flight.control_freq', 20.0)

        self.declare_parameter('target.source', 'simulated')
        self.declare_parameter('target.motion_type', 'static')
        self.declare_parameter('target.center_x', 50.0)
        self.declare_parameter('target.center_y', 50.0)
        self.declare_parameter('target.radius', 30.0)
        self.declare_parameter('target.omega', 0.1)
        self.declare_parameter('target.altitude', 20.0)
        self.declare_parameter('target.altitude_amplitude', 5.0)
        self.declare_parameter('target.altitude_omega', 0.2)
        self.declare_parameter('target.line_vx', 2.0)
        self.declare_parameter('target.line_vy', 2.0)
        self.declare_parameter('target.line_vz', 0.0)
        self.declare_parameter('target.start_x', 50.0)
        self.declare_parameter('target.start_y', 50.0)
        self.declare_parameter('target.initial_phase', 0.0)
        self.declare_parameter('target.timeout', 2.0)

        self.declare_parameter('evaluation.case_id', '')
        self.declare_parameter('evaluation.enable_topics', False)

        self.declare_parameter('debug.log_interval', 30)
        self.declare_parameter('debug.verbose', True)

        # ============================================================
        # 读取参数
        # ============================================================
        guidance_N = float(_get_param(self, 'guidance.N'))
        speed_min = float(_get_param(self, 'guidance.speed_min'))
        speed_max = float(_get_param(self, 'guidance.speed_max'))
        strategy = str(_get_param(self, 'guidance.strategy'))

        self.takeoff_height = -abs(float(_get_param(self, 'flight.takeoff_height')))
        self.intercept_radius = float(_get_param(self, 'flight.intercept_radius'))
        control_freq = float(_get_param(self, 'flight.control_freq'))
        self.dt = 1.0 / control_freq

        target_source = str(_get_param(self, 'target.source'))
        self._simulation_mode = (target_source == 'simulated')
        self.evaluation_case_id = str(_get_param(self, 'evaluation.case_id'))
        self.evaluation_enable_topics = bool(_get_param(self, 'evaluation.enable_topics'))
        self.log_interval = int(_get_param(self, 'debug.log_interval'))
        self.verbose = bool(_get_param(self, 'debug.verbose'))

        self.get_logger().info("=" * 60)
        self.get_logger().info("  反无人机导引节点启动")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"当前是否开启详细日志(verbose): {self.verbose}")

        # ============================================================
        # 配置 QoS
        # ============================================================
        px4_in_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        px4_out_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        event_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ============================================================
        # 创建发布者
        # ============================================================
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_in_qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_in_qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_in_qos_profile)
        self.target_state_pub = None
        self.evaluation_event_pub = None
        if self.evaluation_enable_topics:
            # 评估模式下额外发布虚拟目标真值，普通飞行默认不创建该话题。
            self.target_state_pub = self.create_publisher(
                Odometry, '/anti_drone_guidance/target_state', 10)
            # 事件使用可靠 QoS，保证 rosbag 能记录关键阶段切换。
            self.evaluation_event_pub = self.create_publisher(
                String, '/anti_drone_guidance/evaluation_event', event_qos_profile)

        self.get_logger().info("发布者已创建: offboard_control_mode, trajectory_setpoint, vehicle_command")

        # ============================================================
        # 创建订阅者
        # ============================================================
        # local_position/status 在不同 PX4/px4_msgs 组合里可能带 _v1，也可能不带。
        # 同时订阅两个候选名，哪个有数据就用哪个，避免节点因话题名版本差异无法起飞。
        for topic in LOCAL_POSITION_TOPICS:
            self.create_subscription(
                VehicleLocalPosition, topic, self._on_vehicle_local_position, px4_out_qos_profile)
        for topic in VEHICLE_STATUS_TOPICS:
            self.create_subscription(
                VehicleStatus, topic, self._on_vehicle_status, px4_out_qos_profile)
        self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode',
            self._on_vehicle_control_mode, px4_out_qos_profile)
        self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status',
            self._on_battery_status, px4_out_qos_profile)

        self.get_logger().info(
            "订阅者已创建: vehicle_local_position, vehicle_status, vehicle_control_mode, battery_status"
        )

        # ============================================================
        # 内部状态
        # ============================================================
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        self._position_received = False

        self.tracker_state = State()

        # ---- 状态机 ----
        self._current_state = FlightState.INIT
        self._state_start_time = self.get_clock().now()
        self._offboard_timeout = Duration(seconds=2.0)
        self._arm_timeout = Duration(seconds=2.0)

        # ---- 控制模式确认 ----
        self._control_offboard_enabled = False
        self._is_offboard_transitioning = False

        # ---- 电池相关状态 ----
        self._battery_remaining = 1.0
        self._battery_connected = False
        self._battery_warning = 0
        self._safe_to_fly = False
        self._last_battery_warn_time = self.get_clock().now()

        # ---- 返航(RTL)相关 ----
        self._rtl_command_sent = False
        self._rtl_start_time = self.get_clock().now()
        self._rtl_logged = False

        # ---- 安全监控 ----
        self._last_status_check = self.get_clock().now()
        self._recovery_attempts = 0

        # ---- OFFBOARD_ACTIVE 导引子阶段 ----
        self._guidance_subphase = "TAKEOFF"  # TAKEOFF -> GUIDANCE -> LAND

        # ============================================================
        # 初始化导引核心
        # ============================================================
        self.guidance_core = PNGuidanceCore(
            N=guidance_N,
            speed_min=speed_min,
            speed_max=speed_max,
            strategy=strategy,
            log_func=self.get_logger().info,
        )

        # ============================================================
        # 初始化目标源
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
                'initial_phase': float(_get_param(self, 'target.initial_phase')),
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
            # TODO：实现基于 ROS 话题的目标源，订阅实际传感器数据（如雷达、视觉检测等）
            self.target_provider = RosTopicTarget(
                timeout_sec=float(_get_param(self, 'target.timeout')),
                log_func=self.get_logger().warn,
            )
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
        if self.evaluation_enable_topics:
            self.get_logger().info(f"  评估话题 = 启用，case_id = {self.evaluation_case_id}")
        self.get_logger().info(f"  日志间隔 = 每 {self.log_interval} 周期")

        # ============================================================
        # 启动控制回路定时器
        # ============================================================
        self.timer = self.create_timer(self.dt, self._control_loop)
        self.get_logger().info(f"控制回路定时器已启动，等待飞控连接...")
        self._publish_evaluation_event("case_started")

    # ================================================================
    # 订阅回调
    # ================================================================

    def _on_vehicle_local_position(self, msg: VehicleLocalPosition):
        with self._lock:
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
        with self._lock:
            self.vehicle_status = msg
            self._last_status_check = self.get_clock().now()

    def _on_vehicle_control_mode(self, msg: VehicleControlMode):
        transition = None
        with self._lock:
            was_offboard = self._control_offboard_enabled
            now_offboard = bool(msg.flag_control_offboard_enabled)
            self._control_offboard_enabled = now_offboard
            if (not was_offboard) and now_offboard:
                self._is_offboard_transitioning = True
                transition = 'entered'
            elif was_offboard and (not now_offboard):
                transition = 'exited'

        if transition == 'entered':
            self.get_logger().info('controller entered offboard mode (已进入Offboard模式)')
        elif transition == 'exited':
            self.get_logger().warn('controller exited offboard mode (已退出Offboard模式)')

    def _on_battery_status(self, msg: BatteryStatus):
        with self._lock:
            self._battery_connected = bool(msg.connected)
            self._battery_remaining = float(msg.remaining)
            self._battery_warning = int(msg.warning)

    # ================================================================
    # 状态机辅助
    # ================================================================

    def _update_state(self, new_state: FlightState):
        with self._lock:
            old_state = self._current_state
            if old_state == new_state:
                return
            self._current_state = new_state
            self._state_start_time = self.get_clock().now()
        self.get_logger().info(f'状态切换: {old_state.name} -> {new_state.name}')

    def _elapsed_since_state_start(self) -> Duration:
        with self._lock:
            state_start = self._state_start_time
        now = self.get_clock().now()
        return Duration(nanoseconds=(now.nanoseconds - state_start.nanoseconds))

    # ================================================================
    # PX4 指令发送
    # ================================================================

    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0,
                                  param3=0.0, param4=0.0, param5=0.0,
                                  param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def _arm(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('已发送解锁指令')

    def _disarm(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('已发送上锁指令')

    def _engage_offboard_mode(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("已发送切换离板模式指令")

    def _land(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("已发送降落指令")

    def _request_rtl(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH, param1=0.0, param2=0.0)
        self.get_logger().info("已发送返航(RTL)指令")

    # ================================================================
    # 控制信号发布
    # ================================================================

    def _publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def _publish_trajectory_setpoint(self, pos_cmd: np.ndarray, vel_cmd: np.ndarray):
        msg = TrajectorySetpoint()
        msg.position = [float(pos_cmd[0]), float(pos_cmd[1]), float(pos_cmd[2])]
        msg.velocity = [float(vel_cmd[0]), float(vel_cmd[1]), float(vel_cmd[2])]
        horizontal_speed = np.linalg.norm(vel_cmd[0:2])
        if horizontal_speed > 0.1:
            msg.yaw = float(math.atan2(vel_cmd[1], vel_cmd[0]))
        else:
            msg.yaw = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def _publish_hold_setpoint(self):
        """发布当前位置悬停设定点（无速度指令）"""
        self._publish_trajectory_setpoint(
            self.tracker_state.position,
            np.array([0.0, 0.0, 0.0])
        )

    # ================================================================
    # 评估话题
    # ================================================================

    def _publish_evaluation_event(self, event: str, **fields):
        """发布评估事件。

        事件内容使用 JSON 字符串，便于 rosbag 记录后由脚本离线解析；case_id
        用于把多轮评估的数据隔离开。
        """

        if not self.evaluation_enable_topics or self.evaluation_event_pub is None:
            return

        payload = {
            "event": event,
            "case_id": self.evaluation_case_id,
            "stamp": self.get_clock().now().nanoseconds / 1e9,
        }
        payload.update(fields)
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self.evaluation_event_pub.publish(msg)

    def _publish_target_state(self, target_state: State):
        """发布虚拟目标真值。

        目标不是 Gazebo 模型，因此评估器不能从 Gazebo 读取目标状态；这里将
        ROS2 内部 SimulatedTarget 的位置和速度转换为 Odometry 作为唯一真值源。
        """

        if not self.evaluation_enable_topics or self.target_state_pub is None:
            return

        msg = Odometry()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = "map"
        msg.child_frame_id = "target"
        msg.pose.pose.position.x = float(target_state.position[0])
        msg.pose.pose.position.y = float(target_state.position[1])
        msg.pose.pose.position.z = float(target_state.position[2])
        msg.twist.twist.linear.x = float(target_state.velocity[0])
        msg.twist.twist.linear.y = float(target_state.velocity[1])
        msg.twist.twist.linear.z = float(target_state.velocity[2])
        self.target_state_pub.publish(msg)

    # ================================================================
    # 安全机制
    # ================================================================

    def check_battery_safety(self):
        with self._lock:
            connected = self._battery_connected
            remaining = self._battery_remaining
            warning = self._battery_warning

        is_safe = True
        if not connected:
            is_safe = False
            self.get_logger().warn('battery not connected (未连接电池)')
        elif remaining < 0.20:
            is_safe = False
        elif warning >= BatteryStatus.WARNING_CRITICAL:
            is_safe = False

        with self._lock:
            old_safe = self._safe_to_fly
            self._safe_to_fly = is_safe
            last_warn = self._last_battery_warn_time

        if is_safe != old_safe:
            if is_safe:
                self.get_logger().info(f'battery safe (电池安全): {remaining * 100.0:.1f}% remaining')
            else:
                self.get_logger().error(f'battery unsafe (电池不安全): {remaining * 100.0:.1f}% remaining')

        now = self.get_clock().now()
        if connected and remaining < 0.30 and (now - last_warn).nanoseconds > 30_000_000_000:
            self.get_logger().warn(f'battery low (电量低): {remaining * 100.0:.1f}%')
            with self._lock:
                self._last_battery_warn_time = now

    def monitor_safety(self):
        with self._lock:
            nav_state = int(self.vehicle_status.nav_state)
            arming_state = int(self.vehicle_status.arming_state)

        if nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().warn(f'unexpected mode exit (意外退出导航模式), nav_state={nav_state}')

        if arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().warn('vehicle unexpectedly disarmed (飞行器意外上锁)')
            self._update_state(FlightState.ERROR)

        now = self.get_clock().now()
        with self._lock:
            last_check = self._last_status_check

        if (now - last_check).nanoseconds > 1_000_000_000:
            self.get_logger().warn('vehicle status update timeout (状态更新超时)')

    def handle_error_state(self):
        with self._lock:
            nav_state = int(self.vehicle_status.nav_state)
            arming_state = int(self.vehicle_status.arming_state)
            attempts = self._recovery_attempts

        max_recovery_attempts = 3
        self.get_logger().error(
            f'handling error state (处理错误状态), nav_state={nav_state}, arming_state={arming_state}'
        )

        if arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().warn('attempting disarm in error state (尝试在错误状态下锁)')
            self._disarm()

        if nav_state != VehicleStatus.NAVIGATION_STATE_MANUAL:
            if attempts < max_recovery_attempts:
                self.get_logger().info(
                    f'trying to switch to manual mode (尝试切至手动模式) '
                    f'({attempts + 1}/{max_recovery_attempts})'
                )
                self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=1.0)
                with self._lock:
                    self._recovery_attempts += 1
            else:
                self.get_logger().fatal('recovery failed, timer canceled (恢复失败，停止主循环)')
                if self.timer is not None:
                    self.timer.cancel()
        else:
            self.get_logger().info('manual mode reached, resetting state machine (已回到手动模式)')
            with self._lock:
                self._recovery_attempts = 0
            self._update_state(FlightState.INIT)

    def return_flight(self):
        with self._lock:
            nav_state = int(self.vehicle_status.nav_state)

        if not self._rtl_command_sent:
            self.get_logger().info('error path entered, requesting RTL (请求返航)')
            self._request_rtl()
            self._rtl_command_sent = True
            self._rtl_start_time = self.get_clock().now()

        if nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
            self._publish_offboard_heartbeat()
            self._publish_hold_setpoint()

        if nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL and not self._rtl_logged:
            self.get_logger().warn('RTL active, offboard loop finished (触发返航)')
            self._rtl_logged = True

    # ================================================================
    # OFFBOARD_ACTIVE 导引子阶段
    # ================================================================

    def _handle_takeoff(self):
        self._publish_trajectory_setpoint(
            np.array([0.0, 0.0, self.takeoff_height]),
            np.array([0.0, 0.0, 0.0])
        )

        with self._lock:
            current_z = self.vehicle_local_position.z
            nav_state = int(self.vehicle_status.nav_state)

        is_at_height = current_z <= self.takeoff_height + 0.5
        is_offboard = nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        if self.offboard_setpoint_counter % self.log_interval == 0:
            self.get_logger().info(
                f"[起飞] 当前高度={-current_z:.1f}m, "
                f"目标高度={-self.takeoff_height:.1f}m, "
                f"离板模式={'是' if is_offboard else '否'}"
            )

        if is_at_height and is_offboard:
            self.get_logger().info(f"起飞完成！当前高度 {-current_z:.1f}m，开始导引追踪")
            self._guidance_subphase = "GUIDANCE"
            self.get_logger().info("导引子阶段切换: TAKEOFF -> GUIDANCE")
            # 评估指标从 GUIDANCE 阶段开始，起飞耗时单独统计。
            self._publish_evaluation_event("guidance_started")

    def _handle_guidance(self):
        self.target_provider.update(self.dt)

        if not self.target_provider.is_valid():
            self.get_logger().warn("目标数据无效，保持当前位置悬停")
            self._publish_hold_setpoint()
            return

        target_state = self.target_provider.get_state()
        # 每个导引周期同步发布目标真值，评估器用它计算真实距离。
        self._publish_target_state(target_state)
        dist = float(np.linalg.norm(target_state.position - self.tracker_state.position))

        if dist < self.intercept_radius:
            self.get_logger().info("=" * 40)
            self.get_logger().info(f"拦截成功！距离={dist:.2f}m，执行降落")
            self.get_logger().info("=" * 40)
            # 成功、降落和结束分成三个事件，方便离线分析阶段边界。
            self._publish_evaluation_event("intercept_success", distance=dist)
            self._guidance_subphase = "LAND"
            self.get_logger().info("导引子阶段切换: GUIDANCE -> LAND")
            self._publish_evaluation_event("land_started", distance=dist)
            self._land()
            self._publish_evaluation_event("case_finished", success=True, distance=dist)
            return

        result: GuidanceResult = self.guidance_core.calculate_guidance(
            self.tracker_state, target_state, self.dt
        )

        self._publish_trajectory_setpoint(result.pos_cmd, result.vel_cmd)

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

    # ================================================================
    # 主控制回路（状态机）
    # ================================================================

    def _control_loop(self):
        # 始终发送心跳
        self._publish_offboard_heartbeat()
        if self.evaluation_enable_topics and self.target_provider.is_valid():
            # TAKEOFF 阶段也发布目标真值，让 rosbag 中有完整目标轨迹。
            self._publish_target_state(self.target_provider.get_state())

        # 电池安全检查（仿真模式下跳过）
        if not self._simulation_mode:
            self.check_battery_safety()

        with self._lock:
            state = self._current_state
            nav_state = int(self.vehicle_status.nav_state)
            arming_state = int(self.vehicle_status.arming_state)
            offboard_enabled = self._control_offboard_enabled

        # ---- INIT: 发送设定点并计数 ----
        if state == FlightState.INIT:
            self._publish_trajectory_setpoint(
                np.array([0.0, 0.0, 0.0]),
                np.array([0.0, 0.0, 0.0])
            )
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter >= 10:
                self.get_logger().info(
                    f'sent {self.offboard_setpoint_counter} setpoints, '
                    f'requesting offboard mode (设定点发送完毕，请求切入Offboard)'
                )
                self._engage_offboard_mode()
                self._arm()
                self._update_state(FlightState.REQUESTING_OFFBOARD)
            return

        # ---- REQUESTING_OFFBOARD: 等待 VehicleControlMode 确认 ----
        if state == FlightState.REQUESTING_OFFBOARD:
            self._publish_hold_setpoint()

            if self._elapsed_since_state_start() > self._offboard_timeout:
                self.get_logger().error(
                    f'offboard request timeout, nav_state={nav_state} (请求Offboard超时)'
                )
                self._update_state(FlightState.ERROR)
                return

            if nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and offboard_enabled:
                self.get_logger().info('offboard mode active (Offboard模式激活)')
                self._update_state(FlightState.ARMING)
            return

        # ---- ARMING: 等待解锁状态确认 ----
        if state == FlightState.ARMING:
            self._publish_hold_setpoint()

            if self._elapsed_since_state_start() > self._arm_timeout:
                self.get_logger().error(
                    f'arm confirmation timeout, arming_state={arming_state} (解锁等待确认超时)'
                )
                self._update_state(FlightState.ERROR)
                return

            if arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info('vehicle armed (无人机已解锁)')
                self._update_state(FlightState.OFFBOARD_ACTIVE)
                self._guidance_subphase = "TAKEOFF"
                self.get_logger().info("导引子阶段切换: -> TAKEOFF")
            return

        # ---- OFFBOARD_ACTIVE: 内嵌导引子阶段 ----
        if state == FlightState.OFFBOARD_ACTIVE:
            # 安全监控（仿真模式下跳过）
            if not self._simulation_mode:
                self.monitor_safety()

                # 低电量安全联动：若电池不安全，触发返航
                with self._lock:
                    safe = self._safe_to_fly
                if not safe:
                    self.get_logger().warn("电池不安全，触发返航")
                    self._update_state(FlightState.RETURN_REQUESTED)
                    return

            if self._guidance_subphase == "TAKEOFF":
                self._handle_takeoff()
            elif self._guidance_subphase == "GUIDANCE":
                self._handle_guidance()
            elif self._guidance_subphase == "LAND":
                pass  # 降落由 PX4 自主控制

            self.offboard_setpoint_counter += 1
            return

        # ---- RETURN_REQUESTED: 返航 ----
        if state == FlightState.RETURN_REQUESTED:
            self.return_flight()
            return

        # ---- ERROR: 错误恢复 ----
        if state == FlightState.ERROR:
            self.handle_error_state()
            return


def main(args=None):
    rclpy.init(args=args)
    node = AntiDroneGuidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到键盘中断信号 (SIGINT)，正在关闭...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
