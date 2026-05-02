from enum import Enum, auto
import threading

# 导入PX4相关的ROS 2消息类型，用于与飞控通信
from px4_msgs.msg import BatteryStatus          # 电池状态
from px4_msgs.msg import OffboardControlMode    # Offboard控制模式标志
from px4_msgs.msg import TrajectorySetpoint     # 期望轨迹（位置、速度等）
from px4_msgs.msg import VehicleCommand         # 发送给飞控的指令（如切换模式、解锁）
from px4_msgs.msg import VehicleControlMode     # 飞控当前所处的控制模式
from px4_msgs.msg import VehicleStatus          # 飞控当前的状态（如导航状态、解锁状态）

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class FlightState(Enum):
    """
    定义无人机的飞行状态机枚举类。
    用于在主循环中管理无人机从初始化到接管控制、解锁、再到执行任务的完整流程。
    """
    INIT = auto()                  # 初始化状态
    SENDING_CONTROL = auto()       # 发送初始控制信号（进入Offboard模式前必须先发送一段时间的控制信号）
    REQUESTING_OFFBOARD = auto()   # 请求进入Offboard（外部计算机控制）模式
    ARMING = auto()                # 请求解锁电机
    WAITING_ARM_CONFIRM = auto()   # 等待解锁成功确认
    OFFBOARD_ACTIVE = auto()       # Offboard模式已激活且已解锁，正在执行正常任务
    RETURN_REQUESTED = auto()      # 请求返航 (RTL)
    ERROR = auto()                 # 错误状态，用于异常处理


class OffboardControl(Node):
    def __init__(self):
        # 初始化ROS 2节点，节点名为 'offboard_control'
        super().__init__('offboard_control')

        # 线程锁，防止回调函数（接收数据）和主循环（处理数据）之间发生数据竞争
        self._lock = threading.Lock()

        # 定义发布者变量，初始化为 None
        self._offboard_control_mode_publisher = None
        self._trajectory_setpoint_publisher = None
        self._vehicle_command_publisher = None

        # 定义订阅者变量，初始化为 None
        self._vehicle_status_subscriber = None
        self._vehicle_control_mode_subscriber = None
        self._battery_status_subscriber = None
        self._timer = None

        # 计数器和标志位
        self._offboard_setpoint_counter = 0  # 记录发送了多少个设定点（Setpoint）
        self._offboard_requested = False
        self._current_state = FlightState.INIT

        now = self.get_clock().now()
        self._offboard_request_time = now
        self._state_start_time = now

        # 超时设置
        self._offboard_timeout = Duration(seconds=2.0) # 请求Offboard的超时时间
        self._arm_timeout = Duration(seconds=2.0)      # 请求解锁的超时时间

        # 传感器与状态数据存储
        self._current_nav_state = 0
        self._current_arming_state = 0
        self._last_status_timestamp = 0

        self._control_offboard_enabled = False
        self._is_fully_offboard = False
        self._is_offboard_transitioning = False

        # 电池相关状态
        self._remaining = 1.0         # 电池剩余电量 (0.0 - 1.0)
        self._connected = False       # 电池是否连接
        self._warning = 0             # 电池警告级别
        self._safe_to_fly = False     # 当前电量是否安全
        self._last_safe_state = False
        self._last_battery_warn_time = now

        # 返航(RTL)相关状态
        self._rtl_command_sent = False
        self._rtl_start_time = now
        self._rtl_command_duration = Duration(seconds=1.0)
        self._rtl_logged = False

        # 安全监控和重试限制
        self._last_status_check = now
        self._recovery_attempts = 0

        self.get_logger().info('offboard node initialized (Offboard节点已初始化)')

        # 创建ROS 2发布者和订阅者
        self.initializePublishers()
        self.initializeSubscribers()

        # 创建定时器，每0.1秒（10Hz）执行一次 process 函数，驱动状态机
        self._timer = self.create_timer(0.1, self.process)

    def initializePublishers(self):
        """
        初始化所有发布者。
        这些发布者用于向PX4飞控发送控制指令和期望轨迹。
        """
        # 发布 Offboard 控制模式标志（告诉飞控我们要控制位置、速度还是姿态）
        self._offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10,
        )
        self.get_logger().info('publisher ready: /fmu/in/offboard_control_mode')

        # 发布期望的位置、速度等轨迹信息
        self._trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10,
        )
        self.get_logger().info('publisher ready: /fmu/in/trajectory_setpoint')

        # 发布载具系统命令（如切换飞行模式，解锁/上锁指令）
        self._vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10,
        )
        self.get_logger().info('publisher ready: /fmu/in/vehicle_command')

    def initializeSubscribers(self):
        """
        初始化所有订阅者。
        注意：PX4 ROS 2 桥接通常要求 QoS 设置为 BEST_EFFORT 和 TRANSIENT_LOCAL。
        """
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        # 订阅飞行器的总体状态（判断是否解锁，当前的导航模式等）
        self._vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos,
        )
        self.get_logger().info('subscriber ready: /fmu/out/vehicle_status_v1')

        # 订阅控制模式（了解飞控目前真正使能了哪些控制通道，如是否已进入Offboard）
        self._vehicle_control_mode_subscriber = self.create_subscription(
            VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.vehicle_control_mode_callback,
            qos,
        )
        self.get_logger().info('subscriber ready: /fmu/out/vehicle_control_mode')

        # 订阅电池状态，用于电量监控
        self._battery_status_subscriber = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.battery_status_callback,
            qos,
        )
        self.get_logger().info('subscriber ready: /fmu/out/battery_status')

    def vehicle_status_callback(self, msg: VehicleStatus):
        """ 获取当前的导航状态(nav_state)和解锁状态(arming_state) """
        with self._lock:
            self._current_nav_state = int(msg.nav_state)
            self._current_arming_state = int(msg.arming_state)
            self._last_status_timestamp = int(msg.timestamp)

    def vehicle_control_mode_callback(self, msg: VehicleControlMode):
        """ 监控是否成功进入了 Offboard 模式 """
        transition = None
        with self._lock:
            was_offboard = self._control_offboard_enabled
            now_offboard = bool(msg.flag_control_offboard_enabled)
            self._control_offboard_enabled = now_offboard

            # 状态转变边缘检测
            if (not was_offboard) and now_offboard:
                self._is_offboard_transitioning = True
                transition = 'entered'
            elif was_offboard and (not now_offboard):
                self._is_fully_offboard = False
                transition = 'exited'

        if transition == 'entered':
            self.get_logger().info('controller entered offboard mode (已进入Offboard模式)')
        elif transition == 'exited':
            self.get_logger().warn('controller exited offboard mode (已退出Offboard模式)')

    def battery_status_callback(self, msg: BatteryStatus):
        """ 获取电池剩余电量和警告 """
        with self._lock:
            self._connected = bool(msg.connected)
            self._remaining = float(msg.remaining)
            self._warning = int(msg.warning)

    def _elapsed_since_state_start(self) -> Duration:
        """ 计算当前状态持续的时间 """
        with self._lock:
            state_start = self._state_start_time
        now = self.get_clock().now()
        return Duration(nanoseconds=(now.nanoseconds - state_start.nanoseconds))

    def process(self):
        """
        主控制循环，频率 10Hz（由 timer 设定）。
        这是一个状态机，驱动无人机完成起飞和任务。
        """
        with self._lock:
            nav_state = self._current_nav_state
            arming_state = self._current_arming_state
            was_offboard = self._control_offboard_enabled
            state = self._current_state

        # 1. 初始状态
        if state == FlightState.INIT:
            self.get_logger().info('state: INIT (初始化)')
            with self._lock:
                self._offboard_setpoint_counter = 0
            # 切换到发送控制信息状态
            self.update_state(FlightState.SENDING_CONTROL)
            return

        # 2. 发送控制信号状态 (PX4要求在进入Offboard模式前，必须至少收到一段时间的期望设定点)
        if state == FlightState.SENDING_CONTROL:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0, -3.14)

            with self._lock:
                self._offboard_setpoint_counter += 1
                counter = self._offboard_setpoint_counter

            # 连续发送10个点（1秒钟左右）后，发送切换模式指令
            if counter >= 10:
                self.get_logger().info(
                    f'sent {counter} setpoints, requesting offboard mode (设定点发送完毕，请求切入Offboard)'
                )
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,
                    6.0, # 6.0 对应于 Offboard 模式的 custom mode 序号
                )
                self.update_state(FlightState.REQUESTING_OFFBOARD)
            return

        # 3. 请求Offboard模式状态 (等待飞控确认已进入Offboard)
        if state == FlightState.REQUESTING_OFFBOARD:
            # 持续发送信号以保持Offboard心跳
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0, -3.14)

            # 超时检测
            if self._elapsed_since_state_start() > self._offboard_timeout:
                self.get_logger().error(
                    f'offboard request timeout, nav_state={nav_state} (请求Offboard超时)'
                )
                self.update_state(FlightState.ERROR)
                return

            # 如果确认已处于 Offboard，转入解锁阶段
            if (
                nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and was_offboard
            ):
                self.get_logger().info('offboard mode active (Offboard模式激活)')
                self.update_state(FlightState.ARMING)
            return

        # 4. 请求解锁状态
        if state == FlightState.ARMING:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0, -3.14)

            # 满足解锁条件（必须处于Offboard）后发送解锁指令
            if self.check_pre_arm_conditions():
                self.arm()
                self.update_state(FlightState.WAITING_ARM_CONFIRM)
            else:
                self.get_logger().warn('pre-arm conditions not met yet (尚未满足解锁条件)')
            return

        # 5. 等待飞控解锁确认
        if state == FlightState.WAITING_ARM_CONFIRM:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0, -3.14)

            # 超时检测
            if self._elapsed_since_state_start() > self._arm_timeout:
                self.get_logger().error(
                    f'arm confirmation timeout, arming_state={arming_state} (解锁等待确认超时)'
                )
                self.update_state(FlightState.ERROR)
                return

            # 若成功解锁
            if arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info('vehicle armed (无人机已解锁)')
                self.update_state(FlightState.OFFBOARD_ACTIVE)
            return

        # 6. 正常巡航/任务阶段
        if state == FlightState.OFFBOARD_ACTIVE:
            # ====== 在这里编写您的核心逻辑、轨迹生成算法 ======
            # 下方只是让无人机保持悬停于 (x=0, y=0, z=-5.0) 的高度，朝向 yaw=-3.14
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0, -3.14)
            return

        # 7. 返航状态
        if state == FlightState.RETURN_REQUESTED:
            self.return_flight(nav_state)
            return

        # 8. 错误状态处理
        if state == FlightState.ERROR:
            # 触发保护流程
            return

    def update_state(self, new_state: FlightState):
        """ 切换状态机的状态，并重置时间记录器 """
        with self._lock:
            old_state = self._current_state
            if old_state == new_state:
                return
            self._current_state = new_state
            self._state_start_time = self.get_clock().now()

        self.get_logger().info(f'state transition (状态切换): {old_state.name} -> {new_state.name}')

    def check_pre_arm_conditions(self) -> bool:
        """ 检查是否满足了解锁先决条件 (在此示例中，要求处于Offboard下) """
        with self._lock:
            nav_state = self._current_nav_state
        return nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def check_offboard_mode(self) -> bool:
        """ 返回当前是否正在处于Offboard控制中 """
        with self._lock:
            nav_state = self._current_nav_state
            offboard_enabled = self._control_offboard_enabled
        return (
            nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and offboard_enabled
        )

    def check_armed_state(self) -> bool:
        """ 返回当前是否已经解锁 """
        with self._lock:
            arming_state = self._current_arming_state
        return arming_state == VehicleStatus.ARMING_STATE_ARMED

    def check_battery_safety(self):
        """ 电池安全性检查 """
        with self._lock:
            connected = self._connected
            remaining = self._remaining
            warning = self._warning

        is_safe = True
        if not connected:
            is_safe = False
            self.get_logger().warn('battery not connected (未连接电池)')
        elif remaining < 0.20: # 电量不足 20% 警告
            is_safe = False
        elif warning >= BatteryStatus.WARNING_CRITICAL:
            is_safe = False

        with self._lock:
            old_safe = self._safe_to_fly
            self._safe_to_fly = is_safe
            last_warn = self._last_battery_warn_time

        if is_safe != old_safe:
            if is_safe:
                self.get_logger().info(
                    f'battery safe (电池安全): {remaining * 100.0:.1f}% remaining'
                )
            else:
                self.get_logger().error(
                    f'battery unsafe (电池不安全): {remaining * 100.0:.1f}% remaining'
                )

        now = self.get_clock().now()
        # 控制警告间隔为 30秒
        if connected and remaining < 0.30 and (now - last_warn).nanoseconds > 30_000_000_000:
            self.get_logger().warn(f'battery low (电量低): {remaining * 100.0:.1f}%')
            with self._lock:
                self._last_battery_warn_time = now

    def monitor_safety(self, nav_state: int, arming_state: int):
        """ 监控安全飞行状态，检测异常退出和异常上锁 """
        if nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().warn(
                f'unexpected mode exit (意外退出导航模式), nav_state={nav_state}'
            )

        if arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().warn('vehicle unexpectedly disarmed (飞行器意外上锁)')
            self.update_state(FlightState.ERROR)

        now = self.get_clock().now()
        with self._lock:
            last_check = self._last_status_check
            self._last_status_check = now

        # 若数据流断开超时 1 秒
        if (now - last_check).nanoseconds > 1_000_000_000:
            self.get_logger().warn('vehicle status update timeout (状态更新超时)')

    def handle_error_state(self, nav_state: int, arming_state: int):
        """ 处理错误状态，尝试安全降落及切换至手动 """
        max_recovery_attempts = 3
        self.get_logger().error(
            f'handling error state (处理错误状态), nav_state={nav_state}, arming_state={arming_state}'
        )

        if arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().warn('attempting disarm in error state (尝试在错误状态下锁)')
            self.disarm()

        if nav_state != VehicleStatus.NAVIGATION_STATE_MANUAL:
            with self._lock:
                attempts = self._recovery_attempts

            if attempts < max_recovery_attempts:
                self.get_logger().info(
                    'trying to switch to manual mode (尝试且至手动模式) '
                    f'({attempts + 1}/{max_recovery_attempts})'
                )
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,
                    1.0,
                )
                with self._lock:
                    self._recovery_attempts += 1
            else:
                self.get_logger().fatal('recovery failed, timer canceled (恢复失败，停止主循环)')
                if self._timer is not None:
                    self._timer.cancel()
        else:
            self.get_logger().info('manual mode reached, resetting state machine (已回到手动模式)')
            with self._lock:
                self._recovery_attempts = 0
            self.update_state(FlightState.INIT)

    def return_flight(self, nav_current_state: int):
        """ 返航请求 """
        if not self._rtl_command_sent:
            self.get_logger().info('error path entered, requesting RTL (请求返航)')
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,
                0.0,
                0.0,
            )
            self._rtl_command_sent = True
            self._rtl_start_time = self.get_clock().now()

        if nav_current_state != VehicleStatus.NAVIGATION_STATE_AUTO_RTL:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0, -3.14)

        if (
            nav_current_state == VehicleStatus.NAVIGATION_STATE_AUTO_RTL
            and not self._rtl_logged
        ):
            self.get_logger().warn('RTL active, offboard loop finished (触发返航)')
            self._rtl_logged = True

    def arm(self):
        """ 发送解锁指令 """
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0, # 1.0 表示解锁 (Arm)
            0.0,
        )
        self.get_logger().info('arm command sent (解锁指令已发送)')

    def disarm(self):
        """ 发送上锁指令 """
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0, # 0.0 表示上锁 (Disarm)
            0.0,
        )
        self.get_logger().info('disarm command sent (上锁指令已发送)')

    def publish_offboard_control_mode(self):
        """ 告知飞控我们要使用的 Offboard 控制掩码 """
        msg = OffboardControlMode()
        msg.position = True     # 允许位置控制
        msg.velocity = False    # 不采用纯速度控制
        msg.acceleration = False# 不采用加速度控制
        msg.attitude = False    # 不采用姿态控制
        msg.body_rate = False   # 不采用角速率控制

        # 必须带上有效的时间戳，飞控才会信任并且采用
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        publisher = self._offboard_control_mode_publisher
        if publisher is None:
            self.get_logger().error('外部发布器未初始化')
            return
        publisher.publish(msg)

    def publish_trajectory_setpoint(self, x: float, y: float, z: float, yaw: float):
        """ 告知飞控具体飞向哪个目标点 (NED 坐标系，即 北-东-下) """
        msg = TrajectorySetpoint()
        msg.position = [x, y, z] # 注意 Z 轴是向下的，因此 -5.0 代表起飞到 5米高度
        msg.yaw = float(yaw)     # 偏航角 (弧度)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        publisher = self._trajectory_setpoint_publisher
        if publisher is None:
            self.get_logger().error('轨迹设定点发布器未初始化')
            return
        publisher.publish(msg)

    def publish_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
    ):
        """ 通用指令发送函数，向飞控发送 MAVLink Command """
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1    # 目标系统 ID (通常PX4是 1)
        msg.target_component = 1 # 目标组件 ID (通常PX4是 1)
        msg.source_system = 1    # 来源系统 ID (地面站/机载电脑)
        msg.source_component = 1
        msg.from_external = True # 标记为来自外部系统的指令
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        publisher = self._vehicle_command_publisher
        if publisher is None:
            self.get_logger().error('载具命令发布器未初始化')
            return
        publisher.publish(msg)
