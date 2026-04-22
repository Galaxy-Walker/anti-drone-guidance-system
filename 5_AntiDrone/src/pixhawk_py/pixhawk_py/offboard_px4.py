from enum import Enum, auto
import threading

from px4_msgs.msg import BatteryStatus
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleStatus
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


class FlightState(Enum):
    INIT = auto()
    SENDING_CONTROL = auto()
    REQUESTING_OFFBOARD = auto()
    ARMING = auto()
    WAITING_ARM_CONFIRM = auto()
    OFFBOARD_ACTIVE = auto()
    RETURN_REQUESTED = auto()
    ERROR = auto()


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        self._lock = threading.Lock()

        self._offboard_control_mode_publisher = None
        self._trajectory_setpoint_publisher = None
        self._vehicle_command_publisher = None

        self._vehicle_status_subscriber = None
        self._vehicle_control_mode_subscriber = None
        self._battery_status_subscriber = None
        self._timer = None

        self._offboard_setpoint_counter = 0
        self._offboard_requested = False
        self._current_state = FlightState.INIT

        now = self.get_clock().now()
        self._offboard_request_time = now
        self._state_start_time = now
        self._offboard_timeout = Duration(seconds=2.0)
        self._arm_timeout = Duration(seconds=2.0)

        self._current_nav_state = 0
        self._current_arming_state = 0
        self._last_status_timestamp = 0

        self._control_offboard_enabled = False
        self._is_fully_offboard = False
        self._is_offboard_transitioning = False

        self._remaining = 1.0
        self._connected = False
        self._warning = 0
        self._safe_to_fly = False
        self._last_safe_state = False
        self._last_battery_warn_time = now

        self._rtl_command_sent = False
        self._rtl_start_time = now
        self._rtl_command_duration = Duration(seconds=1.0)
        self._rtl_logged = False

        self._last_status_check = now
        self._recovery_attempts = 0

        self.get_logger().info('offboard node initialized')
        self.initializePublishers()
        self.initializeSubscribers()
        self._timer = self.create_timer(0.1, self.process)

    def initializePublishers(self):
        self._offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10,
        )
        self.get_logger().info('publisher ready: /fmu/in/offboard_control_mode')

        self._trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10,
        )
        self.get_logger().info('publisher ready: /fmu/in/trajectory_setpoint')

        self._vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10,
        )
        self.get_logger().info('publisher ready: /fmu/in/vehicle_command')

    def initializeSubscribers(self):
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos,
        )
        self.get_logger().info('subscriber ready: /fmu/out/vehicle_status_v1')

        self._vehicle_control_mode_subscriber = self.create_subscription(
            VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.vehicle_control_mode_callback,
            qos,
        )
        self.get_logger().info('subscriber ready: /fmu/out/vehicle_control_mode')

        self._battery_status_subscriber = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.battery_status_callback,
            qos,
        )
        self.get_logger().info('subscriber ready: /fmu/out/battery_status')

    def vehicle_status_callback(self, msg: VehicleStatus):
        with self._lock:
            self._current_nav_state = int(msg.nav_state)
            self._current_arming_state = int(msg.arming_state)
            self._last_status_timestamp = int(msg.timestamp)

    def vehicle_control_mode_callback(self, msg: VehicleControlMode):
        transition = None
        with self._lock:
            was_offboard = self._control_offboard_enabled
            now_offboard = bool(msg.flag_control_offboard_enabled)
            self._control_offboard_enabled = now_offboard

            if (not was_offboard) and now_offboard:
                self._is_offboard_transitioning = True
                transition = 'entered'
            elif was_offboard and (not now_offboard):
                self._is_fully_offboard = False
                transition = 'exited'

        if transition == 'entered':
            self.get_logger().info('controller entered offboard mode')
        elif transition == 'exited':
            self.get_logger().warn('controller exited offboard mode')

    def battery_status_callback(self, msg: BatteryStatus):
        with self._lock:
            self._connected = bool(msg.connected)
            self._remaining = float(msg.remaining)
            self._warning = int(msg.warning)

    def _elapsed_since_state_start(self) -> Duration:
        with self._lock:
            state_start = self._state_start_time
        return self.get_clock().now() - state_start

    def process(self):
        with self._lock:
            nav_state = self._current_nav_state
            arming_state = self._current_arming_state
            was_offboard = self._control_offboard_enabled
            state = self._current_state

        if state == FlightState.INIT:
            self.get_logger().info('state: INIT')
            with self._lock:
                self._offboard_setpoint_counter = 0
            self.update_state(FlightState.SENDING_CONTROL)
            return

        if state == FlightState.SENDING_CONTROL:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0, -3.14)

            with self._lock:
                self._offboard_setpoint_counter += 1
                counter = self._offboard_setpoint_counter

            if counter >= 10:
                self.get_logger().info(
                    f'sent {counter} setpoints, requesting offboard mode'
                )
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,
                    6.0,
                )
                self.update_state(FlightState.REQUESTING_OFFBOARD)
            return

        if state == FlightState.REQUESTING_OFFBOARD:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0, -3.14)

            if self._elapsed_since_state_start() > self._offboard_timeout:
                self.get_logger().error(
                    f'offboard request timeout, nav_state={nav_state}'
                )
                self.update_state(FlightState.ERROR)
                return

            if (
                nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and was_offboard
            ):
                self.get_logger().info('offboard mode active')
                self.update_state(FlightState.ARMING)
            return

        if state == FlightState.ARMING:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0, -3.14)

            if self.check_pre_arm_conditions():
                self.arm()
                self.update_state(FlightState.WAITING_ARM_CONFIRM)
            else:
                self.get_logger().warn('pre-arm conditions not met yet')
            return

        if state == FlightState.WAITING_ARM_CONFIRM:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0, -3.14)

            if self._elapsed_since_state_start() > self._arm_timeout:
                self.get_logger().error(
                    f'arm confirmation timeout, arming_state={arming_state}'
                )
                self.update_state(FlightState.ERROR)
                return

            if arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info('vehicle armed')
                self.update_state(FlightState.OFFBOARD_ACTIVE)
            return

        if state == FlightState.OFFBOARD_ACTIVE:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0, -3.14)
            return

        if state == FlightState.RETURN_REQUESTED:
            self.return_flight(nav_state)
            return

        if state == FlightState.ERROR:
            # Keep parity with current C++ behavior: recovery handler exists but is
            # currently not enabled in process().
            return

    def update_state(self, new_state: FlightState):
        with self._lock:
            old_state = self._current_state
            if old_state == new_state:
                return
            self._current_state = new_state
            self._state_start_time = self.get_clock().now()

        self.get_logger().info(f'state transition: {old_state.name} -> {new_state.name}')

    def check_pre_arm_conditions(self) -> bool:
        with self._lock:
            nav_state = self._current_nav_state
        return nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def check_offboard_mode(self) -> bool:
        with self._lock:
            nav_state = self._current_nav_state
            offboard_enabled = self._control_offboard_enabled
        return (
            nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and offboard_enabled
        )

    def check_armed_state(self) -> bool:
        with self._lock:
            arming_state = self._current_arming_state
        return arming_state == VehicleStatus.ARMING_STATE_ARMED

    def check_battery_safety(self):
        with self._lock:
            connected = self._connected
            remaining = self._remaining
            warning = self._warning

        is_safe = True
        if not connected:
            is_safe = False
            self.get_logger().warn('battery not connected')
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
                self.get_logger().info(
                    f'battery safe: {remaining * 100.0:.1f}% remaining'
                )
            else:
                self.get_logger().error(
                    f'battery unsafe: {remaining * 100.0:.1f}% remaining'
                )

        now = self.get_clock().now()
        if connected and remaining < 0.30 and (now - last_warn).nanoseconds > 30_000_000_000:
            self.get_logger().warn(f'battery low: {remaining * 100.0:.1f}%')
            with self._lock:
                self._last_battery_warn_time = now

    def monitor_safety(self, nav_state: int, arming_state: int):
        if nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().warn(
                f'unexpected mode exit, nav_state={nav_state}'
            )

        if arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().warn('vehicle unexpectedly disarmed')
            self.update_state(FlightState.ERROR)

        now = self.get_clock().now()
        with self._lock:
            last_check = self._last_status_check
            self._last_status_check = now

        if (now - last_check).nanoseconds > 1_000_000_000:
            self.get_logger().warn('vehicle status update timeout')

    def handle_error_state(self, nav_state: int, arming_state: int):
        max_recovery_attempts = 3
        self.get_logger().error(
            f'handling error state, nav_state={nav_state}, arming_state={arming_state}'
        )

        if arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().warn('attempting disarm in error state')
            self.disarm()

        if nav_state != VehicleStatus.NAVIGATION_STATE_MANUAL:
            with self._lock:
                attempts = self._recovery_attempts

            if attempts < max_recovery_attempts:
                self.get_logger().info(
                    'trying to switch to manual mode '
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
                self.get_logger().fatal('recovery failed, timer canceled')
                if self._timer is not None:
                    self._timer.cancel()
        else:
            self.get_logger().info('manual mode reached, resetting state machine')
            with self._lock:
                self._recovery_attempts = 0
            self.update_state(FlightState.INIT)

    def return_flight(self, nav_current_state: int):
        if not self._rtl_command_sent:
            self.get_logger().info('error path entered, requesting RTL')
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
            self.get_logger().warn('RTL active, offboard loop finished')
            self._rtl_logged = True

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0,
            0.0,
        )
        self.get_logger().info('arm command sent')

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0,
            0.0,
        )
        self.get_logger().info('disarm command sent')

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x: float, y: float, z: float, yaw: float):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
    ):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._vehicle_command_publisher.publish(msg)
