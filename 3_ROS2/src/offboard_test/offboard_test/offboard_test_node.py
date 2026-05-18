#!/usr/bin/env python3

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def fmt_float(value, digits=2):
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return "n/a"


def constant_name(msg_type, prefix, value):
    for name in dir(msg_type):
        if name.startswith(prefix) and getattr(msg_type, name) == value:
            return name.removeprefix(prefix)
    return str(value)


def constant_value(msg_type, name, fallback):
    return getattr(msg_type, name, fallback)


class OffboardTestNode(Node):
    """Small PX4 offboard/arm test node using PX4 uORB bridge topics."""

    def __init__(self):
        super().__init__('offboard_test_node')

        self.vehicle_local_position_topic = self.declare_parameter(
            'vehicle_local_position_topic', '/fmu/out/vehicle_local_position').value
        self.vehicle_status_topic = self.declare_parameter(
            'vehicle_status_topic', '/fmu/out/vehicle_status').value
        self.offboard_control_mode_topic = self.declare_parameter(
            'offboard_control_mode_topic', '/fmu/in/offboard_control_mode').value
        self.trajectory_setpoint_topic = self.declare_parameter(
            'trajectory_setpoint_topic', '/fmu/in/trajectory_setpoint').value
        self.vehicle_command_topic = self.declare_parameter(
            'vehicle_command_topic', '/fmu/in/vehicle_command').value

        self.control_rate_hz = float(self.declare_parameter('control_rate_hz', 10.0).value)
        self.command_retry_interval_sec = float(
            self.declare_parameter('command_retry_interval_sec', 1.0).value)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, self.offboard_control_mode_topic, qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, self.trajectory_setpoint_topic, qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, self.vehicle_command_topic, qos_profile)

        self.create_subscription(
            VehicleLocalPosition,
            self.vehicle_local_position_topic,
            self.vehicle_local_position_callback,
            qos_profile,
        )
        self.create_subscription(
            VehicleStatus,
            self.vehicle_status_topic,
            self.vehicle_status_callback,
            qos_profile,
        )

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.local_position_received = False
        self.vehicle_status_received = False
        self.hold_position = None
        self.timer_ticks = 0
        self.last_local_position_time = None
        self.last_vehicle_status_time = None
        self.last_offboard_request_time = None
        self.last_arm_request_time = None
        self.offboard_active_since = None
        self.armed_since = None
        self.last_nav_state = None
        self.last_arming_state = None
        self.last_failsafe = None
        self.last_preflight_checks_pass = None

        timer_period = 1.0 / max(self.control_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.offboard_nav_state = constant_value(
            VehicleStatus, 'NAVIGATION_STATE_OFFBOARD', 14)
        self.armed_state = constant_value(VehicleStatus, 'ARMING_STATE_ARMED', 2)

        self.log_startup_summary(timer_period)

    def log_startup_summary(self, timer_period):
        self.get_logger().info('=' * 70)
        self.get_logger().info('PX4 offboard test node started')
        self.get_logger().info('=' * 70)
        self.get_logger().info(
            'Using PX4 topics: OffboardControlMode, TrajectorySetpoint, '
            'VehicleCommand, VehicleLocalPosition, VehicleStatus.'
        )
        self.get_logger().info(
            'Publish topics: '
            f'{self.offboard_control_mode_topic}, {self.trajectory_setpoint_topic}, '
            f'{self.vehicle_command_topic}'
        )
        self.get_logger().info(
            'Subscribe topics: '
            f'{self.vehicle_local_position_topic}, {self.vehicle_status_topic}'
        )
        self.get_logger().info(
            f'Control rate: {1.0 / timer_period:.1f} Hz; '
            f'command_retry_interval_sec={self.command_retry_interval_sec:.1f}'
        )
        self.get_logger().info(
            'Behavior: publish offboard heartbeat/setpoint, request OFFBOARD, '
            'then request ARM. No takeoff, landing, or mission logic is included.'
        )

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        self.last_local_position_time = self.get_clock().now()
        if not self.local_position_received:
            self.local_position_received = True
            self.hold_position = [float(msg.x), float(msg.y), float(msg.z)]
            self.get_logger().info(
                'First vehicle_local_position received: '
                f'position=[{fmt_float(msg.x)}, {fmt_float(msg.y)}, {fmt_float(msg.z)}], '
                f'velocity=[{fmt_float(getattr(msg, "vx", None))}, '
                f'{fmt_float(getattr(msg, "vy", None))}, '
                f'{fmt_float(getattr(msg, "vz", None))}], '
                f'xy_valid={getattr(msg, "xy_valid", "n/a")}, '
                f'z_valid={getattr(msg, "z_valid", "n/a")}'
            )
            self.get_logger().info(
                'Initial hold setpoint locked at current local position: '
                f'[{fmt_float(self.hold_position[0])}, '
                f'{fmt_float(self.hold_position[1])}, '
                f'{fmt_float(self.hold_position[2])}] NED.'
            )

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        self.last_vehicle_status_time = self.get_clock().now()
        if not self.vehicle_status_received:
            self.vehicle_status_received = True
            self.get_logger().info('First vehicle_status received.')
        self.log_status_changes(msg)

    def log_status_changes(self, msg):
        nav_state = getattr(msg, 'nav_state', None)
        arming_state = getattr(msg, 'arming_state', None)
        failsafe = getattr(msg, 'failsafe', None)
        preflight_checks_pass = getattr(msg, 'pre_flight_checks_pass', None)

        if nav_state != self.last_nav_state:
            self.get_logger().info(
                'Vehicle nav_state changed: '
                f'{constant_name(VehicleStatus, "NAVIGATION_STATE_", nav_state)}({nav_state})'
            )
            self.last_nav_state = nav_state

        if arming_state != self.last_arming_state:
            self.get_logger().info(
                'Vehicle arming_state changed: '
                f'{constant_name(VehicleStatus, "ARMING_STATE_", arming_state)}({arming_state})'
            )
            self.last_arming_state = arming_state

        if failsafe != self.last_failsafe:
            log = self.get_logger().warn if failsafe else self.get_logger().info
            log(f'Vehicle failsafe changed: {failsafe}')
            self.last_failsafe = failsafe

        if preflight_checks_pass != self.last_preflight_checks_pass:
            self.get_logger().info(f'Preflight checks pass: {preflight_checks_pass}')
            self.last_preflight_checks_pass = preflight_checks_pass

    def timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def message_age_seconds(self, stamp):
        if stamp is None:
            return 'never'
        age_ns = self.get_clock().now().nanoseconds - stamp.nanoseconds
        return fmt_float(age_ns / 1_000_000_000.0, digits=1)

    def retry_interval_elapsed(self, last_request_time):
        if last_request_time is None:
            return True
        age_ns = self.get_clock().now().nanoseconds - last_request_time.nanoseconds
        return age_ns >= self.command_retry_interval_sec * 1_000_000_000

    def is_offboard(self):
        return getattr(self.vehicle_status, 'nav_state', None) == self.offboard_nav_state

    def is_armed(self):
        return getattr(self.vehicle_status, 'arming_state', None) == self.armed_state

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.timestamp_us()
        self.offboard_control_mode_pub.publish(msg)

    def publish_position_setpoint(self):
        if self.hold_position is None:
            return
        msg = TrajectorySetpoint()
        msg.position = list(self.hold_position)
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = math.nan
        msg.yawspeed = math.nan
        msg.timestamp = self.timestamp_us()
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.timestamp_us()
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info(
            f'Published VehicleCommand: command={command}, '
            f'param1={fmt_float(msg.param1)}, param2={fmt_float(msg.param2)}, '
            f'param3={fmt_float(msg.param3)}, param4={fmt_float(msg.param4)}, '
            f'param5={fmt_float(msg.param5)}, param6={fmt_float(msg.param6)}, '
            f'param7={fmt_float(msg.param7)}'
        )

    def request_offboard(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.get_logger().info(
            'OFFBOARD request sent: VEHICLE_CMD_DO_SET_MODE param1=1.0 param2=6.0'
        )

    def request_arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )
        self.get_logger().info(
            'ARM request sent: VEHICLE_CMD_COMPONENT_ARM_DISARM param1=1.0'
        )

    def log_snapshot(self):
        local = self.vehicle_local_position
        status = self.vehicle_status
        target = self.hold_position or [math.nan, math.nan, math.nan]
        nav_state = getattr(status, 'nav_state', None)
        arming_state = getattr(status, 'arming_state', None)
        self.get_logger().info(
            'Diagnostic snapshot: '
            f'tick={self.timer_ticks}, '
            f'local_position_age={self.message_age_seconds(self.last_local_position_time)}s, '
            f'vehicle_status_age={self.message_age_seconds(self.last_vehicle_status_time)}s, '
            f'nav_state={constant_name(VehicleStatus, "NAVIGATION_STATE_", nav_state)}({nav_state}), '
            f'arming_state={constant_name(VehicleStatus, "ARMING_STATE_", arming_state)}({arming_state}), '
            f'failsafe={getattr(status, "failsafe", "n/a")}, '
            f'preflight_ok={getattr(status, "pre_flight_checks_pass", "n/a")}, '
            f'position=[{fmt_float(getattr(local, "x", None))}, '
            f'{fmt_float(getattr(local, "y", None))}, '
            f'{fmt_float(getattr(local, "z", None))}], '
            f'valid=[xy:{getattr(local, "xy_valid", "n/a")}, '
            f'z:{getattr(local, "z_valid", "n/a")}], '
            f'setpoint=[{fmt_float(target[0])}, {fmt_float(target[1])}, '
            f'{fmt_float(target[2])}], '
            f'offboard_active={self.is_offboard()}, armed={self.is_armed()}'
        )

    def timer_callback(self):
        self.timer_ticks += 1
        self.publish_offboard_control_mode()
        self.publish_position_setpoint()

        if self.timer_ticks % max(int(self.control_rate_hz), 1) == 0:
            self.log_snapshot()

        if not self.vehicle_status_received or not self.local_position_received:
            if self.timer_ticks % max(int(self.control_rate_hz), 1) == 0:
                self.get_logger().info(
                    'Waiting for PX4 messages before requesting OFFBOARD: '
                    f'vehicle_status_received={self.vehicle_status_received}, '
                    f'local_position_received={self.local_position_received}'
                )
            return

        if not self.is_offboard():
            if self.retry_interval_elapsed(self.last_offboard_request_time):
                self.get_logger().info('Vehicle is not in OFFBOARD; requesting OFFBOARD mode.')
                self.last_offboard_request_time = self.get_clock().now()
                self.request_offboard()
            return

        if self.offboard_active_since is None:
            self.offboard_active_since = self.get_clock().now()
            self.get_logger().info('OFFBOARD mode is active.')

        if not self.is_armed():
            if self.retry_interval_elapsed(self.last_arm_request_time):
                self.get_logger().info('Vehicle is in OFFBOARD but not armed; sending ARM request.')
                self.last_arm_request_time = self.get_clock().now()
                self.request_arm()
            return

        if self.armed_since is None:
            self.armed_since = self.get_clock().now()
            self.get_logger().info('Vehicle is armed. Continuing offboard heartbeat/setpoint.')


def main(args=None):
    print('Starting offboard_test_node...')
    rclpy.init(args=args)
    node = OffboardTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received; shutting down.')
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
