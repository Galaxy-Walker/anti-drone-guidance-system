#!/usr/bin/env python3

import math
import enum

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


def constant_value(msg_type, name, fallback):
    return getattr(msg_type, name, fallback)


class FlightState(enum.IntEnum):
    WAITING_FOR_DATA = 0
    REQUESTING_OFFBOARD = 1
    REQUESTING_ARM = 2
    TAKEOFF = 3
    HOVER = 4
    LANDING = 5
    DISARMING = 6
    DONE = 7


class OffboardTakeoffNode(Node):
    """Offboard node: arm, takeoff to 1m, hover 10s, land, disarm."""

    def __init__(self):
        super().__init__('offboard_takeoff_node')

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
        self.takeoff_height = float(self.declare_parameter('takeoff_height', 1.0).value)
        self.hover_duration_sec = float(self.declare_parameter('hover_duration_sec', 10.0).value)
        self.position_reached_threshold = float(
            self.declare_parameter('position_reached_threshold', 0.2).value)

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
        self.takeoff_ground_z = None

        self.flight_state = FlightState.WAITING_FOR_DATA
        self.state_start_time = None

        self.timer_ticks = 0
        self.last_offboard_request_time = None
        self.last_arm_request_time = None
        self.last_disarm_request_time = None

        self.offboard_nav_state = constant_value(
            VehicleStatus, 'NAVIGATION_STATE_OFFBOARD', 14)
        self.armed_state = constant_value(VehicleStatus, 'ARMING_STATE_ARMED', 2)

        timer_period = 1.0 / max(self.control_rate_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.log_startup_summary(timer_period)

    def log_startup_summary(self, timer_period):
        self.get_logger().info('=' * 70)
        self.get_logger().info('PX4 offboard takeoff-hover-land node started')
        self.get_logger().info('=' * 70)
        self.get_logger().info(
            f'takeoff_height={self.takeoff_height}m, '
            f'hover_duration={self.hover_duration_sec}s, '
            f'threshold={self.position_reached_threshold}m'
        )
        self.get_logger().info(
            f'Control rate: {1.0 / timer_period:.1f} Hz; '
            f'command_retry_interval_sec={self.command_retry_interval_sec:.1f}'
        )
        self.get_logger().info(
            'Sequence: OFFBOARD -> ARM -> takeoff to 1m -> hover 10s -> land -> disarm'
        )

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        if not self.local_position_received:
            self.local_position_received = True
            self.hold_position = [float(msg.x), float(msg.y), float(msg.z)]
            self.takeoff_ground_z = float(msg.z)
            self.get_logger().info(
                f'First vehicle_local_position: '
                f'pos=[{fmt_float(msg.x)}, {fmt_float(msg.y)}, {fmt_float(msg.z)}], '
                f'ground_z={fmt_float(self.takeoff_ground_z)}'
            )

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        if not self.vehicle_status_received:
            self.vehicle_status_received = True
            self.get_logger().info('First vehicle_status received.')

    def timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

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

    def request_offboard(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.get_logger().info('OFFBOARD mode requested.')

    def request_arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )
        self.get_logger().info('ARM requested.')

    def request_disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
        )
        self.get_logger().info('DISARM requested.')

    def position_error(self):
        if self.hold_position is None:
            return float('inf')
        local = self.vehicle_local_position
        dx = float(getattr(local, 'x', 0)) - self.hold_position[0]
        dy = float(getattr(local, 'y', 0)) - self.hold_position[1]
        dz = float(getattr(local, 'z', 0)) - self.hold_position[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def transit(self, new_state):
        self.get_logger().info(
            f'State transition: {self.flight_state.name} -> {new_state.name}'
        )
        self.flight_state = new_state
        self.state_start_time = self.get_clock().now()

    def state_elapsed_seconds(self):
        if self.state_start_time is None:
            return 0.0
        return (self.get_clock().now().nanoseconds - self.state_start_time.nanoseconds) / 1e9

    def timer_callback(self):
        self.timer_ticks += 1
        self.publish_offboard_control_mode()
        self.publish_position_setpoint()

        if self.timer_ticks % max(int(self.control_rate_hz), 1) == 0:
            self.log_status()

        state = self.flight_state

        if state == FlightState.WAITING_FOR_DATA:
            if self.vehicle_status_received and self.local_position_received:
                self.transit(FlightState.REQUESTING_OFFBOARD)

        elif state == FlightState.REQUESTING_OFFBOARD:
            if self.is_offboard():
                self.get_logger().info('OFFBOARD mode active.')
                self.transit(FlightState.REQUESTING_ARM)
            elif self.retry_interval_elapsed(self.last_offboard_request_time):
                self.last_offboard_request_time = self.get_clock().now()
                self.request_offboard()

        elif state == FlightState.REQUESTING_ARM:
            if self.is_armed():
                self.get_logger().info('Armed successfully.')
                target_z = self.takeoff_ground_z - self.takeoff_height
                self.hold_position = [self.hold_position[0], self.hold_position[1], target_z]
                self.get_logger().info(
                    f'Takeoff target set: z={fmt_float(target_z)} '
                    f'(ground_z={fmt_float(self.takeoff_ground_z)})'
                )
                self.transit(FlightState.TAKEOFF)
            elif self.retry_interval_elapsed(self.last_arm_request_time):
                self.last_arm_request_time = self.get_clock().now()
                self.request_arm()

        elif state == FlightState.TAKEOFF:
            err = self.position_error()
            if err < self.position_reached_threshold:
                self.get_logger().info(
                    f'Reached takeoff altitude (err={fmt_float(err)}m).'
                )
                self.transit(FlightState.HOVER)

        elif state == FlightState.HOVER:
            elapsed = self.state_elapsed_seconds()
            if self.timer_ticks % max(int(self.control_rate_hz), 1) == 0:
                remaining = self.hover_duration_sec - elapsed
                self.get_logger().info(
                    f'Hovering: {fmt_float(elapsed)}s / {fmt_float(self.hover_duration_sec)}s, '
                    f'z={fmt_float(getattr(self.vehicle_local_position, "z", 0))}'
                )
            if elapsed >= self.hover_duration_sec:
                self.hold_position = [
                    self.hold_position[0],
                    self.hold_position[1],
                    self.takeoff_ground_z,
                ]
                self.get_logger().info(
                    f'Landing target: z={fmt_float(self.takeoff_ground_z)} (ground)'
                )
                self.transit(FlightState.LANDING)

        elif state == FlightState.LANDING:
            err = self.position_error()
            if err < self.position_reached_threshold:
                self.get_logger().info(
                    f'Reached ground (err={fmt_float(err)}m). Disarming...'
                )
                self.transit(FlightState.DISARMING)

        elif state == FlightState.DISARMING:
            if not self.is_armed():
                self.get_logger().info('Disarmed. Mission complete.')
                self.transit(FlightState.DONE)
            elif self.retry_interval_elapsed(self.last_disarm_request_time):
                self.last_disarm_request_time = self.get_clock().now()
                self.request_disarm()

        elif state == FlightState.DONE:
            pass

    def log_status(self):
        state = self.flight_state
        local = self.vehicle_local_position
        self.get_logger().info(
            f'State={state.name}, tick={self.timer_ticks}, '
            f'pos=[{fmt_float(getattr(local, "x", 0))}, '
            f'{fmt_float(getattr(local, "y", 0))}, '
            f'{fmt_float(getattr(local, "z", 0))}], '
            f'setpoint_z={fmt_float(self.hold_position[2]) if self.hold_position else "n/a"}, '
            f'offboard={self.is_offboard()}, armed={self.is_armed()}'
        )


def main(args=None):
    print('Starting offboard_takeoff_node...')
    rclpy.init(args=args)
    node = OffboardTakeoffNode()
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
