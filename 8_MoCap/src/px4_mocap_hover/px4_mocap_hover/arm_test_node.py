"""ROS 2 node that safely tests PX4 arming without commanding takeoff."""

import math
from typing import Optional, Tuple

from px4_msgs.msg import (
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from px4_mocap_hover.arm_test_mission import (
    ArmTestConfig,
    ArmTestInput,
    ArmTestMission,
)


class ArmTestNode(Node):
    """Check motion-capture/PX4 health, arm briefly, then disarm."""

    SAFE_NAV_STATES = {
        VehicleStatus.NAVIGATION_STATE_MANUAL,
        VehicleStatus.NAVIGATION_STATE_ALTCTL,
        VehicleStatus.NAVIGATION_STATE_POSCTL,
        VehicleStatus.NAVIGATION_STATE_ACRO,
        VehicleStatus.NAVIGATION_STATE_STAB,
    }

    def __init__(self) -> None:
        super().__init__('px4_mocap_arm_test')
        self._declare_parameters()

        self.mocap_topic = str(self.get_parameter('mocap_topic').value)
        self.mocap_timeout = float(self.get_parameter('mocap_timeout').value)
        self.required_mocap_samples = int(
            self.get_parameter('required_mocap_samples').value)
        control_rate = float(self.get_parameter('control_rate').value)
        log_rate = float(self.get_parameter('log_rate').value)
        if control_rate <= 0.0:
            raise ValueError('control_rate must be greater than zero')
        if log_rate <= 0.0:
            raise ValueError('log_rate must be greater than zero')
        if self.mocap_timeout <= 0.0:
            raise ValueError('mocap_timeout must be greater than zero')
        if self.required_mocap_samples < 1:
            raise ValueError('required_mocap_samples must be at least one')

        config = ArmTestConfig(
            armed_duration=float(self.get_parameter('armed_duration').value),
            arm_request_timeout=float(
                self.get_parameter('arm_request_timeout').value),
            command_retry_interval=float(
                self.get_parameter('command_retry_interval').value),
        )
        if config.armed_duration <= 0.0:
            raise ValueError('armed_duration must be greater than zero')
        if config.arm_request_timeout <= 0.0:
            raise ValueError('arm_request_timeout must be greater than zero')
        if config.command_retry_interval <= 0.0:
            raise ValueError('command_retry_interval must be greater than zero')
        self.test = ArmTestMission(config)

        self.ned_mocap: Optional[Tuple[float, float, float]] = None
        self.last_mocap_at: Optional[float] = None
        self.valid_mocap_samples = 0
        self.local_position: Optional[VehicleLocalPosition] = None
        self.vehicle_status: Optional[VehicleStatus] = None
        self.last_state = self.test.state
        self.shutdown_requested = False

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)
        self.create_subscription(
            VehicleOdometry, self.mocap_topic, self._mocap_callback, px4_qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_position_callback, px4_qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._vehicle_status_callback, px4_qos)
        self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack',
            self._command_ack_callback, px4_qos)

        self.control_timer = self.create_timer(1.0 / control_rate, self._control)
        self.log_timer = self.create_timer(1.0 / log_rate, self._log_status)
        self.get_logger().warning(
            'ARM-ONLY TEST ENABLED. No Offboard mode or trajectory setpoints '
            'will be published. Remove propellers before testing.')

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            'mocap_topic', '/fmu/in/vehicle_visual_odometry')
        self.declare_parameter('mocap_timeout', 0.5)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('log_rate', 1.0)
        self.declare_parameter('command_retry_interval', 1.0)
        self.declare_parameter('required_mocap_samples', 20)
        self.declare_parameter('armed_duration', 3.0)
        self.declare_parameter('arm_request_timeout', 10.0)

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _now_microseconds(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _mocap_callback(self, odometry: VehicleOdometry) -> None:
        position = tuple(float(value) for value in odometry.position)
        if (
            odometry.pose_frame != VehicleOdometry.POSE_FRAME_NED
            or not all(math.isfinite(value) for value in position)
        ):
            self.valid_mocap_samples = 0
            self.get_logger().error(
                'Rejected bridged motion-capture position: expected finite NED')
            return

        self.ned_mocap = position
        self.last_mocap_at = self._now_seconds()
        self.valid_mocap_samples += 1

    def _local_position_callback(self, message: VehicleLocalPosition) -> None:
        self.local_position = message

    def _vehicle_status_callback(self, message: VehicleStatus) -> None:
        self.vehicle_status = message

    def _command_ack_callback(self, message: VehicleCommandAck) -> None:
        result_names = {
            VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED: 'ACCEPTED',
            VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
                'TEMPORARILY_REJECTED',
            VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: 'DENIED',
            VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: 'UNSUPPORTED',
            VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: 'FAILED',
            VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS: 'IN_PROGRESS',
            VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED: 'CANCELLED',
        }
        self.get_logger().info(
            f'Command ACK command={message.command} '
            f'result={result_names.get(message.result, message.result)} '
            f'param1={message.result_param1} param2={message.result_param2}')

    def _control(self) -> None:
        now = self._now_seconds()
        status = self.vehicle_status
        armed = (
            status is not None
            and status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )
        mocap_fresh = (
            self.last_mocap_at is not None
            and now - self.last_mocap_at <= self.mocap_timeout
        )
        armed_safety_ok = (
            mocap_fresh
            and self._local_valid()
            and self._safe_nav_state()
            and status is not None
            and status.vehicle_type == VehicleStatus.VEHICLE_TYPE_ROTARY_WING
        )
        prearm_safety_ok = (
            armed_safety_ok
            and status is not None
            and status.pre_flight_checks_pass
        )
        output = self.test.update(ArmTestInput(
            now=now,
            data_ready=(
                prearm_safety_ok
                and self.valid_mocap_samples >= self.required_mocap_samples
                and status is not None
                and status.arming_state
                == VehicleStatus.ARMING_STATE_DISARMED
            ),
            prearm_safety_ok=prearm_safety_ok,
            armed_safety_ok=armed_safety_ok,
            armed=armed,
        ))
        self._log_state_transition()

        if output.request_arm:
            self._publish_arm_command(True)
        if output.request_disarm:
            self._publish_arm_command(False)
        if output.shutdown and not self.shutdown_requested:
            self.shutdown_requested = True
            self.get_logger().info('Arm-only test finished')
            self.create_timer(0.2, self._shutdown)

    def _local_valid(self) -> bool:
        return (
            self.local_position is not None
            and self.local_position.xy_valid
            and self.local_position.z_valid
            and all(math.isfinite(value) for value in self._local_tuple())
            and math.isfinite(self.local_position.heading)
        )

    def _safe_nav_state(self) -> bool:
        return (
            self.vehicle_status is not None
            and self.vehicle_status.nav_state in self.SAFE_NAV_STATES
        )

    def _local_tuple(self) -> Tuple[float, float, float]:
        if self.local_position is None:
            return (math.nan, math.nan, math.nan)
        return (
            self.local_position.x,
            self.local_position.y,
            self.local_position.z,
        )

    def _publish_arm_command(self, arm: bool) -> None:
        message = VehicleCommand()
        message.timestamp = self._now_microseconds()
        message.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        message.param1 = 1.0 if arm else 0.0
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.command_publisher.publish(message)
        self.get_logger().warning(
            'Sent PX4 ARM command' if arm else 'Sent PX4 DISARM command')

    def request_disarm(self) -> None:
        """Request disarm during an orderly interruption."""
        self._publish_arm_command(False)

    def _log_state_transition(self) -> None:
        if self.test.state == self.last_state:
            return
        detail = (
            f': {self.test.abort_reason}'
            if self.test.abort_reason else ''
        )
        self.get_logger().warning(
            f'Arm test state {self.last_state.name} -> '
            f'{self.test.state.name}{detail}')
        self.last_state = self.test.state

    def _log_status(self) -> None:
        status = self.vehicle_status
        nav_state = status.nav_state if status is not None else None
        armed = (
            status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            if status is not None else None
        )
        self.get_logger().info(
            f'state={self.test.state.name} mocap_ned={self.ned_mocap} '
            f'px4_ned={self._local_tuple()} nav_state={nav_state} armed={armed}')

    def _shutdown(self) -> None:
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    """Run the PX4 arm-only test node."""
    rclpy.init(args=args)
    node = ArmTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.request_disarm()
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
