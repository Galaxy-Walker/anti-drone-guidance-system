"""ROS 2 node for a PX4 motion-capture takeoff and hover mission."""

import math
from typing import Optional, Tuple

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
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

from px4_mocap_hover.mission import (
    HoverMission,
    MissionConfig,
    MissionInput,
    MissionState,
)
from px4_mocap_hover.stability import PositionStabilityMonitor


class MocapHoverNode(Node):
    """Consume bridged mocap odometry and execute a guarded mission."""

    NODE_NAME = 'px4_mocap_hover'
    DEFAULT_TAKEOFF_HEIGHT = 1.0
    DEFAULT_HOVER_DURATION = 10.0
    DEFAULT_CONTROL_RATE = 10.0
    ABORTED_STATE = MissionState.ABORTED
    STATE_LOG_NAME = 'Mission'
    STARTUP_WARNING = (
        'Automatic flight enabled. Consuming bridged NED position from '
        '{mocap_topic}; verify bridge axes before flying with propellers.'
    )

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME)
        self._declare_parameters()

        self.mocap_topic = self.get_parameter('mocap_topic').value
        self.mocap_timeout = float(self.get_parameter('mocap_timeout').value)
        self.local_position_timeout = float(
            self.get_parameter('local_position_timeout').value)
        self.required_mocap_samples = int(
            self.get_parameter('required_mocap_samples').value)
        prearm_stable_duration = float(
            self.get_parameter('prearm_stable_duration').value)
        prearm_position_tolerance = float(
            self.get_parameter('prearm_position_tolerance').value)
        control_rate = float(self.get_parameter('control_rate').value)
        log_rate = float(self.get_parameter('log_rate').value)
        if control_rate <= 2.0:
            raise ValueError('control_rate must be greater than 2 Hz')
        if log_rate <= 0.0:
            raise ValueError('log_rate must be greater than zero')
        if self.mocap_timeout <= 0.0:
            raise ValueError('mocap_timeout must be greater than zero')
        if self.local_position_timeout <= 0.0:
            raise ValueError(
                'local_position_timeout must be greater than zero')
        if self.required_mocap_samples < 1:
            raise ValueError('required_mocap_samples must be at least one')
        self.position_stability = PositionStabilityMonitor(
            prearm_stable_duration,
            prearm_position_tolerance,
        )

        self.mission = self._create_mission()
        self._validate_mission()

        self.ned_mocap: Optional[Tuple[float, float, float]] = None
        self.last_mocap_at: Optional[float] = None
        self.valid_mocap_samples = 0
        self.local_position: Optional[VehicleLocalPosition] = None
        self.last_local_position_at: Optional[float] = None
        self.vehicle_status: Optional[VehicleStatus] = None
        self.last_state = self.mission.state
        self.shutdown_requested = False

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
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

        self.control_timer = self.create_timer(
            1.0 / control_rate, self._control)
        self.log_timer = self.create_timer(1.0 / log_rate, self._log_status)
        self.get_logger().warning(
            self.STARTUP_WARNING.format(mocap_topic=self.mocap_topic))

    def _create_mission(self) -> HoverMission:
        config = MissionConfig(
            takeoff_height=float(self.get_parameter('takeoff_height').value),
            hover_duration=float(self.get_parameter('hover_duration').value),
            position_tolerance=float(
                self.get_parameter('position_tolerance').value),
            stable_duration=float(self.get_parameter('stable_duration').value),
            prestream_duration=float(
                self.get_parameter('prestream_duration').value),
            command_retry_interval=float(
                self.get_parameter('command_retry_interval').value),
        )
        return HoverMission(config)

    def _validate_mission(self) -> None:
        """Allow specialized missions to validate additional parameters."""

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            'mocap_topic', '/fmu/in/vehicle_visual_odometry')
        self.declare_parameter(
            'takeoff_height', self.DEFAULT_TAKEOFF_HEIGHT)
        self.declare_parameter(
            'hover_duration', self.DEFAULT_HOVER_DURATION)
        self.declare_parameter('mocap_timeout', 0.5)
        self.declare_parameter('local_position_timeout', 0.5)
        self.declare_parameter('control_rate', self.DEFAULT_CONTROL_RATE)
        self.declare_parameter('log_rate', 1.0)
        self.declare_parameter('prearm_stable_duration', 10.0)
        self.declare_parameter('prearm_position_tolerance', 0.15)
        self.declare_parameter('position_tolerance', 0.15)
        self.declare_parameter('stable_duration', 1.0)
        self.declare_parameter('prestream_duration', 1.0)
        self.declare_parameter('command_retry_interval', 1.0)
        self.declare_parameter('required_mocap_samples', 20)

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _now_microseconds(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _mocap_callback(self, odometry: VehicleOdometry) -> None:
        now = self._now_seconds()
        position = tuple(float(value) for value in odometry.position)
        if (
            odometry.pose_frame != VehicleOdometry.POSE_FRAME_NED
            or not all(math.isfinite(value) for value in position)
        ):
            self.valid_mocap_samples = 0
            self.position_stability.reset()
            self.get_logger().error(
                'Rejected bridged motion-capture position: '
                'expected finite NED')
            return

        if (
            self.last_mocap_at is not None
            and now - self.last_mocap_at > self.mocap_timeout
        ):
            self.valid_mocap_samples = 0
            self.position_stability.reset()
        self.ned_mocap = position
        self.last_mocap_at = now
        self.valid_mocap_samples += 1

    def _local_position_callback(self, message: VehicleLocalPosition) -> None:
        now = self._now_seconds()
        if (
            self.last_local_position_at is None
            or now - self.last_local_position_at > self.local_position_timeout
        ):
            self.position_stability.reset()
        self.local_position = message
        self.last_local_position_at = now
        self.position_stability.update(
            now,
            self._local_tuple(),
            valid=self._local_message_valid(message),
        )

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
        local_valid = self._local_valid(now)
        if not local_valid:
            self.position_stability.reset()
        position = self._local_tuple()
        status = self.vehicle_status
        mocap_fresh = (
            self.last_mocap_at is not None
            and now - self.last_mocap_at <= self.mocap_timeout
        )
        if not mocap_fresh:
            self.position_stability.reset()
        data = MissionInput(
            now=now,
            mocap_fresh=mocap_fresh,
            mocap_stable=(
                self.valid_mocap_samples >= self.required_mocap_samples
            ),
            position_stable=(
                local_valid and self.position_stability.is_stable(now)
            ),
            local_valid=local_valid,
            start_allowed=self._start_allowed(),
            position=position,
            heading=self.local_position.heading if local_valid else 0.0,
            offboard=(
                status is not None
                and status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ),
            armed=(
                status is not None
                and status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            ),
            landing=self._landing(),
        )
        output = self.mission.update(data)
        self._log_state_transition()

        if output.publish_offboard:
            self._publish_offboard_control_mode()
            self._publish_target()
        if output.request_offboard:
            self._publish_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        if output.request_arm:
            self._publish_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        if output.request_land:
            self._publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        if output.shutdown and not self.shutdown_requested:
            self.shutdown_requested = True
            self.get_logger().info('Mission complete; PX4 is disarmed')
            self.create_timer(0.2, self._shutdown)

    def _local_valid(self, now: Optional[float] = None) -> bool:
        if now is None:
            now = self._now_seconds()
        return (
            self.local_position is not None
            and self.last_local_position_at is not None
            and now - self.last_local_position_at
            <= self.local_position_timeout
            and self._local_message_valid(self.local_position)
        )

    @staticmethod
    def _local_message_valid(message: VehicleLocalPosition) -> bool:
        return (
            message.xy_valid
            and message.z_valid
            and all(math.isfinite(value) for value in (
                message.x,
                message.y,
                message.z,
                message.heading,
            ))
        )

    def _start_allowed(self) -> bool:
        status = self.vehicle_status
        return (
            status is not None
            and status.pre_flight_checks_pass
            and status.vehicle_type == VehicleStatus.VEHICLE_TYPE_ROTARY_WING
            and status.arming_state == VehicleStatus.ARMING_STATE_DISARMED
        )

    def _landing(self) -> bool:
        status = self.vehicle_status
        return (
            status is not None
            and status.nav_state in {
                VehicleStatus.NAVIGATION_STATE_AUTO_LAND,
                VehicleStatus.NAVIGATION_STATE_AUTO_PRECLAND,
                VehicleStatus.NAVIGATION_STATE_DESCEND,
            }
        )

    def _local_tuple(self) -> Tuple[float, float, float]:
        if self.local_position is None:
            return (math.nan, math.nan, math.nan)
        return (
            self.local_position.x,
            self.local_position.y,
            self.local_position.z,
        )

    def _publish_offboard_control_mode(self) -> None:
        message = OffboardControlMode()
        message.timestamp = self._now_microseconds()
        message.position = True
        self.offboard_publisher.publish(message)

    def _publish_target(self) -> None:
        if self.mission.target is None:
            return
        message = TrajectorySetpoint()
        message.timestamp = self._now_microseconds()
        message.position = list(self.mission.target[:3])
        message.velocity = [math.nan, math.nan, math.nan]
        message.acceleration = [math.nan, math.nan, math.nan]
        message.jerk = [math.nan, math.nan, math.nan]
        message.yaw = self.mission.target[3]
        message.yawspeed = math.nan
        self.setpoint_publisher.publish(message)

    def _publish_command(self, command: int, **params: float) -> None:
        message = VehicleCommand()
        message.timestamp = self._now_microseconds()
        message.command = command
        message.param1 = params.get('param1', 0.0)
        message.param2 = params.get('param2', 0.0)
        message.param3 = params.get('param3', 0.0)
        message.param4 = params.get('param4', 0.0)
        message.param5 = params.get('param5', 0.0)
        message.param6 = params.get('param6', 0.0)
        message.param7 = params.get('param7', 0.0)
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.command_publisher.publish(message)
        self.get_logger().info(f'Sent PX4 command={command} params={params}')

    def _log_state_transition(self) -> None:
        if self.mission.state == self.last_state:
            return
        detail = (
            f': {self.mission.abort_reason}'
            if self.mission.state == self.ABORTED_STATE else ''
        )
        self.get_logger().warning(
            f'{self.STATE_LOG_NAME} state {self.last_state.name} -> '
            f'{self.mission.state.name}{detail}')
        self.last_state = self.mission.state

    def _log_status(self) -> None:
        now = self._now_seconds()
        status = self.vehicle_status
        nav_state = status.nav_state if status is not None else None
        armed = (
            status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            if status is not None else None
        )
        self.get_logger().info(
            f'state={self.mission.state.name} mocap_ned={self.ned_mocap} '
            f'px4_ned={self._local_tuple()} target={self.mission.target} '
            f'prearm_stable_for='
            f'{self.position_stability.stable_for(now):.1f}s '
            f'nav_state={nav_state} armed={armed}')

    def _shutdown(self) -> None:
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    """Run the motion-capture hover node."""
    rclpy.init(args=args)
    node = MocapHoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
