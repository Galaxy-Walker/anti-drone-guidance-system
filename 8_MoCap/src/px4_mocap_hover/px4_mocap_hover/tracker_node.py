"""ROS 2 node for PX4 motion-capture horizontal target tracking."""

import math
from typing import Optional, Tuple

from geometry_msgs.msg import PoseStamped
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

from px4_mocap_hover.sample_gate import ConsecutiveSampleGate
from px4_mocap_hover.stability import PositionStabilityMonitor
from px4_mocap_hover.tracker_control import (
    TrackerControlConfig,
    TrackerVelocityController,
)
from px4_mocap_hover.tracker_mission import (
    TrackerMission,
    TrackerMissionConfig,
    TrackerMissionInput,
    TrackerState,
)
from px4_mocap_hover.transforms import mocap_to_ned_position


class MocapTrackerNode(Node):
    """Take off, hold altitude, and track another mocap rigid body in XY."""

    VELOCITY_STATES = {
        TrackerState.WAIT_TARGET,
        TrackerState.TRACKING,
    }

    def __init__(self) -> None:
        """Initialize subscriptions, publishers, mission, and controller."""
        super().__init__('px4_mocap_tracker')
        self._declare_parameters()

        self.self_odometry_topic = str(
            self.get_parameter('self_odometry_topic').value)
        self.target_mocap_topic = str(
            self.get_parameter('target_mocap_topic').value)
        self.self_mocap_timeout = float(
            self.get_parameter('mocap_timeout').value)
        self.target_mocap_timeout = float(
            self.get_parameter('target_mocap_timeout').value)
        self.local_position_timeout = float(
            self.get_parameter('local_position_timeout').value)
        self.required_mocap_samples = int(
            self.get_parameter('required_mocap_samples').value)
        self.target_reacquire_samples = int(
            self.get_parameter('target_reacquire_samples').value)
        prearm_stable_duration = float(
            self.get_parameter('prearm_stable_duration').value)
        prearm_position_tolerance = float(
            self.get_parameter('prearm_position_tolerance').value)
        control_rate = float(self.get_parameter('control_rate').value)
        log_rate = float(self.get_parameter('log_rate').value)
        self._validate_parameters(control_rate, log_rate)

        self.position_stability = PositionStabilityMonitor(
            prearm_stable_duration,
            prearm_position_tolerance,
        )
        self.target_gate = ConsecutiveSampleGate(
            self.target_reacquire_samples,
            self.target_mocap_timeout,
        )
        self.mission = TrackerMission(TrackerMissionConfig(
            takeoff_height=float(
                self.get_parameter('takeoff_height').value),
            position_tolerance=float(
                self.get_parameter('position_tolerance').value),
            stable_duration=float(
                self.get_parameter('stable_duration').value),
            prestream_duration=float(
                self.get_parameter('prestream_duration').value),
            command_retry_interval=float(
                self.get_parameter('command_retry_interval').value),
        ))
        self.controller = TrackerVelocityController(TrackerControlConfig(
            xy_kp=float(self.get_parameter('xy_kp').value),
            xy_deadband=float(self.get_parameter('xy_deadband').value),
            max_xy_speed=float(
                self.get_parameter('max_xy_speed').value),
            max_xy_acceleration=float(
                self.get_parameter('max_xy_acceleration').value),
            z_kp=float(self.get_parameter('z_kp').value),
            max_z_speed=float(self.get_parameter('max_z_speed').value),
        ))
        self._validate_mission_config()

        self.self_mocap: Optional[Tuple[float, float, float]] = None
        self.last_self_mocap_at: Optional[float] = None
        self.valid_self_mocap_samples = 0
        self.target_mocap: Optional[Tuple[float, float, float]] = None
        self.local_position: Optional[VehicleLocalPosition] = None
        self.last_local_position_at: Optional[float] = None
        self.vehicle_status: Optional[VehicleStatus] = None
        self.last_state = self.mission.state

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        mocap_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.offboard_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)

        self.create_subscription(
            VehicleOdometry,
            self.self_odometry_topic,
            self._self_mocap_callback,
            px4_qos,
        )
        self.create_subscription(
            PoseStamped,
            self.target_mocap_topic,
            self._target_mocap_callback,
            mocap_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._local_position_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self._vehicle_status_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self._command_ack_callback,
            px4_qos,
        )

        self.control_timer = self.create_timer(
            1.0 / control_rate, self._control)
        self.log_timer = self.create_timer(1.0 / log_rate, self._log_status)
        self.get_logger().warning(
            'Automatic tracking flight enabled. The tracker will take off and '
            'follow the target in XY. Verify rigid-body identities and axes, '
            'PX4 failsafe settings, and manual takeover before flight.')

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            'self_odometry_topic', '/fmu/in/vehicle_visual_odometry')
        self.declare_parameter(
            'target_mocap_topic', '/vrpn_mocap/RigidBody_002/pose')
        self.declare_parameter('takeoff_height', 1.2)
        self.declare_parameter('mocap_timeout', 0.5)
        self.declare_parameter('target_mocap_timeout', 0.5)
        self.declare_parameter('local_position_timeout', 0.5)
        self.declare_parameter('target_reacquire_samples', 5)
        self.declare_parameter('xy_kp', 0.8)
        self.declare_parameter('xy_deadband', 0.10)
        self.declare_parameter('max_xy_speed', 0.5)
        self.declare_parameter('max_xy_acceleration', 0.5)
        self.declare_parameter('z_kp', 1.0)
        self.declare_parameter('max_z_speed', 0.3)
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('log_rate', 1.0)
        self.declare_parameter('prearm_stable_duration', 10.0)
        self.declare_parameter('prearm_position_tolerance', 0.15)
        self.declare_parameter('position_tolerance', 0.15)
        self.declare_parameter('stable_duration', 1.0)
        self.declare_parameter('prestream_duration', 1.0)
        self.declare_parameter('command_retry_interval', 1.0)
        self.declare_parameter('required_mocap_samples', 20)

    def _validate_parameters(
            self, control_rate: float, log_rate: float) -> None:
        if control_rate <= 2.0:
            raise ValueError('control_rate must be greater than 2 Hz')
        if log_rate <= 0.0:
            raise ValueError('log_rate must be greater than zero')
        if self.self_mocap_timeout <= 0.0:
            raise ValueError('mocap_timeout must be greater than zero')
        if self.target_mocap_timeout <= 0.0:
            raise ValueError(
                'target_mocap_timeout must be greater than zero')
        if self.local_position_timeout <= 0.0:
            raise ValueError(
                'local_position_timeout must be greater than zero')
        if self.required_mocap_samples < 1:
            raise ValueError('required_mocap_samples must be at least one')
        if self.target_reacquire_samples < 1:
            raise ValueError(
                'target_reacquire_samples must be at least one')

    def _validate_mission_config(self) -> None:
        config = self.mission.config
        if config.takeoff_height <= 0.0:
            raise ValueError('takeoff_height must be greater than zero')
        if config.position_tolerance <= 0.0:
            raise ValueError('position_tolerance must be greater than zero')
        if config.stable_duration <= 0.0:
            raise ValueError('stable_duration must be greater than zero')
        if config.prestream_duration <= 0.0:
            raise ValueError('prestream_duration must be greater than zero')
        if config.command_retry_interval <= 0.0:
            raise ValueError(
                'command_retry_interval must be greater than zero')

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _now_microseconds(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _self_mocap_callback(self, odometry: VehicleOdometry) -> None:
        now = self._now_seconds()
        position = tuple(float(value) for value in odometry.position)
        if (
            odometry.pose_frame != VehicleOdometry.POSE_FRAME_NED
            or not all(math.isfinite(value) for value in position)
        ):
            self.self_mocap = None
            self.last_self_mocap_at = None
            self.valid_self_mocap_samples = 0
            self.position_stability.reset()
            self.get_logger().error(
                'Rejected self motion-capture position: expected finite NED')
            return

        if (
            self.last_self_mocap_at is not None
            and now - self.last_self_mocap_at > self.self_mocap_timeout
        ):
            self.valid_self_mocap_samples = 0
            self.position_stability.reset()
        self.self_mocap = position
        self.last_self_mocap_at = now
        self.valid_self_mocap_samples += 1

    def _target_mocap_callback(self, pose: PoseStamped) -> None:
        now = self._now_seconds()
        position = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        if not all(math.isfinite(value) for value in position):
            self.target_mocap = None
            self.target_gate.reset()
            self.get_logger().warning(
                'Rejected invalid target motion-capture position')
            return

        self.target_mocap = mocap_to_ned_position(position)
        self.target_gate.accept(now)

    def _local_position_callback(self, message: VehicleLocalPosition) -> None:
        now = self._now_seconds()
        if (
            self.last_local_position_at is None
            or now - self.last_local_position_at
            > self.local_position_timeout
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
        self_mocap_fresh = self._self_mocap_fresh(now)
        target_fresh = self._target_mocap_fresh(now)
        if not local_valid or not self_mocap_fresh:
            self.position_stability.reset()

        status = self.vehicle_status
        previous_state = self.mission.state
        output = self.mission.update(TrackerMissionInput(
            now=now,
            self_mocap_fresh=self_mocap_fresh,
            self_mocap_stable=(
                self.valid_self_mocap_samples
                >= self.required_mocap_samples
            ),
            position_stable=(
                local_valid and self.position_stability.is_stable(now)
            ),
            local_valid=local_valid,
            start_allowed=self._start_allowed(),
            position=self._local_tuple(),
            heading=(
                self.local_position.heading if local_valid else 0.0
            ),
            offboard=(
                status is not None
                and status.nav_state
                == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ),
            armed=(
                status is not None
                and status.arming_state
                == VehicleStatus.ARMING_STATE_ARMED
            ),
            target_fresh=target_fresh,
            target_ready=(
                target_fresh
                and self.target_gate.ready
            ),
        ))
        self._log_state_transition()

        if output.publish_position:
            self._publish_offboard_control_mode(position=True)
            self._publish_takeoff_target()
        if output.publish_velocity:
            if previous_state not in self.VELOCITY_STATES:
                self.controller.reset(now)
            self._publish_offboard_control_mode(velocity=True)
            self._publish_velocity_target(now)
        if output.request_offboard:
            self._publish_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0,
            )
        if output.request_arm:
            self._publish_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
            )

    def _self_mocap_fresh(self, now: float) -> bool:
        return (
            self.last_self_mocap_at is not None
            and now - self.last_self_mocap_at <= self.self_mocap_timeout
        )

    def _target_mocap_fresh(self, now: float) -> bool:
        return self.target_gate.fresh(now)

    def _local_valid(self, now: float) -> bool:
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
            and status.vehicle_type
            == VehicleStatus.VEHICLE_TYPE_ROTARY_WING
            and status.arming_state
            == VehicleStatus.ARMING_STATE_DISARMED
        )

    def _local_tuple(self) -> Tuple[float, float, float]:
        if self.local_position is None:
            return (math.nan, math.nan, math.nan)
        return (
            self.local_position.x,
            self.local_position.y,
            self.local_position.z,
        )

    def _publish_offboard_control_mode(
        self,
        *,
        position: bool = False,
        velocity: bool = False,
    ) -> None:
        message = OffboardControlMode()
        message.timestamp = self._now_microseconds()
        message.position = position
        message.velocity = velocity
        self.offboard_publisher.publish(message)

    def _publish_takeoff_target(self) -> None:
        if self.mission.takeoff_target is None:
            return
        message = TrajectorySetpoint()
        message.timestamp = self._now_microseconds()
        message.position = list(self.mission.takeoff_target[:3])
        message.velocity = [math.nan, math.nan, math.nan]
        message.acceleration = [math.nan, math.nan, math.nan]
        message.jerk = [math.nan, math.nan, math.nan]
        message.yaw = self.mission.takeoff_target[3]
        message.yawspeed = math.nan
        self.setpoint_publisher.publish(message)

    def _publish_velocity_target(self, now: float) -> None:
        if (
            self.mission.takeoff_target is None
            or self.self_mocap is None
            or self.local_position is None
        ):
            return

        target_xy = None
        if (
            self.mission.state == TrackerState.TRACKING
            and self.target_mocap is not None
        ):
            target_xy = self.target_mocap[:2]
        velocity = self.controller.update(
            now=now,
            self_xy=self.self_mocap[:2],
            target_xy=target_xy,
            current_z=self.local_position.z,
            target_z=self.mission.takeoff_target[2],
        )

        message = TrajectorySetpoint()
        message.timestamp = self._now_microseconds()
        message.position = [math.nan, math.nan, math.nan]
        message.velocity = list(velocity)
        message.acceleration = [math.nan, math.nan, math.nan]
        message.jerk = [math.nan, math.nan, math.nan]
        message.yaw = self.mission.takeoff_target[3]
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
            if self.mission.state == TrackerState.ABORTED else ''
        )
        self.get_logger().warning(
            f'Tracker state {self.last_state.name} -> '
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
            f'state={self.mission.state.name} '
            f'self_mocap_ned={self.self_mocap} '
            f'target_mocap_ned={self.target_mocap} '
            f'target_fresh={self._target_mocap_fresh(now)} '
            f'target_samples={self.target_gate.samples} '
            f'px4_ned={self._local_tuple()} '
            f'takeoff_target={self.mission.takeoff_target} '
            f'prearm_stable_for='
            f'{self.position_stability.stable_for(now):.1f}s '
            f'nav_state={nav_state} armed={armed}')


def main(args=None) -> None:
    """Run the motion-capture tracking node."""
    rclpy.init(args=args)
    node = MocapTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
