"""ROS 2 node for guarded replay of an absolute-NED trajectory CSV."""

import rclpy
from px4_msgs.msg import VehicleStatus

from px4_mocap_hover.node import MocapHoverNode
from px4_mocap_hover.trajectory_replay import load_trajectory_csv
from px4_mocap_hover.trajectory_replay_mission import (
    TrajectoryReplayConfig,
    TrajectoryReplayMission,
    TrajectoryReplayState,
)


class MocapTrajectoryReplayNode(MocapHoverNode):
    """Take off, replay every CSV XY point at fixed height, and land."""

    NODE_NAME = 'px4_mocap_trajectory_replay'
    DEFAULT_TAKEOFF_HEIGHT = 1.2
    DEFAULT_HOVER_DURATION = 3.0
    DEFAULT_CONTROL_RATE = 60.0
    ABORTED_STATE = TrajectoryReplayState.ABORTED
    STATE_LOG_NAME = 'Trajectory replay'
    STARTUP_WARNING = (
        'Automatic absolute-NED trajectory replay enabled. Consuming bridged '
        'self position from {mocap_topic}; verify that the PX4 local XY frame '
        'matches the CSV absolute NED frame, and verify clearance, failsafes, '
        'and manual takeover before flying with propellers.'
    )

    def _declare_parameters(self) -> None:
        super()._declare_parameters()
        self.declare_parameter('trajectory_file', '')

    def _create_mission(self) -> TrajectoryReplayMission:
        self.trajectory_file = str(
            self.get_parameter('trajectory_file').value)
        self.trajectory = load_trajectory_csv(self.trajectory_file)
        return TrajectoryReplayMission(
            TrajectoryReplayConfig(
                takeoff_height=float(
                    self.get_parameter('takeoff_height').value),
                hover_duration=float(
                    self.get_parameter('hover_duration').value),
                position_tolerance=float(
                    self.get_parameter('position_tolerance').value),
                stable_duration=float(
                    self.get_parameter('stable_duration').value),
                prestream_duration=float(
                    self.get_parameter('prestream_duration').value),
                command_retry_interval=float(
                    self.get_parameter('command_retry_interval').value),
            ),
            self.trajectory,
        )

    def _validate_mission(self) -> None:
        config = self.mission.config
        positive_values = {
            'takeoff_height': config.takeoff_height,
            'hover_duration': config.hover_duration,
            'position_tolerance': config.position_tolerance,
            'stable_duration': config.stable_duration,
            'prestream_duration': config.prestream_duration,
            'command_retry_interval': config.command_retry_interval,
        }
        for name, value in positive_values.items():
            if value <= 0.0:
                raise ValueError(f'{name} must be greater than zero')

    def _log_status(self) -> None:
        now = self._now_seconds()
        status = self.vehicle_status
        nav_state = status.nav_state if status is not None else None
        armed = (
            status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            if status is not None else None
        )
        total = len(self.trajectory)
        row = self.mission.playback_row
        progress = 100.0 * row / total
        self.get_logger().info(
            f'state={self.mission.state.name} '
            f'playback_row={row}/{total} progress={progress:.1f}% '
            f'mocap_ned={self.ned_mocap} '
            f'px4_ned={self._local_tuple()} target={self.mission.target} '
            f'prearm_stable_for='
            f'{self.position_stability.stable_for(now):.1f}s '
            f'nav_state={nav_state} armed={armed} '
            f'trajectory_file={self.trajectory_file}')


def main(args=None) -> None:
    """Run the absolute-NED trajectory replay node."""
    rclpy.init(args=args)
    node = MocapTrajectoryReplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
