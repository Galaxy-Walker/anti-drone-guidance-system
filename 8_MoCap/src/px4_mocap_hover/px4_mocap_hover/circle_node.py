"""ROS 2 node for a guarded PX4 motion-capture circle mission."""

import rclpy

from px4_mocap_hover.circle_mission import (
    CircleMission,
    CircleMissionConfig,
    CircleMissionState,
)
from px4_mocap_hover.node import MocapHoverNode


class MocapCircleNode(MocapHoverNode):
    """Take off, hover, fly one counterclockwise circle, and land."""

    NODE_NAME = 'px4_mocap_circle'
    DEFAULT_HOVER_DURATION = 3.0
    ABORTED_STATE = CircleMissionState.ABORTED
    STATE_LOG_NAME = 'Circle mission'
    STARTUP_WARNING = (
        'Automatic circle flight enabled. Consuming bridged NED position from '
        '{mocap_topic}; the aircraft will take off, fly a configured-radius '
        'counterclockwise circle, and land. Verify axes, clearance, PX4 '
        'failsafes, and manual takeover before flying with propellers.'
    )

    def _declare_parameters(self) -> None:
        super()._declare_parameters()
        self.declare_parameter('circle_radius', 0.5)
        self.declare_parameter('circle_duration', 10.0)

    def _create_mission(self) -> CircleMission:
        return CircleMission(CircleMissionConfig(
            takeoff_height=float(
                self.get_parameter('takeoff_height').value),
            hover_duration=float(
                self.get_parameter('hover_duration').value),
            circle_radius=float(
                self.get_parameter('circle_radius').value),
            circle_duration=float(
                self.get_parameter('circle_duration').value),
            position_tolerance=float(
                self.get_parameter('position_tolerance').value),
            stable_duration=float(
                self.get_parameter('stable_duration').value),
            prestream_duration=float(
                self.get_parameter('prestream_duration').value),
            command_retry_interval=float(
                self.get_parameter('command_retry_interval').value),
        ))

    def _validate_mission(self) -> None:
        config = self.mission.config
        positive_values = {
            'takeoff_height': config.takeoff_height,
            'hover_duration': config.hover_duration,
            'circle_radius': config.circle_radius,
            'circle_duration': config.circle_duration,
            'position_tolerance': config.position_tolerance,
            'stable_duration': config.stable_duration,
            'prestream_duration': config.prestream_duration,
            'command_retry_interval': config.command_retry_interval,
        }
        for name, value in positive_values.items():
            if value <= 0.0:
                raise ValueError(f'{name} must be greater than zero')


def main(args=None) -> None:
    """Run the motion-capture circle node."""
    rclpy.init(args=args)
    node = MocapCircleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
