"""Record a VRPN motion-capture trajectory as NED coordinates in CSV."""

import csv
from typing import Optional

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from px4_mocap_hover.trajectory_csv import (
    TrajectoryCsvWriter,
    valid_mocap_ned_position,
)


class MocapTrajectoryRecorderNode(Node):
    """Record each valid target motion-capture position in absolute NED."""

    def __init__(self) -> None:
        """Initialize parameters, the lazy CSV writer, and subscription."""
        super().__init__('mocap_trajectory_recorder')
        self.declare_parameter(
            'mocap_topic', '/vrpn_mocap/RigidBody_002/pose')
        self.declare_parameter(
            'output_directory', '/home/nvidia/ws_ros2/trajectory_csv')
        self.declare_parameter('file_prefix', 'target_trajectory')

        mocap_topic = str(self.get_parameter('mocap_topic').value)
        output_directory = str(
            self.get_parameter('output_directory').value)
        file_prefix = str(self.get_parameter('file_prefix').value)
        if not mocap_topic:
            raise ValueError('mocap_topic must not be empty')

        self.writer = TrajectoryCsvWriter(output_directory, file_prefix)
        self._fatal_error: Optional[str] = None

        mocap_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.subscription = self.create_subscription(
            PoseStamped, mocap_topic, self._pose_callback, mocap_qos)
        self.get_logger().info(
            f'Ready to record {mocap_topic} in absolute NED coordinates; '
            'the CSV file will be created when the first valid sample arrives')

    def _pose_callback(self, pose: PoseStamped) -> None:
        if self._fatal_error is not None:
            return

        ned_position = valid_mocap_ned_position((
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ))
        if ned_position is None:
            self.get_logger().warning(
                'Rejected invalid target motion-capture position')
            return

        ros_time_ns = self.get_clock().now().nanoseconds
        try:
            path = self.writer.record(ros_time_ns, ned_position)
        except (OSError, csv.Error, ValueError) as error:
            self._fatal_error = str(error)
            self.get_logger().error(
                f'Failed to write trajectory CSV: {error}; stopping node')
            try:
                self.writer.close()
            except OSError as close_error:
                self.get_logger().error(
                    f'Failed to close trajectory CSV: {close_error}')
            if rclpy.ok(context=self.context):
                rclpy.shutdown(context=self.context)
            return

        if self.writer.sample_count == 1:
            self.get_logger().info(f'Recording trajectory to {path}')

    def close(self) -> None:
        """Close the CSV and report the final recording result."""
        path = self.writer.path
        sample_count = self.writer.sample_count
        self.writer.close()
        if path is None:
            message = (
                'No valid motion-capture samples received; no CSV created')
        else:
            message = f'Saved {sample_count} trajectory samples to {path}'

        if rclpy.ok(context=self.context):
            self.get_logger().info(message)
        else:
            print(f'[mocap_trajectory_recorder] {message}')


def main(args=None) -> None:
    """Run the target motion-capture trajectory recorder."""
    rclpy.init(args=args)
    node = MocapTrajectoryRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
