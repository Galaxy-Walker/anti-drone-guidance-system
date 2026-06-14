"""Bridge a VRPN ROS pose to PX4 motion-capture odometry."""

import math

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from px4_mocap_hover.transforms import mocap_to_ned_position


IDENTITY_QUATERNION_WXYZ = [1.0, 0.0, 0.0, 0.0]


class MocapBridgeNode(Node):
    """Convert motion-capture PoseStamped positions to PX4 NED odometry."""

    def __init__(self) -> None:
        super().__init__('px4_mocap_bridge')
        self._declare_parameters()

        mocap_topic = str(self.get_parameter('mocap_topic').value)
        px4_topic = str(self.get_parameter('px4_topic').value)
        self.position_variance = float(
            self.get_parameter('position_variance').value)
        self.orientation_variance = float(
            self.get_parameter('orientation_variance').value)
        self.quality = int(self.get_parameter('quality').value)
        if self.position_variance < 0.0:
            raise ValueError('position_variance must not be negative')
        if self.orientation_variance < 0.0:
            raise ValueError('orientation_variance must not be negative')
        if not -1 <= self.quality <= 100:
            raise ValueError('quality must be between -1 and 100')

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

        self.publisher = self.create_publisher(
            VehicleOdometry, px4_topic, px4_qos)
        self.create_subscription(
            PoseStamped, mocap_topic, self._pose_callback, mocap_qos)

        self.get_logger().info(
            f'Bridging {mocap_topic} -> {px4_topic}; '
            'converting mocap axes (x, y, z) to NED (x, z, -y) and publishing '
            'a fixed identity attitude; configure PX4 to fuse position only')

    def _declare_parameters(self) -> None:
        self.declare_parameter('mocap_topic', '/vrpn_mocap/RigidBody/pose')
        self.declare_parameter(
            'px4_topic', '/fmu/in/vehicle_visual_odometry')
        self.declare_parameter('position_variance', 0.0025)
        self.declare_parameter('orientation_variance', 0.01)
        self.declare_parameter('quality', 100)

    def _pose_callback(self, pose: PoseStamped) -> None:
        position = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        if not all(math.isfinite(value) for value in position):
            self.get_logger().warning('Rejected invalid motion-capture position')
            return

        timestamp = self.get_clock().now().nanoseconds // 1000
        odometry = VehicleOdometry()
        odometry.timestamp = timestamp
        odometry.timestamp_sample = timestamp
        odometry.pose_frame = VehicleOdometry.POSE_FRAME_NED
        odometry.position = list(mocap_to_ned_position(position))
        odometry.q = IDENTITY_QUATERNION_WXYZ
        odometry.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        odometry.velocity = [math.nan, math.nan, math.nan]
        odometry.angular_velocity = [math.nan, math.nan, math.nan]
        odometry.position_variance = [self.position_variance] * 3
        odometry.orientation_variance = [self.orientation_variance] * 3
        odometry.velocity_variance = [math.nan, math.nan, math.nan]
        odometry.quality = self.quality
        self.publisher.publish(odometry)


def main(args=None) -> None:
    """Run the standalone motion-capture bridge."""
    rclpy.init(args=args)
    node = MocapBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
