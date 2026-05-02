import rclpy

from pixhawk_py.offboard_px4 import OffboardControl


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
