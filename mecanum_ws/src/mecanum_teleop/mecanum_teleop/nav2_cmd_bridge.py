#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped


class Nav2CmdBridge(Node):

    def __init__(self):
        super().__init__('nav2_cmd_bridge')

        self.sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel_nav',
            self.cmd_callback,
            10
        )

        self.pub = self.create_publisher(
            TwistStamped,
            '/autonomous_cmd_vel',
            10
        )

        self.get_logger().info(
            'NAV2 CMD BRIDGE READY'
        )

        self.get_logger().info(
            '/cmd_vel_nav -> /autonomous_cmd_vel'
        )

    def cmd_callback(self, msg):

        self.pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = Nav2CmdBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
