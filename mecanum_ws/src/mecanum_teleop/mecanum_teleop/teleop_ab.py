#!/usr/bin/env python3

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool


class MecanumTeleopAB(Node):

    def __init__(self):
        super().__init__('mecanum_teleop_ab')

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        self.auto_pub = self.create_publisher(
            Bool,
            '/autonomous_enable',
            10
        )

        self.timer = self.create_timer(0.05, self.publish_cmd)

        self.manual_mode = True

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        self.speed = 0.20
        self.turn_speed = 0.50

        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info('======================================')
        self.get_logger().info(' MECANUM A/B TELEOPERATION READY')
        self.get_logger().info(' A = MANUAL TELEOPERATION')
        self.get_logger().info(' B = AUTONOMOUS MODE')
        self.get_logger().info(' W/S = Forward / Backward')
        self.get_logger().info(' D/F = Strafe Right / Left')
        self.get_logger().info(' Q/E = Rotate Left / Right')
        self.get_logger().info(' X = STOP')
        self.get_logger().info(' CTRL+C = EXIT')
        self.get_logger().info('======================================')

    def publish_cmd(self):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        if self.manual_mode:
            msg.twist.linear.x = self.linear_x
            msg.twist.linear.y = self.linear_y
            msg.twist.angular.z = self.angular_z

        else:
            # Autonomous node owns /cmd_vel while B is active.
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.angular.z = 0.0

        self.cmd_pub.publish(msg)

    def set_manual(self):
        if not self.manual_mode:
            self.stop_robot()

        self.manual_mode = True

        msg = Bool()
        msg.data = False
        self.auto_pub.publish(msg)

        self.get_logger().info('A pressed -> MANUAL TELEOPERATION')

    def set_autonomous(self):
        self.stop_robot()

        self.manual_mode = False

        msg = Bool()
        msg.data = True
        self.auto_pub.publish(msg)

        self.get_logger().info('B pressed -> AUTONOMOUS MODE')

    def stop_robot(self):
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.angular.z = 0.0

        self.cmd_pub.publish(msg)

    def process_key(self, key):

        # MODE SWITCHING
        if key == 'a':
            self.set_manual()
            return

        if key == 'b':
            self.set_autonomous()
            return

        # STOP
        if key == 'x':
            self.stop_robot()
            return

        # Ignore movement commands in autonomous mode.
        if not self.manual_mode:
            return

        # MANUAL MOVEMENT
        if key == 'w':
            self.linear_x = self.speed

        elif key == 's':
            self.linear_x = -self.speed

        elif key == 'd':
            self.linear_y = -self.speed

        elif key == 'f':
            self.linear_y = self.speed

        elif key == 'q':
            self.angular_z = self.turn_speed

        elif key == 'e':
            self.angular_z = -self.turn_speed

        elif key == ' ':
            self.stop_robot()

    def spin_keyboard(self):

        try:
            while rclpy.ok():

                rclpy.spin_once(self, timeout_sec=0.01)

                ready, _, _ = select.select(
                    [sys.stdin],
                    [],
                    [],
                    0.01
                )

                if ready:
                    key = sys.stdin.read(1)

                    if key == '\x03':
                        break

                    self.process_key(key)

        finally:
            self.stop_robot()
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self.settings
            )


def main(args=None):

    rclpy.init(args=args)

    node = MecanumTeleopAB()

    try:
        node.spin_keyboard()

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
