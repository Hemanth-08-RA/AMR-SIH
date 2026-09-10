#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool


class JoystickAB(Node):

    def __init__(self):
        super().__init__('joystick_ab')

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

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.publish_manual_cmd
        )

        self.manual_mode = True

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        self.max_linear = 0.30
        self.max_strafe = 0.30
        self.max_angular = 0.80

        self.previous_buttons = []

        self.get_logger().info('======================================')
        self.get_logger().info(' MECANUM JOYSTICK A/B READY')
        self.get_logger().info(' A = MANUAL MODE')
        self.get_logger().info(' B = AUTONOMOUS MODE')
        self.get_logger().info(' Left stick Y = Forward / Backward')
        self.get_logger().info(' Left stick X = Strafe')
        self.get_logger().info(' Right stick X = Rotation')
        self.get_logger().info('======================================')

    def joy_callback(self, msg):

        # --------------------------------------------------
        # BUTTONS
        #
        # Standard joystick/Xbox mapping:
        # A = button 0
        # B = button 1
        # X = button 2
        # Y = button 3
        # --------------------------------------------------

        if len(msg.buttons) >= 2:

            # A button -> MANUAL
            if msg.buttons[0] == 1 and (
                len(self.previous_buttons) == 0
                or self.previous_buttons[0] == 0
            ):
                self.enter_manual_mode()

            # B button -> AUTONOMOUS
            if msg.buttons[1] == 1 and (
                len(self.previous_buttons) == 0
                or self.previous_buttons[1] == 0
            ):
                self.enter_autonomous_mode()

        # --------------------------------------------------
        # AXES
        # --------------------------------------------------

        if self.manual_mode:

            # Typical Xbox / Logitech style:
            # axis 1 = left stick vertical
            # axis 0 = left stick horizontal
            # axis 3 = right stick horizontal

            if len(msg.axes) > 1:
                self.linear_x = msg.axes[1] * self.max_linear

            if len(msg.axes) > 0:
                self.linear_y = msg.axes[0] * self.max_strafe

            if len(msg.axes) > 3:
                self.angular_z = msg.axes[3] * self.max_angular

        self.previous_buttons = list(msg.buttons)

    def enter_manual_mode(self):

        if not self.manual_mode:
            self.stop_robot()

        self.manual_mode = True

        msg = Bool()
        msg.data = False
        self.auto_pub.publish(msg)

        self.get_logger().info('A -> MANUAL MODE')

    def enter_autonomous_mode(self):

        # Stop manual command first.
        self.stop_robot()

        self.manual_mode = False

        msg = Bool()
        msg.data = True
        self.auto_pub.publish(msg)

        self.get_logger().info('B -> AUTONOMOUS MODE')

    def publish_manual_cmd(self):

        # CRITICAL:
        #
        # In autonomous mode we DO NOT publish /cmd_vel.
        #
        # autonomous_mode.py gets exclusive control of /cmd_vel.
        #

        if not self.manual_mode:
            return

        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.twist.linear.x = self.linear_x
        msg.twist.linear.y = self.linear_y
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = self.angular_z

        self.cmd_pub.publish(msg)

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


def main(args=None):

    rclpy.init(args=args)

    node = JoystickAB()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
