#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class AutonomousMode(Node):

    def __init__(self):
        super().__init__('autonomous_mode')

        # ==============================================
        # JOYSTICK
        # ==============================================

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # ==============================================
        # CMD_VEL
        # ==============================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ==============================================
        # CONTROL LOOP
        # ==============================================

        self.timer = self.create_timer(
            0.05,       # 20 Hz
            self.control_loop
        )

        # ==============================================
        # STATE
        # ==============================================

        self.mode = 'STOP'

        self.auto_speed = 0.15

        self.manual_x = 0.0
        self.manual_y = 0.0
        self.manual_w = 0.0

        self.prev_a = 0
        self.prev_b = 0
        self.prev_x = 0

        self.get_logger().info(
            '=============================================='
        )
        self.get_logger().info(
            ' AUTONOMOUS COMMAND NODE'
        )
        self.get_logger().info(
            ' A = MANUAL'
        )
        self.get_logger().info(
            ' B = AUTONOMOUS'
        )
        self.get_logger().info(
            ' X = STOP'
        )
        self.get_logger().info(
            ' Publishing geometry_msgs/msg/Twist'
        )
        self.get_logger().info(
            '=============================================='
        )

    # ==============================================
    # JOYSTICK CALLBACK
    # ==============================================

    def joy_callback(self, msg):

        a = msg.buttons[0] if len(msg.buttons) > 0 else 0
        b = msg.buttons[1] if len(msg.buttons) > 1 else 0
        x = msg.buttons[2] if len(msg.buttons) > 2 else 0

        # ------------------------------------------
        # A = MANUAL
        # ------------------------------------------

        if a == 1 and self.prev_a == 0:

            self.mode = 'MANUAL'

            self.get_logger().info(
                'A PRESSED -> MANUAL'
            )

        # ------------------------------------------
        # B = AUTONOMOUS
        # ------------------------------------------

        if b == 1 and self.prev_b == 0:

            self.mode = 'AUTONOMOUS'

            self.get_logger().info(
                'B PRESSED -> AUTONOMOUS'
            )

        # ------------------------------------------
        # X = STOP
        # ------------------------------------------

        if x == 1 and self.prev_x == 0:

            self.mode = 'STOP'

            self.manual_x = 0.0
            self.manual_y = 0.0
            self.manual_w = 0.0

            self.get_logger().warn(
                'X PRESSED -> STOP'
            )

        # ------------------------------------------
        # MANUAL JOYSTICK
        # ------------------------------------------

        if self.mode == 'MANUAL':

            lx = msg.axes[0] if len(msg.axes) > 0 else 0.0
            ly = msg.axes[1] if len(msg.axes) > 1 else 0.0
            rx = msg.axes[3] if len(msg.axes) > 3 else 0.0

            self.manual_x = -ly * 0.30
            self.manual_y = lx * 0.30
            self.manual_w = rx * 1.0

        self.prev_a = a
        self.prev_b = b
        self.prev_x = x

    # ==============================================
    # CONTROL LOOP
    # ==============================================

    def control_loop(self):

        msg = Twist()

        # ------------------------------------------
        # AUTONOMOUS
        # ------------------------------------------

        if self.mode == 'AUTONOMOUS':

            msg.linear.x = self.auto_speed
            msg.linear.y = 0.0
            msg.linear.z = 0.0

            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

        # ------------------------------------------
        # MANUAL
        # ------------------------------------------

        elif self.mode == 'MANUAL':

            msg.linear.x = self.manual_x
            msg.linear.y = self.manual_y
            msg.linear.z = 0.0

            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = self.manual_w

        # ------------------------------------------
        # STOP
        # ------------------------------------------

        else:

            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0

            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0

        self.cmd_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = AutonomousMode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        try:
            if rclpy.ok():
                stop = Twist()

                for _ in range(5):
                    node.cmd_pub.publish(stop)

        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
