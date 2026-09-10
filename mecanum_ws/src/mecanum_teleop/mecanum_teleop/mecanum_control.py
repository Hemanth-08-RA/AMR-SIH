#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, LaserScan
from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import String

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


# ============================================================
# MODE MANAGER
#
# A = MANUAL
# B = AUTONOMOUS
# X = STOP
# ============================================================

class ModeManager(Node):

    def __init__(self):
        super().__init__('mode_manager')

        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.pub = self.create_publisher(
            String,
            '/control_mode',
            10
        )

        self.mode = 'STOP'

        self.prev_a = 0
        self.prev_b = 0
        self.prev_x = 0

        self.publish_mode()

        self.get_logger().info('======================================')
        self.get_logger().info('MODE MANAGER READY')
        self.get_logger().info('A = MANUAL')
        self.get_logger().info('B = AUTONOMOUS')
        self.get_logger().info('X = STOP')
        self.get_logger().info('======================================')

    def publish_mode(self):
        msg = String()
        msg.data = self.mode
        self.pub.publish(msg)

    def joy_callback(self, msg):

        a = msg.buttons[0] if len(msg.buttons) > 0 else 0
        b = msg.buttons[1] if len(msg.buttons) > 1 else 0
        x = msg.buttons[2] if len(msg.buttons) > 2 else 0

        if a == 1 and self.prev_a == 0:
            self.mode = 'MANUAL'
            self.publish_mode()
            self.get_logger().info('MODE -> MANUAL')

        if b == 1 and self.prev_b == 0:
            self.mode = 'AUTONOMOUS'
            self.publish_mode()
            self.get_logger().info('MODE -> AUTONOMOUS')

        if x == 1 and self.prev_x == 0:
            self.mode = 'STOP'
            self.publish_mode()
            self.get_logger().info('MODE -> STOP')

        self.prev_a = a
        self.prev_b = b
        self.prev_x = x


# ============================================================
# MANUAL TELEOPERATION
# ============================================================

class ManualTeleop(Node):

    def __init__(self):
        super().__init__('manual_teleop')

        self.mode = 'STOP'

        self.manual_x = 0.0
        self.manual_y = 0.0
        self.manual_w = 0.0

        self.mode_sub = self.create_subscription(
            String,
            '/control_mode',
            self.mode_callback,
            10
        )

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.pub = self.create_publisher(
            TwistStamped,
            '/manual_cmd_vel',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info('MANUAL TELEOP READY')

    def mode_callback(self, msg):
        self.mode = msg.data

    def joy_callback(self, msg):

        if self.mode != 'MANUAL':
            self.manual_x = 0.0
            self.manual_y = 0.0
            self.manual_w = 0.0
            return

        lx = msg.axes[0] if len(msg.axes) > 0 else 0.0
        ly = msg.axes[1] if len(msg.axes) > 1 else 0.0
        rx = msg.axes[3] if len(msg.axes) > 3 else 0.0

        self.manual_x = -ly * 0.30
        self.manual_y = lx * 0.30
        self.manual_w = rx * 1.0

    def control_loop(self):

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self.mode == 'MANUAL':
            msg.twist.linear.x = self.manual_x
            msg.twist.linear.y = self.manual_y
            msg.twist.angular.z = self.manual_w

        self.pub.publish(msg)


# ============================================================
# AUTONOMOUS NAVIGATOR
#
# IMPORTANT:
# /scan is BEST_EFFORT on the YDLIDAR publisher.
# Therefore this subscriber MUST use BEST_EFFORT.
# ============================================================

class AutonomousNavigator(Node):

    def __init__(self):
        super().__init__('autonomous_navigator')

        self.mode = 'STOP'

        self.scan_received = False
        self.front_distance = float('inf')

        self.mode_sub = self.create_subscription(
            String,
            '/control_mode',
            self.mode_callback,
            10
        )

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos
        )

        self.pub = self.create_publisher(
            TwistStamped,
            '/autonomous_cmd_vel',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'AUTONOMOUS NAVIGATOR READY - WAITING FOR /scan'
        )

    def mode_callback(self, msg):
        self.mode = msg.data

        if self.mode == 'AUTONOMOUS':
            self.get_logger().info(
                'AUTONOMOUS ACTIVE'
            )

    def scan_callback(self, msg):

        self.scan_received = True

        ranges = list(msg.ranges)

        if not ranges:
            self.front_distance = float('inf')
            return

        # Front sector around 0 radians.
        count = len(ranges)

        center = count // 2

        sector_size = max(1, count // 12)

        front_ranges = (
            ranges[0:sector_size] +
            ranges[count - sector_size:count]
        )

        valid = [
            r for r in front_ranges
            if msg.range_min < r < msg.range_max
        ]

        if valid:
            self.front_distance = min(valid)
        else:
            self.front_distance = float('inf')

    def control_loop(self):

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self.mode == 'AUTONOMOUS':

            # Basic autonomous forward motion.
            #
            # If an obstacle is detected in front,
            # stop rather than drive into it.

            if self.front_distance < 0.60:

                msg.twist.linear.x = 0.0
                msg.twist.linear.y = 0.0
                msg.twist.angular.z = 0.0

            else:

                msg.twist.linear.x = 0.15
                msg.twist.linear.y = 0.0
                msg.twist.angular.z = 0.0

        self.pub.publish(msg)


# ============================================================
# COMMAND MUX
#
# This is the ONLY node that owns /cmd_vel.
#
# MANUAL       -> /manual_cmd_vel
# AUTONOMOUS   -> /autonomous_cmd_vel
# STOP         -> zero
# ============================================================

class CommandMux(Node):

    def __init__(self):
        super().__init__('command_mux')

        self.mode = 'STOP'

        self.manual_cmd = TwistStamped()
        self.auto_cmd = TwistStamped()

        self.mode_sub = self.create_subscription(
            String,
            '/control_mode',
            self.mode_callback,
            10
        )

        self.manual_sub = self.create_subscription(
            TwistStamped,
            '/manual_cmd_vel',
            self.manual_callback,
            10
        )

        self.auto_sub = self.create_subscription(
            TwistStamped,
            '/autonomous_cmd_vel',
            self.auto_callback,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'COMMAND MUX READY - /cmd_vel OWNER'
        )

    def mode_callback(self, msg):
        self.mode = msg.data

    def manual_callback(self, msg):
        self.manual_cmd = msg

    def auto_callback(self, msg):
        self.auto_cmd = msg

    def control_loop(self):

        cmd = Twist()

        if self.mode == 'MANUAL':

            cmd.linear.x = self.manual_cmd.twist.linear.x
            cmd.linear.y = self.manual_cmd.twist.linear.y
            cmd.angular.z = self.manual_cmd.twist.angular.z

        elif self.mode == 'AUTONOMOUS':

            cmd.linear.x = self.auto_cmd.twist.linear.x
            cmd.linear.y = self.auto_cmd.twist.linear.y
            cmd.angular.z = self.auto_cmd.twist.angular.z

        else:

            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

        self.pub.publish(cmd)


# ============================================================
# MAIN FUNCTIONS
# ============================================================

def main_mode_manager(args=None):

    rclpy.init(args=args)

    node = ModeManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_manual(args=None):

    rclpy.init(args=args)

    node = ManualTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_autonomous(args=None):

    rclpy.init(args=args)

    node = AutonomousNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_mux(args=None):

    rclpy.init(args=args)

    node = CommandMux()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    rclpy.init()
