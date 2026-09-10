#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OpenLoopOdom(Node):

    def __init__(self):
        super().__init__("open_loop_odom")

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.last_time = self.get_clock().now()

        self.cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            "/odom",
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(
            0.02,
            self.update
        )

        self.get_logger().info("OPEN LOOP ODOMETRY READY")
        self.get_logger().info("/cmd_vel -> /odom")
        self.get_logger().info("odom -> base_link")

    def cmd_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

    def update(self):
        now = self.get_clock().now()

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 0.5:
            return

        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)

        world_vx = self.vx * cos_theta - self.vy * sin_theta
        world_vy = self.vx * sin_theta + self.vy * cos_theta

        self.x += world_vx * dt
        self.y += world_vy * dt
        self.theta += self.wz * dt

        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        stamp = now.to_msg()

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OpenLoopOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
