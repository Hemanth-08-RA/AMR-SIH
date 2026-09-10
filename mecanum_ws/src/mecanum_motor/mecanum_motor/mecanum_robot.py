#!/usr/bin/env python3

import time

import lgpio
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


# ============================================================
# GPIO
# ============================================================

FL_IN1 = 17
FL_IN2 = 18

FR_IN1 = 22
FR_IN2 = 23

RL_IN1 = 24
RL_IN2 = 25

RR_IN1 = 5
RR_IN2 = 6

PINS = [
    FL_IN1, FL_IN2,
    FR_IN1, FR_IN2,
    RL_IN1, RL_IN2,
    RR_IN1, RR_IN2
]


# ============================================================
# SETTINGS
# ============================================================

JOY_TIMEOUT = 0.50
CMD_TIMEOUT = 0.50
DEADZONE = 0.20

A_BUTTON = 0
B_BUTTON = 1
X_BUTTON = 2


class MecanumRobot(Node):

    def __init__(self):

        super().__init__('mecanum_robot')

        # ----------------------------------------------------
        # GPIO
        # ----------------------------------------------------

        self.h = lgpio.gpiochip_open(0)

        for pin in PINS:
            lgpio.gpio_claim_output(self.h, pin, 0)

        # ----------------------------------------------------
        # MODE
        # ----------------------------------------------------

        self.mode = 'STOP'

        self.previous_a = 0
        self.previous_b = 0
        self.previous_x = 0

        # ----------------------------------------------------
        # JOYSTICK
        # ----------------------------------------------------

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # ----------------------------------------------------
        # AUTONOMOUS COMMAND
        #
        # IMPORTANT:
        # This is Twist, NOT TwistStamped.
        # ----------------------------------------------------

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # ----------------------------------------------------
        # WATCHDOG
        # ----------------------------------------------------

        self.last_joy_time = time.monotonic()
        self.last_cmd_time = time.monotonic()

        self.watchdog_timer = self.create_timer(
            0.05,
            self.watchdog
        )

        self.current_wheels = (0, 0, 0, 0)

        self.stop_all()

        self.get_logger().info('==========================================')
        self.get_logger().info(' MECANUM ROBOT GPIO CONTROLLER')
        self.get_logger().info('==========================================')
        self.get_logger().info('A = MANUAL TELEOP')
        self.get_logger().info('B = AUTONOMOUS')
        self.get_logger().info('X = STOP')
        self.get_logger().info('GPIO FL = 17,18')
        self.get_logger().info('GPIO FR = 22,23')
        self.get_logger().info('GPIO RL = 24,25')
        self.get_logger().info('GPIO RR = 5,6')
        self.get_logger().info('Waiting for joystick...')
        self.get_logger().info('==========================================')

    # ========================================================
    # MOTOR
    # ========================================================

    def motor(self, in1, in2, direction):

        if direction > 0:

            lgpio.gpio_write(self.h, in1, 1)
            lgpio.gpio_write(self.h, in2, 0)

        elif direction < 0:

            lgpio.gpio_write(self.h, in1, 0)
            lgpio.gpio_write(self.h, in2, 1)

        else:

            lgpio.gpio_write(self.h, in1, 0)
            lgpio.gpio_write(self.h, in2, 0)

    # ========================================================
    # STOP
    # ========================================================

    def stop_all(self):

        self.motor(FL_IN1, FL_IN2, 0)
        self.motor(FR_IN1, FR_IN2, 0)
        self.motor(RL_IN1, RL_IN2, 0)
        self.motor(RR_IN1, RR_IN2, 0)

        self.current_wheels = (0, 0, 0, 0)

    # ========================================================
    # WHEELS
    # ========================================================

    def set_wheels(self, fl, fr, rl, rr):

        self.motor(FL_IN1, FL_IN2, fl)
        self.motor(FR_IN1, FR_IN2, fr)
        self.motor(RL_IN1, RL_IN2, rl)
        self.motor(RR_IN1, RR_IN2, rr)

        self.current_wheels = (fl, fr, rl, rr)

    # ========================================================
    # MODE
    # ========================================================

    def set_mode(self, mode):

        if self.mode == mode:
            return

        self.stop_all()

        self.mode = mode

        if mode == 'MANUAL':

            self.get_logger().info(
                'A -> MANUAL TELEOPERATION'
            )

        elif mode == 'AUTONOMOUS':

            self.get_logger().info(
                'B -> AUTONOMOUS MODE'
            )

            self.get_logger().info(
                'Waiting for /cmd_vel'
            )

        elif mode == 'STOP':

            self.get_logger().info(
                'X -> STOP'
            )

    # ========================================================
    # JOYSTICK
    # ========================================================

    def joy_callback(self, msg):

        self.last_joy_time = time.monotonic()

        if len(msg.buttons) > A_BUTTON:

            a = msg.buttons[A_BUTTON]
            b = msg.buttons[B_BUTTON] if len(msg.buttons) > B_BUTTON else 0
            x = msg.buttons[X_BUTTON] if len(msg.buttons) > X_BUTTON else 0

            if a == 1 and self.previous_a == 0:
                self.set_mode('MANUAL')

            if b == 1 and self.previous_b == 0:
                self.set_mode('AUTONOMOUS')

            if x == 1 and self.previous_x == 0:
                self.set_mode('STOP')

            self.previous_a = a
            self.previous_b = b
            self.previous_x = x

        # ----------------------------------------------------
        # ONLY MANUAL MODE USES STICK
        # ----------------------------------------------------

        if self.mode != 'MANUAL':
            return

        if len(msg.axes) < 4:
            self.stop_all()
            return

        x = msg.axes[0]
        y = -msg.axes[1]
        rotation = msg.axes[3]

        if abs(x) < DEADZONE:
            x = 0.0

        if abs(y) < DEADZONE:
            y = 0.0

        if abs(rotation) < DEADZONE:
            rotation = 0.0

        if x == 0.0 and y == 0.0 and rotation == 0.0:
            self.stop_all()
            return

        # Mecanum mixing
        fl = y + x + rotation
        fr = y - x - rotation
        rl = y - x + rotation
        rr = y + x - rotation

        def direction(value):

            if value > 0.20:
                return 1

            if value < -0.20:
                return -1

            return 0

        self.set_wheels(
            direction(fl),
            direction(fr),
            direction(rl),
            direction(rr)
        )

    # ========================================================
    # AUTONOMOUS /cmd_vel
    # ========================================================

    def cmd_vel_callback(self, msg):

        self.last_cmd_time = time.monotonic()

        if self.mode != 'AUTONOMOUS':
            return

        x = msg.linear.x
        y = msg.linear.y
        rotation = msg.angular.z

        if abs(x) < 0.05:
            x = 0.0

        if abs(y) < 0.05:
            y = 0.0

        if abs(rotation) < 0.05:
            rotation = 0.0

        # ----------------------------------------------------
        # MECANUM MIXING
        # ----------------------------------------------------

        fl = x + y + rotation
        fr = x - y - rotation
        rl = x - y + rotation
        rr = x + y - rotation

        maximum = max(
            1.0,
            abs(fl),
            abs(fr),
            abs(rl),
            abs(rr)
        )

        fl /= maximum
        fr /= maximum
        rl /= maximum
        rr /= maximum

        def direction(value):

            if value > 0.20:
                return 1

            if value < -0.20:
                return -1

            return 0

        self.set_wheels(
            direction(fl),
            direction(fr),
            direction(rl),
            direction(rr)
        )

    # ========================================================
    # WATCHDOG
    # ========================================================

    def watchdog(self):

        now = time.monotonic()

        if self.mode == 'MANUAL':

            if now - self.last_joy_time > JOY_TIMEOUT:
                self.stop_all()

        elif self.mode == 'AUTONOMOUS':

            if now - self.last_cmd_time > CMD_TIMEOUT:
                self.stop_all()

    # ========================================================
    # CLEANUP
    # ========================================================

    def destroy_node(self):

        try:
            self.stop_all()

            for pin in PINS:
                try:
                    lgpio.gpio_free(self.h, pin)
                except Exception:
                    pass

            try:
                lgpio.gpiochip_close(self.h)
            except Exception:
                pass

        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = MecanumRobot()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
