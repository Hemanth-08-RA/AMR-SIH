#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import NavigateToPose


# ============================================================
# XBOX CONTROLLER BUTTONS
# ============================================================

A_BUTTON = 0          # MANUAL
B_BUTTON = 1          # AUTONOMOUS
X_BUTTON = 2          # STOP
Y_BUTTON = 3          # SAVE GOAL
START_BUTTON = 7      # SAVE START


class GoalManager(Node):

    def __init__(self):

        super().__init__('goal_manager')

        # ----------------------------------------------------
        # Saved poses
        # ----------------------------------------------------

        self.start_pose = None
        self.goal_pose = None

        # Current Nav2 goal
        self.current_goal_handle = None

        # Previous joystick button states
        self.last_buttons = []

        # ----------------------------------------------------
        # TF
        # ----------------------------------------------------

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ----------------------------------------------------
        # Nav2 NavigateToPose action
        # ----------------------------------------------------

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        # ----------------------------------------------------
        # Xbox controller
        # ----------------------------------------------------

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # ----------------------------------------------------
        # Startup messages
        # ----------------------------------------------------

        self.get_logger().info(
            '================================================'
        )

        self.get_logger().info(
            'GOAL MANAGER READY'
        )

        self.get_logger().info(
            'A = MANUAL'
        )

        self.get_logger().info(
            'B = AUTONOMOUS'
        )

        self.get_logger().info(
            'X = STOP'
        )

        self.get_logger().info(
            'Y = SAVE GOAL'
        )

        self.get_logger().info(
            'START = SAVE START'
        )

        self.get_logger().info(
            '================================================'
        )

    # ========================================================
    # JOYSTICK CALLBACK
    # ========================================================

    def joy_callback(self, msg):

        buttons = list(msg.buttons)

        # First callback
        if not self.last_buttons:

            self.last_buttons = [
                0
            ] * len(buttons)

        # ----------------------------------------------------
        # Y -> SAVE GOAL
        # ----------------------------------------------------

        if self.button_pressed(
                buttons,
                Y_BUTTON):

            self.save_goal()

        # ----------------------------------------------------
        # START -> SAVE START
        # ----------------------------------------------------

        if self.button_pressed(
                buttons,
                START_BUTTON):

            self.save_start()

        # ----------------------------------------------------
        # B -> AUTONOMOUS
        # ----------------------------------------------------

        if self.button_pressed(
                buttons,
                B_BUTTON):

            self.start_navigation()

        # ----------------------------------------------------
        # X -> STOP
        # ----------------------------------------------------

        if self.button_pressed(
                buttons,
                X_BUTTON):

            self.cancel_navigation()

        # ----------------------------------------------------
        # Remember button state
        # ----------------------------------------------------

        self.last_buttons = buttons

    # ========================================================
    # BUTTON EDGE DETECTION
    # ========================================================

    def button_pressed(
            self,
            buttons,
            button_index):

        if button_index >= len(buttons):

            return False

        previous = 0

        if button_index < len(self.last_buttons):

            previous = self.last_buttons[
                button_index
            ]

        return (
            buttons[button_index] == 1
            and previous == 0
        )

    # ========================================================
    # GET CURRENT ROBOT POSE
    # ========================================================

    def get_current_pose(self):

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    'map',
                    'base_link',
                    rclpy.time.Time()
                )
            )

            pose = PoseStamped()

            pose.header.frame_id = 'map'

            pose.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            pose.pose.position.x = (
                transform.transform.translation.x
            )

            pose.pose.position.y = (
                transform.transform.translation.y
            )

            pose.pose.position.z = 0.0

            pose.pose.orientation = (
                transform.transform.rotation
            )

            return pose

        except Exception as error:

            self.get_logger().warn(
                'Unable to get map -> base_link '
                f'transform: {error}'
            )

            return None

    # ========================================================
    # SAVE START POSITION
    # ========================================================

    def save_start(self):

        pose = self.get_current_pose()

        if pose is None:

            self.get_logger().warn(
                'START NOT SAVED'
            )

            self.get_logger().warn(
                'map -> base_link transform unavailable'
            )

            return

        self.start_pose = pose

        self.get_logger().info(
            '------------------------------------------------'
        )

        self.get_logger().info(
            'START POSITION SAVED'
        )

        self.get_logger().info(
            f'X = {pose.pose.position.x:.3f} m'
        )

        self.get_logger().info(
            f'Y = {pose.pose.position.y:.3f} m'
        )

        self.get_logger().info(
            '------------------------------------------------'
        )

    # ========================================================
    # SAVE GOAL POSITION
    # ========================================================

    def save_goal(self):

        pose = self.get_current_pose()

        if pose is None:

            self.get_logger().warn(
                'GOAL NOT SAVED'
            )

            self.get_logger().warn(
                'map -> base_link transform unavailable'
            )

            return

        self.goal_pose = pose

        self.get_logger().info(
            '------------------------------------------------'
        )

        self.get_logger().info(
            'GOAL POSITION SAVED'
        )

        self.get_logger().info(
            f'X = {pose.pose.position.x:.3f} m'
        )

        self.get_logger().info(
            f'Y = {pose.pose.position.y:.3f} m'
        )

        self.get_logger().info(
            '------------------------------------------------'
        )

    # ========================================================
    # START NAVIGATION
    # ========================================================

    def start_navigation(self):

        # ----------------------------------------------------
        # Check goal
        # ----------------------------------------------------

        if self.goal_pose is None:

            self.get_logger().warn(
                'NO GOAL SAVED'
            )

            self.get_logger().warn(
                'Move robot to desired goal position '
                'and press Y first.'
            )

            return

        # ----------------------------------------------------
        # Check Nav2
        # ----------------------------------------------------

        self.get_logger().info(
            'Waiting for Nav2 NavigateToPose...'
        )

        if not self.nav_client.wait_for_server(
                timeout_sec=3.0):

            self.get_logger().error(
                'Nav2 NavigateToPose action server '
                'is not available.'
            )

            return

        # ----------------------------------------------------
        # Create navigation goal
        # ----------------------------------------------------

        goal = NavigateToPose.Goal()

        goal.pose = self.goal_pose

        self.get_logger().info(
            '================================================'
        )

        self.get_logger().info(
            'AUTONOMOUS NAVIGATION STARTED'
        )

        self.get_logger().info(
            f'Goal X = '
            f'{self.goal_pose.pose.position.x:.3f} m'
        )

        self.get_logger().info(
            f'Goal Y = '
            f'{self.goal_pose.pose.position.y:.3f} m'
        )

        self.get_logger().info(
            '================================================'
        )

        # ----------------------------------------------------
        # Send goal
        # ----------------------------------------------------

        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    # ========================================================
    # NAV2 GOAL RESPONSE
    # ========================================================

    def goal_response_callback(self, future):

        try:

            goal_handle = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Failed to send Nav2 goal: {error}'
            )

            return

        if not goal_handle.accepted:

            self.get_logger().warn(
                'NAVIGATION GOAL REJECTED'
            )

            self.current_goal_handle = None

            return

        # Save handle so X can cancel it
        self.current_goal_handle = goal_handle

        self.get_logger().info(
            'NAVIGATION GOAL ACCEPTED'
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.navigation_result_callback
        )

    # ========================================================
    # NAV2 FEEDBACK
    # ========================================================

    def feedback_callback(
            self,
            feedback_msg):

        feedback = feedback_msg.feedback

        distance = feedback.distance_remaining

        self.get_logger().info(
            f'Distance remaining: '
            f'{distance:.2f} m',
            throttle_duration_sec=2.0
        )

    # ========================================================
    # NAVIGATION RESULT
    # ========================================================

    def navigation_result_callback(
            self,
            future):

        try:

            result = future.result()

            self.get_logger().info(
                '================================================'
            )

            self.get_logger().info(
                f'NAVIGATION FINISHED'
            )

            self.get_logger().info(
                f'Nav2 status: {result.status}'
            )

            self.get_logger().info(
                '================================================'
            )

        except Exception as error:

            self.get_logger().error(
                f'Navigation result error: {error}'
            )

        finally:

            self.current_goal_handle = None

    # ========================================================
    # CANCEL NAVIGATION
    # ========================================================

    def cancel_navigation(self):

        self.get_logger().info(
            'STOP BUTTON PRESSED'
        )

        if self.current_goal_handle is None:

            self.get_logger().info(
                'No active Nav2 goal.'
            )

            return

        self.get_logger().info(
            'Cancelling autonomous navigation...'
        )

        future = (
            self.current_goal_handle
            .cancel_goal_async()
        )

        future.add_done_callback(
            self.cancel_response_callback
        )

    # ========================================================
    # CANCEL RESPONSE
    # ========================================================

    def cancel_response_callback(
            self,
            future):

        try:

            response = future.result()

            if response.goals_canceling:

                self.get_logger().info(
                    'AUTONOMOUS NAVIGATION CANCELLED'
                )

            else:

                self.get_logger().warn(
                    'Nav2 did not cancel the goal.'
                )

        except Exception as error:

            self.get_logger().error(
                f'Cancel error: {error}'
            )

        finally:

            self.current_goal_handle = None


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = GoalManager()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
