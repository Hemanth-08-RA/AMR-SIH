from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # ====================================================
        # XBOX CONTROLLER
        # ====================================================

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # ====================================================
        # MODE MANAGER
        # A = MANUAL
        # B = AUTONOMOUS
        # X = STOP
        # ====================================================

        Node(
            package='mecanum_teleop',
            executable='mode_manager',
            name='mode_manager',
            output='screen'
        ),

        # ====================================================
        # MANUAL TELEOPERATION
        # ====================================================

        Node(
            package='mecanum_teleop',
            executable='manual_teleop',
            name='manual_teleop',
            output='screen'
        ),

        # ====================================================
        # GOAL MANAGER
        # Y     = SAVE GOAL
        # START = SAVE START
        # B     = START AUTONOMOUS NAVIGATION
        # X     = CANCEL NAVIGATION
        # ====================================================

        Node(
            package='mecanum_teleop',
            executable='goal_manager',
            name='goal_manager',
            output='screen'
        ),

        # ====================================================
        # COMMAND MUX
        #
        # MANUAL / AUTONOMOUS
        #          ↓
        #       /cmd_vel
        # ====================================================

        Node(
             package='mecanum_teleop',
             executable='command_mux',
             name='command_mux',
             output='screen',

             remappings=[
                         ('/autonomous_cmd_vel', '/cmd_vel_nav')
         ]
        ),

        # ====================================================
        # PHYSICAL MECANUM MOTOR DRIVER
        #
        # /cmd_vel
        #     ↓
        # GPIO
        #     ↓
        # L298N
        #     ↓
        # 4 MOTORS
        # ====================================================

        Node(
            package='mecanum_motor',
            executable='mecanum_robot',
            name='mecanum_robot',
            output='screen'
        ),
    ])
