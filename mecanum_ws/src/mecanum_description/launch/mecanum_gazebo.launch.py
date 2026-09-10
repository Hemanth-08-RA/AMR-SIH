from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # =========================================================
    # ROBOT DESCRIPTION
    # =========================================================

    robot_description_content = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('mecanum_description'),
                    'urdf',
                    'mecanum_robot.urdf.xacro'
                ]
            )
        ]
    )

    robot_description = {
        'robot_description': robot_description_content,
        'use_sim_time': True
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('mecanum_description'),
            'config',
            'mecanum_drive_controller.yaml'
        ]
    )

    # =========================================================
    # GAZEBO FORTRESS
    # =========================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('ign_ros2_control_demos'),
                    'launch',
                    'ign_gazebo.launch.py'
                ]
            )
        )
    )

    # =========================================================
    # ROBOT STATE PUBLISHER
    # =========================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # =========================================================
    # SPAWN ROBOT
    # =========================================================

    spawn_robot = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-topic',
            'robot_description',
            '-name',
            'mecanum_robot',
            '-allow_renaming',
            'true',
            '-x',
            '0',
            '-y',
            '0',
            '-z',
            '0.08'
        ]
    )

    # =========================================================
    # JOINT STATE BROADCASTER
    # =========================================================

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    # =========================================================
    # MECANUM CONTROLLER
    # =========================================================

    mecanum_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mecanum_drive_controller',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            robot_controllers
        ],
        output='screen'
    )

    # =========================================================
    # START CONTROLLERS AFTER ROBOT SPAWN
    # =========================================================

    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster]
        )
    )

    start_mecanum_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[mecanum_controller]
        )
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            spawn_robot,
            start_joint_state_broadcaster,
            start_mecanum_controller
        ]
    )
