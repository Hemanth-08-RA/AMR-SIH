from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_share = get_package_share_directory(
        'mecanum_description'
    )

    xacro_file = os.path.join(
        pkg_share,
        'urdf',
        'mecanum_robot.urdf.xacro'
    )

    robot_description = Command([
        'xacro',
        ' ',
        xacro_file
    ])

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description
            }
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description
            }
        ]
    )

    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'mecanum.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=[
            '-d',
            rviz_config
        ]
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        rviz
    ])
