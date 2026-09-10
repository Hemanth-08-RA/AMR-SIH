import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # -----------------------------
    # YDLIDAR X2
    # -----------------------------
    ydlidar_share = FindPackageShare(
        package='ydlidar_ros2_driver'
    ).find('ydlidar_ros2_driver')

    ydlidar_launch = os.path.join(
        ydlidar_share,
        'launch',
        'ydlidar_launch.py'
    )

    ydlidar_config = os.path.join(
        ydlidar_share,
        'params',
        'X2.yaml'
    )

    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch),
        launch_arguments={
            'params_file': ydlidar_config
        }.items()
    )

    # -----------------------------
    # OPEN LOOP ODOMETRY
    # -----------------------------
    open_loop_odom = Node(
        package='mecanum_teleop',
        executable='open_loop_odom',
        name='open_loop_odom',
        output='screen'
    )

    # -----------------------------
    # SLAM TOOLBOX
    # -----------------------------
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {
                'use_sim_time': False,

                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',

                'scan_topic': '/scan',

                'mode': 'mapping',

                'resolution': 0.05,
                'max_laser_range': 12.0,

                'minimum_time_interval': 0.2,
                'transform_publish_period': 0.02,

                'map_update_interval': 2.0,

                'minimum_travel_distance': 0.05,
                'minimum_travel_heading': 0.05,

                'use_scan_matching': True,
                'use_scan_barycenter': True,

                'debug_logging': False,
            }
        ]
    )

    return LaunchDescription([
        ydlidar,
        open_loop_odom,
        slam_toolbox,
    ])
