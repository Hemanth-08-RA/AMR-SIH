import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ========================================================
    # SAVED MAP
    # ========================================================

    default_map = os.path.expanduser(
        '~/mecanum_ws/src/mecanum_teleop/maps/warehouse.yaml'
    )

    map_file = LaunchConfiguration('map')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path to saved occupancy grid map'
    )

    # ========================================================
    # NAV2
    # ========================================================

    nav2_share = FindPackageShare(
        package='nav2_bringup'
    ).find('nav2_bringup')

    navigation_launch = os.path.join(
        nav2_share,
        'launch',
        'navigation_launch.py'
    )

    # ========================================================
    # NAV2 PARAMETERS
    # ========================================================

    mecanum_share = FindPackageShare(
        package='mecanum_teleop'
    ).find('mecanum_teleop')

    params_file = os.path.join(
        mecanum_share,
        'config',
        'nav2_params.yaml'
    )

    # ========================================================
    # NAV2 NAVIGATION
    # ========================================================

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            navigation_launch
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
        }.items()
    )

    # ========================================================
    # NAV2 CMD_VEL BRIDGE
    #
    # /cmd_vel_nav
    #       ↓
    # nav2_cmd_bridge
    #       ↓
    # /autonomous_cmd_vel
    #       ↓
    # CommandMux
    # ========================================================

    cmd_bridge = Node(
        package='mecanum_teleop',
        executable='nav2_cmd_bridge',
        name='nav2_cmd_bridge',
        output='screen'
    )

    return LaunchDescription([
        declare_map,
        nav2,
        cmd_bridge
    ])
