from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'mecanum_teleop'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*.yaml')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'maps'
            ),
            glob('maps/*')
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='mobile',

    maintainer_email='mobile@localhost',

    description=(
        'Mecanum robot teleoperation, mapping '
        'and autonomous navigation'
    ),

    license='Apache License 2.0',

    tests_require=[
        'pytest'
    ],

    entry_points={
        'console_scripts': [

            # Existing working nodes
            'mode_manager = mecanum_teleop.mecanum_control:main_mode_manager',

            'manual_teleop = mecanum_teleop.mecanum_control:main_manual',

            'autonomous_navigator = mecanum_teleop.mecanum_control:main_autonomous',

            'command_mux = mecanum_teleop.mecanum_control:main_mux',

            # New Phase 2 node
            'goal_manager = mecanum_teleop.goal_manager:main',

            'nav2_cmd_bridge = mecanum_teleop.nav2_cmd_bridge:main',
            'open_loop_odom = mecanum_teleop.open_loop_odom:main',
        ],
    },
)
