cmake_minimum_required(VERSION 3.8)
project(mecanum_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    urdf
    launch
    rviz
    config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/autonomous_mode.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
