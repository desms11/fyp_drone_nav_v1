cmake_minimum_required(VERSION 3.8)
project(drone_teleop)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY
  scripts
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
