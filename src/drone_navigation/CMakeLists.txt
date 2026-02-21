cmake_minimum_required(VERSION 3.8)
project(drone_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY
  config
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/set_nav_goal.py
  scripts/navigate_with_algorithm.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
