cmake_minimum_required(VERSION 3.8)
project(drone_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/maps")
  install(DIRECTORY
    maps
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()
