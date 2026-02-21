#!/usr/bin/env python3
"""
drone_slam/launch/slam.launch.py

Launches slam_toolbox in online-async mapping mode.
After exploration, save the map with:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

Usage:
  ros2 launch drone_slam slam.launch.py
  ros2 launch drone_slam slam.launch.py slam_params_file:=/path/to/custom.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory("drone_slam")
    default_params = os.path.join(pkg, "config", "slam_toolbox.yaml")

    slam_params_arg = DeclareLaunchArgument(
        "slam_params_file",
        default_value=default_params,
        description="Full path to slam_toolbox parameters file",
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[LaunchConfiguration("slam_params_file"), {"use_sim_time": True}],
        remappings=[
            ("/scan", "/scan"),
            ("/map", "/map"),
            ("/odom", "/odom"),
        ],
    )

    return LaunchDescription([
        slam_params_arg,
        slam_node,
    ])
