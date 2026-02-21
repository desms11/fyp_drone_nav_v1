#!/usr/bin/env python3
"""
drone_slam/launch/map_saver.launch.py

Convenience launch to save the current map from slam_toolbox.

Usage:
  ros2 launch drone_slam map_saver.launch.py map_name:=my_map save_dir:=/home/user/maps
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="saved_map",
        description="Output map file name (without extension)",
    )
    save_dir_arg = DeclareLaunchArgument(
        "save_dir",
        default_value=os.path.expanduser("~/maps"),
        description="Directory to save the map files",
    )

    save_map = ExecuteProcess(
        cmd=[
            "bash", "-c",
            [
                "mkdir -p ",
                LaunchConfiguration("save_dir"),
                " && ros2 run nav2_map_server map_saver_cli -f ",
                LaunchConfiguration("save_dir"),
                "/",
                LaunchConfiguration("map_name"),
                " --ros-args -p use_sim_time:=true",
            ],
        ],
        output="screen",
    )

    return LaunchDescription([
        map_name_arg,
        save_dir_arg,
        save_map,
    ])
