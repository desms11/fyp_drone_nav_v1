#!/usr/bin/env python3
"""
drone_bringup/launch/slam_bringup.launch.py

Phase 1: Spawn drone + run SLAM for map exploration.

Usage:
  ros2 launch drone_bringup slam_bringup.launch.py world:=simple_room
  ros2 launch drone_bringup slam_bringup.launch.py world:=maze
  ros2 launch drone_bringup slam_bringup.launch.py world:=office

After exploring, save the map:
  ros2 launch drone_slam map_saver.launch.py map_name:=my_map

Then switch to navigation mode:
  ros2 launch drone_bringup nav_bringup.launch.py map:=~/maps/my_map.yaml algorithm:=astar
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    pkg_gazebo = get_package_share_directory("drone_gazebo")
    pkg_slam   = get_package_share_directory("drone_slam")

    # ── Arguments ────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="simple_room",
        description="Gazebo world to load: simple_room | maze | office",
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="Spawn X")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Spawn Y")

    # ── Gazebo + drone spawn ─────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world":    LaunchConfiguration("world"),
            "x":        LaunchConfiguration("x"),
            "y":        LaunchConfiguration("y"),
            "use_rviz": "false",  # SLAM brings its own RViz
        }.items(),
    )

    # ── SLAM ─────────────────────────────────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, "launch", "slam.launch.py")
        ),
    )

    return LaunchDescription([
        world_arg,
        x_arg,
        y_arg,
        LogInfo(msg="═══ Phase 1: SLAM Exploration ═══"),
        LogInfo(msg="Use keyboard teleop to explore the map:"),
        LogInfo(msg="  ros2 run drone_teleop teleop_keyboard.py"),
        LogInfo(msg="Save map when done:"),
        LogInfo(msg="  ros2 launch drone_slam map_saver.launch.py map_name:=my_map"),
        gazebo_launch,
        TimerAction(period=6.0, actions=[slam_launch]),
    ])
