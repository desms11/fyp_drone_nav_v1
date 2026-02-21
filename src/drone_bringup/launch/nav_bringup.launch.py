#!/usr/bin/env python3
"""
drone_bringup/launch/nav_bringup.launch.py

Phase 2: Spawn drone + run Nav2 navigation with a pre-built map.

Prerequisite: complete Phase 1 (SLAM) and save the map.

Usage:
  ros2 launch drone_bringup nav_bringup.launch.py \\
      map:=/home/<user>/maps/my_map.yaml \\
      algorithm:=astar   \\
      world:=simple_room

  Available algorithms: dijkstra | astar | smac

Then set navigation goals:
  ros2 run drone_navigation set_nav_goal.py
  # or via CLI:
  ros2 run drone_navigation navigate_with_algorithm.py \\
      --start 0 0 --goal 4 3 --algorithm smac
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
    pkg_nav    = get_package_share_directory("drone_navigation")
    pkg_nav2   = get_package_share_directory("nav2_bringup")

    # ── Arguments ────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="simple_room",
        description="Gazebo world: simple_room | maze | office",
    )
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Path to map YAML file (saved from SLAM phase)",
    )
    algorithm_arg = DeclareLaunchArgument(
        "algorithm",
        default_value="astar",
        description="Path planning algorithm: dijkstra | astar | smac",
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")

    # ── Gazebo + drone spawn ─────────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world":    LaunchConfiguration("world"),
            "x":        LaunchConfiguration("x"),
            "y":        LaunchConfiguration("y"),
            "use_rviz": "false",
        }.items(),
    )

    # ── Nav2 ──────────────────────────────────────────────────────────────
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "map":       LaunchConfiguration("map"),
            "algorithm": LaunchConfiguration("algorithm"),
            "use_rviz":  "true",
        }.items(),
    )

    return LaunchDescription([
        world_arg,
        map_arg,
        algorithm_arg,
        x_arg,
        y_arg,
        LogInfo(msg="═══ Phase 2: Autonomous Navigation ═══"),
        gazebo_launch,
        TimerAction(period=6.0, actions=[nav_launch]),
    ])
