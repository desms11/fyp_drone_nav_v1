#!/usr/bin/env python3
"""
drone_navigation/launch/navigation.launch.py

Launches the Nav2 navigation stack with the chosen planning algorithm.

Usage:
  # Default (A*)
  ros2 launch drone_navigation navigation.launch.py map:=/home/user/maps/my_map.yaml

  # Choose algorithm
  ros2 launch drone_navigation navigation.launch.py \
      map:=/home/user/maps/my_map.yaml \
      algorithm:=dijkstra

  ros2 launch drone_navigation navigation.launch.py \
      map:=/home/user/maps/my_map.yaml \
      algorithm:=smac

Available algorithms: dijkstra | astar | smac
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    pkg_nav  = get_package_share_directory("drone_navigation")
    pkg_desc = get_package_share_directory("drone_description")
    pkg_nav2 = get_package_share_directory("nav2_bringup")

    # Arguments
    algorithm_arg = DeclareLaunchArgument(
        "algorithm",
        default_value="astar",
        description="Planning algorithm: dijkstra | astar | smac",
    )
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to the map YAML file",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to start RViz2",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Auto-start Nav2 lifecycle nodes",
    )

    algorithm = LaunchConfiguration("algorithm")
    map_yaml  = LaunchConfiguration("map")

    # Explicit param file paths
    params_dijkstra = os.path.join(pkg_nav, "config", "nav2_dijkstra_params.yaml")
    params_astar    = os.path.join(pkg_nav, "config", "nav2_astar_params.yaml")
    params_smac     = os.path.join(pkg_nav, "config", "nav2_smac_params.yaml")

    # Nav2 instances, one per algorithm (gated by IfCondition)
    nav2_dijkstra = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map":          map_yaml,
            "use_sim_time": "true",
            "params_file":  params_dijkstra,
            "autostart":    LaunchConfiguration("autostart"),
            "use_rviz":     "false",
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", algorithm, "' == 'dijkstra'"])
        ),
    )

    nav2_astar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map":          map_yaml,
            "use_sim_time": "true",
            "params_file":  params_astar,
            "autostart":    LaunchConfiguration("autostart"),
            "use_rviz":     "false",
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", algorithm, "' == 'astar'"])
        ),
    )

    nav2_smac = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "map":          map_yaml,
            "use_sim_time": "true",
            "params_file":  params_smac,
            "autostart":    LaunchConfiguration("autostart"),
            "use_rviz":     "false",
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", algorithm, "' == 'smac'"])
        ),
    )

    # RViz2 with navigation config
    nav_rviz_config = os.path.join(pkg_desc, "rviz", "navigation.rviz")
    rviz_config = nav_rviz_config if os.path.exists(nav_rviz_config) \
        else os.path.join(pkg_desc, "rviz", "drone.rviz")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        algorithm_arg,
        map_arg,
        use_rviz_arg,
        autostart_arg,
        nav2_dijkstra,
        nav2_astar,
        nav2_smac,
        rviz,
    ])
