#!/usr/bin/env python3
"""
drone_gazebo/launch/gazebo.launch.py

Launches Gazebo Harmonic with the chosen world and spawns the drone.

Usage (from workspace root after build):
  ros2 launch drone_gazebo gazebo.launch.py world:=simple_room
  ros2 launch drone_gazebo gazebo.launch.py world:=maze
  ros2 launch drone_gazebo gazebo.launch.py world:=office
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Package shares ──────────────────────────────────────────────────
    pkg_drone_gazebo      = get_package_share_directory("drone_gazebo")
    pkg_drone_description = get_package_share_directory("drone_description")
    pkg_ros_gz_sim        = get_package_share_directory("ros_gz_sim")

    # ── Arguments ───────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="simple_room",
        description="World to load: simple_room | maze | office",
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0",  description="Spawn X")
    y_arg = DeclareLaunchArgument("y", default_value="0.0",  description="Spawn Y")
    z_arg = DeclareLaunchArgument("z", default_value="0.15", description="Spawn Z")
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Launch RViz2"
    )

    # ── URDF from xacro ─────────────────────────────────────────────────
    urdf_file = os.path.join(pkg_drone_description, "urdf", "drone.urdf.xacro")
    robot_description = Command(["xacro ", urdf_file])

    # ── Gazebo Harmonic ─────────────────────────────────────────────────
    world_sdf = PathJoinSubstitution(
        [pkg_drone_gazebo, "worlds", PythonExpression(["'", LaunchConfiguration("world"), "' + '.sdf'"])]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r -v4 ", world_sdf],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ── Robot State Publisher ────────────────────────────────────────────
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True}
        ],
    )

    # ── Spawn drone into Gazebo ─────────────────────────────────────────
    spawn_drone = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_drone",
        arguments=[
            "-name",  "drone",
            "-topic", "robot_description",
            "-x",     LaunchConfiguration("x"),
            "-y",     LaunchConfiguration("y"),
            "-z",     LaunchConfiguration("z"),
        ],
        output="screen",
    )

    # ── ROS–Gazebo topic bridge ──────────────────────────────────────────
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "config_file": os.path.join(
                    get_package_share_directory("drone_bringup"),
                    "config",
                    "ros_gz_bridge.yaml",
                ),
            }
        ],
    )

    # ── RViz2 ────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_drone_description, "rviz", "drone.rviz")
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
        world_arg,
        x_arg, y_arg, z_arg,
        use_rviz_arg,
        gazebo,
        rsp,
        # Small delay so Gazebo is ready before spawning
        TimerAction(period=3.0, actions=[spawn_drone]),
        TimerAction(period=4.0, actions=[bridge]),
        TimerAction(period=5.0, actions=[rviz]),
    ])
