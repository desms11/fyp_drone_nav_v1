#!/usr/bin/env python3
"""
Launch file to display the drone model in RViz.
Used for URDF visualisation and debugging.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("drone_description")
    urdf_file = PathJoinSubstitution([pkg, "urdf", "drone.urdf.xacro"])

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    robot_description = Command(["xacro ", urdf_file])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description, "use_sim_time": use_sim_time}
            ],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([pkg, "rviz", "drone.rviz"])],
        ),
    ])
