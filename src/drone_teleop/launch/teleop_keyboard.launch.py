#!/usr/bin/env python3
"""Launch the keyboard teleop node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="Topic to publish velocity commands",
        ),
        DeclareLaunchArgument(
            "max_linear_vel",
            default_value="0.5",
            description="Maximum linear velocity (m/s)",
        ),
        DeclareLaunchArgument(
            "max_angular_vel",
            default_value="1.5",
            description="Maximum angular velocity (rad/s)",
        ),
        Node(
            package="drone_teleop",
            executable="teleop_keyboard.py",
            name="teleop_keyboard",
            output="screen",
            prefix="xterm -e",
            parameters=[{
                "cmd_vel_topic":  LaunchConfiguration("cmd_vel_topic"),
                "max_linear_vel": LaunchConfiguration("max_linear_vel"),
                "max_angular_vel": LaunchConfiguration("max_angular_vel"),
            }],
        ),
    ])
