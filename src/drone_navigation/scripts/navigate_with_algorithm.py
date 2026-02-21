#!/usr/bin/env python3
"""
drone_navigation/scripts/navigate_with_algorithm.py

CLI tool to send a navigation goal with the chosen planning algorithm.

Usage:
  ros2 run drone_navigation navigate_with_algorithm.py \\
      --start 0.0 0.0 \\
      --goal  4.0 3.0 \\
      --algorithm astar

  Algorithms: dijkstra | astar | smac

The algorithm selection is made at Nav2 launch time via the
`algorithm:=` argument to navigation.launch.py, which loads the
corresponding Nav2 parameter file with the right planner plugin.
This script sends the initial pose and navigation goal to a running
Nav2 stack.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


VALID_ALGORITHMS = {"dijkstra", "astar", "smac"}


class NavigationClient(Node):

    def __init__(self, algorithm: str):
        super().__init__("navigation_client")
        self.algorithm = algorithm.lower()
        if self.algorithm not in VALID_ALGORITHMS:
            self.get_logger().error(
                f"Unknown algorithm '{algorithm}'. Choose from: dijkstra, astar, smac"
            )
            sys.exit(1)

        # Action client for Nav2 NavigateToPose
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Publisher for initial pose
        latching_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "initialpose", latching_qos
        )

        self.get_logger().info(f"Navigation client ready. Algorithm: {self.algorithm}")

    # ── Initial pose ────────────────────────────────────────────────────
    def set_initial_pose(self, x: float, y: float, yaw: float = 0.0):
        """Publish the initial pose (Point A) so AMCL can localise."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Covariance: moderately uncertain
        msg.pose.covariance[0]  = 0.25   # x
        msg.pose.covariance[7]  = 0.25   # y
        msg.pose.covariance[35] = 0.068  # yaw
        self._initial_pose_pub.publish(msg)
        self.get_logger().info(f"Initial pose set: ({x:.2f}, {y:.2f})")
        time.sleep(0.5)

    # ── Navigation goal ─────────────────────────────────────────────────
    def navigate_to(self, x: float, y: float, yaw: float = 0.0):
        """Send a NavigateToPose goal (Point B) and wait for completion."""
        self.get_logger().info(
            f"Waiting for Nav2 action server (algorithm={self.algorithm})…"
        )
        self._nav_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.behavior_tree = ""

        self.get_logger().info(f"Sending goal: ({x:.2f}, {y:.2f}) yaw={yaw:.3f}")
        send_goal_future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False

        self.get_logger().info("Goal accepted. Navigating…")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("✔  Goal reached successfully!")
            return True
        else:
            self.get_logger().error(f"✘  Navigation failed with status: {result.status}")
            return False

    def _feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"Distance remaining: {dist:.2f} m", throttle_duration_sec=2)


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Send a navigation goal to the drone using a chosen algorithm."
    )
    parser.add_argument(
        "--start", nargs=2, type=float, metavar=("X", "Y"),
        default=[0.0, 0.0],
        help="Start position (Point A): X Y (default: 0 0)"
    )
    parser.add_argument(
        "--goal", nargs=2, type=float, metavar=("X", "Y"),
        required=True,
        help="Goal position (Point B): X Y"
    )
    parser.add_argument(
        "--algorithm", choices=["dijkstra", "astar", "smac"],
        default="astar",
        help="Path planning algorithm (default: astar)"
    )
    parser.add_argument(
        "--start-yaw", type=float, default=0.0,
        help="Initial yaw in radians (default: 0)"
    )
    parser.add_argument(
        "--goal-yaw", type=float, default=0.0,
        help="Goal yaw in radians (default: 0)"
    )

    # Remove ROS args before argparse
    args = parser.parse_args(sys.argv[1:])

    rclpy.init()
    client = NavigationClient(algorithm=args.algorithm)
    client.set_initial_pose(args.start[0], args.start[1], args.start_yaw)

    success = client.navigate_to(args.goal[0], args.goal[1], args.goal_yaw)
    client.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
