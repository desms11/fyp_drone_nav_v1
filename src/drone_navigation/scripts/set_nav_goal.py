#!/usr/bin/env python3
"""
drone_navigation/scripts/set_nav_goal.py

Interactive script to set Point A (start) and Point B (goal) for navigation.

Usage:
  ros2 run drone_navigation set_nav_goal.py

The script prompts for:
  - Start position (x, y) – Point A
  - Goal  position (x, y) – Point B
  - Algorithm choice (1=Dijkstra, 2=A*, 3=Smac Hybrid-A*)

Then it launches the appropriate navigation configuration and sends the goal.
"""

import math
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


ALGORITHMS = {
    "1": ("Dijkstra",        "dijkstra"),
    "2": ("A*",              "astar"),
    "3": ("Smac Hybrid-A*",  "smac"),
}


class GoalSetter(Node):

    def __init__(self):
        super().__init__("set_nav_goal")

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        latching_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "initialpose", latching_qos
        )

    # ────────────────────────────────────────────────────────────────────
    def set_initial_pose(self, x: float, y: float, yaw: float = 0.0):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0]  = 0.25
        msg.pose.covariance[7]  = 0.25
        msg.pose.covariance[35] = 0.068
        self._initial_pose_pub.publish(msg)
        self.get_logger().info(f"▶  Initial pose published: ({x:.2f}, {y:.2f})")
        time.sleep(0.5)

    # ────────────────────────────────────────────────────────────────────
    def send_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        self.get_logger().info("Waiting for Nav2 action server…")
        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "Nav2 action server not available! "
                "Make sure navigation.launch.py is running."
            )
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f"▶  Sending goal: ({x:.2f}, {y:.2f})")
        future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        ok = result.status == GoalStatus.STATUS_SUCCEEDED
        if ok:
            self.get_logger().info("✔  Goal reached!")
        else:
            self.get_logger().error(f"✘  Navigation failed (status={result.status})")
        return ok

    def _feedback_cb(self, fb):
        dist = fb.feedback.distance_remaining
        self.get_logger().info(
            f"   Distance remaining: {dist:.2f} m", throttle_duration_sec=2
        )


# ─────────────────────────────────────────────────────────────────────────────

def _prompt_float(prompt: str) -> float:
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("  Please enter a valid number.")


def _prompt_choice(options: dict) -> str:
    while True:
        choice = input("Your choice: ").strip()
        if choice in options:
            return choice
        print(f"  Invalid choice. Enter one of: {', '.join(options.keys())}")


def main():
    print("=" * 60)
    print("  Drone Navigation Goal Setter")
    print("=" * 60)

    # ── Point A ──────────────────────────────────────────────────────────
    print("\n[1/3] Set the START position (Point A):")
    start_x = _prompt_float("  Start X [m]: ")
    start_y = _prompt_float("  Start Y [m]: ")

    # ── Point B ──────────────────────────────────────────────────────────
    print("\n[2/3] Set the GOAL position (Point B):")
    goal_x = _prompt_float("  Goal X [m]: ")
    goal_y = _prompt_float("  Goal Y [m]: ")
    goal_yaw = _prompt_float("  Goal yaw [rad, default 0]: ") \
        if input("  Set goal orientation? [y/N]: ").strip().lower() == "y" \
        else 0.0

    # ── Algorithm ────────────────────────────────────────────────────────
    print("\n[3/3] Choose a navigation algorithm:")
    for key, (name, _) in ALGORITHMS.items():
        print(f"  {key}) {name}")
    algo_key   = _prompt_choice(ALGORITHMS)
    algo_name, algo_id = ALGORITHMS[algo_key]
    print(f"\n  Selected: {algo_name} ({algo_id})\n")

    # ── Run ──────────────────────────────────────────────────────────────
    print("=" * 60)
    print(f"  Start : ({start_x:.2f}, {start_y:.2f})")
    print(f"  Goal  : ({goal_x:.2f}, {goal_y:.2f})  yaw={goal_yaw:.3f}")
    print(f"  Algo  : {algo_name}")
    print("=" * 60)
    print()

    rclpy.init()
    node = GoalSetter()
    node.set_initial_pose(start_x, start_y)
    success = node.send_goal(goal_x, goal_y, goal_yaw)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
