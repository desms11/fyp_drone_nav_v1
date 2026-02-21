#!/usr/bin/env python3
"""
drone_teleop/scripts/teleop_keyboard.py

Keyboard teleoperation for the drone.
Publishes geometry_msgs/Twist on /cmd_vel.

Controls
--------
  w / s        : increase / decrease forward linear velocity
  a / d        : rotate left / right (yaw)
  u / j        : strafe left / right (lateral velocity)
  t / g        : increase / decrease altitude (vertical velocity)
  SPACE        : stop (zero all velocities)
  q            : quit

Hold a key to keep sending; release to let velocity decay.

Usage:
  ros2 run drone_teleop teleop_keyboard.py
  ros2 run drone_teleop teleop_keyboard.py --ros-args -p max_linear_vel:=1.0
"""

import sys
import select
import tty
import termios
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ── Key definitions ───────────────────────────────────────────────────────────
MSG_HEADER = """
╔══════════════════════════════════════════════════════════╗
║          Drone Keyboard Teleoperation                    ║
╠══════════════════════════════════════════════════════════╣
║  w / s   : forward / backward (linear X)                ║
║  a / d   : rotate left / right (angular Z / yaw)        ║
║  u / j   : strafe left / right (linear Y)               ║
║  t / g   : ascend / descend (linear Z)                  ║
║  SPACE   : emergency stop (zero all)                    ║
║  q       : quit                                         ║
╠══════════════════════════════════════════════════════════╣
║  Current speeds are shown below.                        ║
║  Each key press increments / decrements by a step.     ║
╚══════════════════════════════════════════════════════════╝
"""

KEY_BINDINGS = {
    # key : (linear_x, linear_y, linear_z, angular_z)
    "w":  ( 1,  0,  0,  0),
    "s":  (-1,  0,  0,  0),
    "a":  ( 0,  0,  0,  1),
    "d":  ( 0,  0,  0, -1),
    "u":  ( 0,  1,  0,  0),
    "j":  ( 0, -1,  0,  0),
    "t":  ( 0,  0,  1,  0),
    "g":  ( 0,  0, -1,  0),
}

SPEED_BINDINGS = {
    "q": None,  # quit – handled separately
    " ": None,  # stop – handled separately
}


def get_key(settings):
    """Read a single key from stdin (non-blocking with 0.05 s timeout)."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopKeyboard(Node):

    def __init__(self):
        super().__init__("teleop_keyboard")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("max_linear_vel",  0.5)
        self.declare_parameter("max_angular_vel", 1.5)
        self.declare_parameter("max_vertical_vel", 0.5)
        self.declare_parameter("linear_step",    0.05)
        self.declare_parameter("angular_step",   0.1)
        self.declare_parameter("vertical_step",  0.05)
        self.declare_parameter("publish_rate",   20.0)
        self.declare_parameter("cmd_vel_topic",  "/cmd_vel")

        self.max_lin  = self.get_parameter("max_linear_vel").value
        self.max_ang  = self.get_parameter("max_angular_vel").value
        self.max_vert = self.get_parameter("max_vertical_vel").value
        self.lin_step = self.get_parameter("linear_step").value
        self.ang_step = self.get_parameter("angular_step").value
        self.ver_step = self.get_parameter("vertical_step").value
        rate          = self.get_parameter("publish_rate").value
        topic         = self.get_parameter("cmd_vel_topic").value

        # ── State ─────────────────────────────────────────────────────────
        self.vx = 0.0   # forward / backward
        self.vy = 0.0   # strafe
        self.vz = 0.0   # altitude
        self.wz = 0.0   # yaw

        self._lock = threading.Lock()

        # ── Publisher ─────────────────────────────────────────────────────
        self._pub = self.create_publisher(Twist, topic, 10)
        self._timer = self.create_timer(1.0 / rate, self._publish_cb)

        self.get_logger().info(
            f"TeleopKeyboard started  (topic={topic}, max_lin={self.max_lin} m/s)"
        )

    def _publish_cb(self):
        with self._lock:
            msg = Twist()
            msg.linear.x  = self.vx
            msg.linear.y  = self.vy
            msg.linear.z  = self.vz
            msg.angular.z = self.wz
        self._pub.publish(msg)

    def update_velocity(self, key: str) -> bool:
        """Return False if the user wants to quit."""
        if key == "q":
            return False
        if key == " ":
            with self._lock:
                self.vx = self.vy = self.vz = self.wz = 0.0
            return True
        if key in KEY_BINDINGS:
            dx, dy, dz, dw = KEY_BINDINGS[key]
            with self._lock:
                self.vx = self._clamp(self.vx + dx * self.lin_step, self.max_lin)
                self.vy = self._clamp(self.vy + dy * self.lin_step, self.max_lin)
                self.vz = self._clamp(self.vz + dz * self.ver_step, self.max_vert)
                self.wz = self._clamp(self.wz + dw * self.ang_step, self.max_ang)
        return True

    @staticmethod
    def _clamp(val: float, limit: float) -> float:
        return max(-limit, min(limit, val))

    def print_status(self):
        with self._lock:
            print(
                f"\r  vx={self.vx:+.2f}  vy={self.vy:+.2f}  "
                f"vz={self.vz:+.2f}  wz={self.wz:+.2f}   ",
                end="",
                flush=True,
            )

    def emergency_stop(self):
        with self._lock:
            self.vx = self.vy = self.vz = self.wz = 0.0
        self._publish_cb()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    settings = termios.tcgetattr(sys.stdin)
    print(MSG_HEADER)

    # Spin in a background thread so the publisher timer works
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while True:
            key = get_key(settings)
            if not node.update_velocity(key):
                break
            if key:
                node.print_status()
    except Exception as exc:
        node.get_logger().error(f"Teleoperation error: {exc}")
    finally:
        node.emergency_stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("\nTeleoperation ended. All velocities zeroed.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
