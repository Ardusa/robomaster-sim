#!/usr/bin/env python3
"""Keyboard teleop: chassis Twist + Cartesian arm / gripper actions.

Run this directly (`ros2 run`), never under `ros2 launch`: launch gives child
processes /dev/null on stdin, so raw-mode reads fail. Chassis keys match
teleop_twist_keyboard; arm keys call robomaster_arm actions.
"""

from __future__ import annotations

import select
import sys
import termios
import threading
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.node import Node

from robomaster_arm.action import MoveArm, SetGripper

MSG = """
RoboMaster teleop — chassis + arm
---------------------------------
Chassis (holonomic):
   u    i    o
   j    k    l
   m    ,    .

  q/z : scale +/- 10%
  w/x : only linear scale
  e/c : only angular scale

Arm (Cartesian jog, metres):
  t/g : +x / -x (forward)
  y/h : +z / -z (up)
  1   : tuck-ish preset
  2   : reach forward
  3   : raise
  [ / ] : gripper open / close

CTRL-C to quit
"""

MOVE_BINDINGS = {
    "i": (1, 0, 0, 0),
    "o": (1, 0, 0, -1),
    "j": (0, 0, 0, 1),
    "l": (0, 0, 0, -1),
    "u": (1, 0, 0, 1),
    ",": (-1, 0, 0, 0),
    ".": (-1, 0, 0, 1),
    "m": (-1, 0, 0, -1),
    "O": (1, -1, 0, 0),
    "I": (1, 0, 0, 0),
    "J": (0, 1, 0, 0),
    "L": (0, -1, 0, 0),
    "U": (1, 1, 0, 0),
    "<": (-1, 0, 0, 0),
    ">": (-1, -1, 0, 0),
    "M": (-1, 1, 0, 0),
    "t": "arm_x+",
    "g": "arm_x-",
    "y": "arm_z+",
    "h": "arm_z-",
    "1": "preset_tuck",
    "2": "preset_reach",
    "3": "preset_raise",
    "[": "grip_open",
    "]": "grip_close",
}

SPEED_BINDINGS = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
    "w": (1.1, 1.0),
    "x": (0.9, 1.0),
    "e": (1.0, 1.1),
    "c": (1.0, 0.9),
}

PRESETS = {
    "preset_tuck": (0.05, 0.18),
    "preset_reach": (0.18, 0.10),
    "preset_raise": (0.08, 0.22),
}

JOG = 0.02  # metres per keypress


class TeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("robomaster_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel_teleop", 1)
        self.move_arm = ActionClient(self, MoveArm, "/robomaster_arm/move_arm")
        self.set_gripper = ActionClient(self, SetGripper, "/robomaster_arm/set_gripper")
        self.speed = 0.3
        self.turn = 0.8
        self._lock = threading.Lock()
        self._busy = False

    def send_twist(self, x, y, th) -> None:
        tw = Twist()
        tw.linear.x = x * self.speed
        tw.linear.y = y * self.speed
        tw.angular.z = th * self.turn
        self.pub.publish(tw)

    def stop(self) -> None:
        self.pub.publish(Twist())

    def _arm_goal(self, x: float, z: float, absolute: bool) -> None:
        with self._lock:
            if self._busy:
                return
            self._busy = True
        if not self.move_arm.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("move_arm action not available")
            with self._lock:
                self._busy = False
            return
        goal = MoveArm.Goal()
        goal.x = x
        goal.z = z
        goal.absolute = absolute
        fut = self.move_arm.send_goal_async(goal)

        def _done(f):
            with self._lock:
                self._busy = False
            try:
                gh = f.result()
                if gh is None or not gh.accepted:
                    self.get_logger().warn("move_arm rejected")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"move_arm failed: {exc}")

        fut.add_done_callback(_done)

    def _gripper(self, open_cmd: bool) -> None:
        if not self.set_gripper.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("set_gripper action not available")
            return
        goal = SetGripper.Goal()
        goal.command = SetGripper.Goal.OPEN if open_cmd else SetGripper.Goal.CLOSE
        goal.force_level = 1
        self.set_gripper.send_goal_async(goal)

    def handle_key(self, key: str) -> None:
        if key in SPEED_BINDINGS:
            ds, dt = SPEED_BINDINGS[key]
            self.speed = max(0.05, self.speed * ds)
            self.turn = max(0.05, self.turn * dt)
            print(f"speed {self.speed:.2f} turn {self.turn:.2f}")
            return
        binding = MOVE_BINDINGS.get(key)
        if binding is None:
            self.stop()
            return
        if isinstance(binding, str):
            if binding == "arm_x+":
                self._arm_goal(JOG, 0.0, False)
            elif binding == "arm_x-":
                self._arm_goal(-JOG, 0.0, False)
            elif binding == "arm_z+":
                self._arm_goal(0.0, JOG, False)
            elif binding == "arm_z-":
                self._arm_goal(0.0, -JOG, False)
            elif binding.startswith("preset_"):
                x, z = PRESETS[binding]
                self._arm_goal(x, z, True)
            elif binding == "grip_open":
                self._gripper(True)
            elif binding == "grip_close":
                self._gripper(False)
            return
        x, y, _z, th = binding
        self.send_twist(x, y, th)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None) -> None:
    if not sys.stdin.isatty():
        sys.exit(
            "teleop needs a terminal on stdin. Run it with `ros2 run "
            "robomaster_teleop teleop_node.py` in an interactive shell — not "
            "under `ros2 launch`, and not with `docker compose exec -T`."
        )
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = TeleopNode()
    print(MSG)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            key = get_key(settings)
            if key == "\x03":
                break
            if key:
                node.handle_key(key)
            else:
                node.stop()
    except Exception as exc:  # noqa: BLE001
        print(exc)
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
