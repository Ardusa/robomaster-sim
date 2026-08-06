#!/usr/bin/env python3
"""Execute ActionPrimitive sequences via drivetrain + arm public interfaces."""

from __future__ import annotations

import math
import os
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robomaster_arm.action import MoveArm, SetGripper
from robomaster_command.msg import ActionPrimitive
from robomaster_command.srv import ExecuteActionSequence

# Two-phase navigate P-controller (rotate then drive).
NAV_LINEAR_GAIN = 0.6
NAV_ANGULAR_GAIN = 1.5
NAV_LINEAR_MAX = 0.35
NAV_ANGULAR_MAX = 0.8
NAV_POS_TOL = 0.12  # m
NAV_YAW_TOL = 0.12  # rad (~7 deg)
NAV_FACE_TOL = 0.15  # rad before starting the drive phase
NAV_TIMEOUT_SEC = 15.0
NAV_RATE_HZ = 20.0

ACTION_SERVER_WAIT_SEC = 5.0
ACTION_RESULT_TIMEOUT_SEC = 30.0


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TranslatorNode(Node):
    def __init__(self) -> None:
        super().__init__("robomaster_command_translator")
        self.declare_parameter("semantic_map_path", "")

        self._cb = ReentrantCallbackGroup()
        self._odom_lock = threading.Lock()
        self._odom: dict[str, float] | None = None

        map_path = self.get_parameter("semantic_map_path").get_parameter_value().string_value
        self._zones = self._load_semantic_map(map_path)
        if self._zones is None:
            return

        self.create_subscription(
            Odometry,
            "/mecanum_drive_controller/odometry",
            self._on_odom,
            10,
            callback_group=self._cb,
        )
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_autonomy", 10)
        self._move_arm = ActionClient(
            self, MoveArm, "/robomaster_arm/move_arm", callback_group=self._cb
        )
        self._set_gripper = ActionClient(
            self, SetGripper, "/robomaster_arm/set_gripper", callback_group=self._cb
        )
        self.create_service(
            ExecuteActionSequence,
            "~/execute_action_sequence",
            self._on_execute,
            callback_group=self._cb,
        )
        self.get_logger().info(
            f"translator ready; zones={sorted(self._zones)} map={map_path}"
        )

    def _load_semantic_map(self, map_path: str) -> dict[str, dict[str, float]] | None:
        if not map_path:
            self.get_logger().error(
                "semantic_map_path is unset. Pass it via command.launch.py "
                "(config/semantic_maps/<semantic_map>)."
            )
            return None
        if not os.path.isfile(map_path):
            self.get_logger().error(
                f"semantic map file does not exist: {map_path}"
            )
            return None
        try:
            with open(map_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"failed to parse semantic map {map_path}: {exc}")
            return None

        zones_raw = data.get("zones") if isinstance(data, dict) else None
        if not isinstance(zones_raw, dict) or not zones_raw:
            self.get_logger().error(
                f"semantic map {map_path} has no usable 'zones' mapping"
            )
            return None

        zones: dict[str, dict[str, float]] = {}
        for name, pose in zones_raw.items():
            try:
                zones[str(name)] = {
                    "x": float(pose["x"]),
                    "y": float(pose["y"]),
                    "theta": float(pose["theta"]),
                }
            except (TypeError, KeyError, ValueError) as exc:
                self.get_logger().error(
                    f"semantic map {map_path}: zone {name!r} is invalid: {exc}"
                )
                return None
        return zones

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        with self._odom_lock:
            self._odom = {"x": float(p.x), "y": float(p.y), "yaw": float(yaw)}

    def _pose(self) -> dict[str, float] | None:
        with self._odom_lock:
            return None if self._odom is None else dict(self._odom)

    def _stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _on_execute(
        self,
        request: ExecuteActionSequence.Request,
        response: ExecuteActionSequence.Response,
    ) -> ExecuteActionSequence.Response:
        for i, action in enumerate(request.actions):
            ok, message = self._run_action(i, action)
            if not ok:
                self._stop()
                response.success = False
                response.message = message
                return response
        self._stop()
        response.success = True
        response.message = f"executed {len(request.actions)} action(s)"
        return response

    def _run_action(
        self, index: int, action: ActionPrimitive
    ) -> tuple[bool, str]:
        kind = action.type.strip().lower()
        self.get_logger().info(f"step {index}: {kind}")
        if kind == "navigate":
            return self._navigate(index, action)
        if kind == "arm_goto":
            return self._arm_goto(index, action)
        if kind == "gripper":
            return self._gripper(index, action)
        return False, f"step {index}: unknown type {action.type!r}"

    def _resolve_nav_goal(
        self, action: ActionPrimitive
    ) -> tuple[float, float, float] | None:
        zone = action.target_zone.strip()
        if zone:
            pose = self._zones.get(zone)
            if pose is None:
                return None
            return pose["x"], pose["y"], pose["theta"]
        # Explicit map-frame goal when target_zone is empty.
        return float(action.x), float(action.y), float(action.theta)

    def _navigate(
        self, index: int, action: ActionPrimitive
    ) -> tuple[bool, str]:
        goal = self._resolve_nav_goal(action)
        if goal is None:
            zone = action.target_zone.strip()
            return (
                False,
                f"step {index}: navigate target_zone {zone!r} not in semantic map",
            )

        gx, gy, gtheta = goal
        label = action.target_zone.strip() or f"({gx:.2f},{gy:.2f},{gtheta:.2f})"
        self.get_logger().info(f"navigate to {label} -> ({gx:.2f}, {gy:.2f}, {gtheta:.2f})")

        if self._pose() is None:
            # Wait briefly for first odom.
            deadline = time.monotonic() + 2.0
            while self._pose() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            if self._pose() is None:
                return False, f"step {index}: no odometry on /mecanum_drive_controller/odometry"

        period = 1.0 / NAV_RATE_HZ
        start = time.monotonic()
        facing = False

        while time.monotonic() - start < NAV_TIMEOUT_SEC:
            pose = self._pose()
            if pose is None:
                time.sleep(period)
                continue

            dx = gx - pose["x"]
            dy = gy - pose["y"]
            dist = math.hypot(dx, dy)
            bearing = math.atan2(dy, dx)
            yaw_err_face = _wrap_angle(bearing - pose["yaw"])
            yaw_err_goal = _wrap_angle(gtheta - pose["yaw"])

            cmd = Twist()
            if not facing:
                if abs(yaw_err_face) < NAV_FACE_TOL or dist < NAV_POS_TOL:
                    facing = True
                else:
                    cmd.angular.z = _clamp(NAV_ANGULAR_GAIN * yaw_err_face, NAV_ANGULAR_MAX)
            elif dist > NAV_POS_TOL:
                # Keep a light heading correction while driving.
                cmd.linear.x = _clamp(NAV_LINEAR_GAIN * dist, NAV_LINEAR_MAX)
                cmd.angular.z = _clamp(0.5 * NAV_ANGULAR_GAIN * yaw_err_face, NAV_ANGULAR_MAX)
            elif abs(yaw_err_goal) > NAV_YAW_TOL:
                cmd.angular.z = _clamp(NAV_ANGULAR_GAIN * yaw_err_goal, NAV_ANGULAR_MAX)
            else:
                self._stop()
                return True, ""

            self._cmd_pub.publish(cmd)
            time.sleep(period)

        self._stop()
        return False, f"step {index}: navigate to {label} timed out after {NAV_TIMEOUT_SEC:.0f}s"

    def _wait_action_result(self, client, goal_msg, label: str, index: int):
        if not client.wait_for_server(timeout_sec=ACTION_SERVER_WAIT_SEC):
            return False, f"step {index}: {label} action server not available"

        send_future = client.send_goal_async(goal_msg)
        try:
            goal_handle = send_future.result(timeout=ACTION_SERVER_WAIT_SEC)
        except Exception as exc:  # noqa: BLE001
            return False, f"step {index}: {label} send failed: {exc}"

        if goal_handle is None or not goal_handle.accepted:
            return False, f"step {index}: {label} goal rejected"

        result_future = goal_handle.get_result_async()
        try:
            wrapped = result_future.result(timeout=ACTION_RESULT_TIMEOUT_SEC)
        except Exception as exc:  # noqa: BLE001
            return False, f"step {index}: {label} result wait failed: {exc}"

        result = wrapped.result
        if not result.success:
            return False, f"step {index}: {label} failed: {result.message}"
        return True, ""

    def _arm_goto(
        self, index: int, action: ActionPrimitive
    ) -> tuple[bool, str]:
        goal = MoveArm.Goal()
        goal.x = float(action.arm_x)
        goal.z = float(action.arm_z)
        goal.absolute = True
        goal.use_joints = False
        return self._wait_action_result(self._move_arm, goal, "move_arm", index)

    def _gripper(
        self, index: int, action: ActionPrimitive
    ) -> tuple[bool, str]:
        goal = SetGripper.Goal()
        goal.command = (
            SetGripper.Goal.OPEN if action.gripper_open else SetGripper.Goal.CLOSE
        )
        goal.force_level = 1
        return self._wait_action_result(self._set_gripper, goal, "set_gripper", index)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TranslatorNode()
    if node._zones is None:
        node.destroy_node()
        rclpy.shutdown()
        return

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
