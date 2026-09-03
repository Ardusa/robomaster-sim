#!/usr/bin/env python3
"""Execute ActionPrimitive sequences via drivetrain Nav2 + arm interfaces."""

from __future__ import annotations

import os
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robomaster_arm.action import MoveArm, SetGripper
from robomaster_command.msg import ActionPrimitive
from robomaster_command.srv import ExecuteActionSequence
from robomaster_drivetrain.srv import GoToPose

ACTION_SERVER_WAIT_SEC = 5.0
ACTION_RESULT_TIMEOUT_SEC = 30.0
NAV_SERVICE_WAIT_SEC = 30.0
NAV_CALL_TIMEOUT_SEC = 180.0


class TranslatorNode(Node):
    def __init__(self) -> None:
        super().__init__("robomaster_command_translator")
        self.declare_parameter("semantic_map_path", "")

        self._cb = ReentrantCallbackGroup()

        map_path = self.get_parameter("semantic_map_path").get_parameter_value().string_value
        self._zones = self._load_semantic_map(map_path)
        if self._zones is None:
            return

        self._go_to_pose = self.create_client(
            GoToPose, "/go_to_pose/go_to_pose", callback_group=self._cb
        )
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
                "(config/semantic_maps/<world_stem>.yaml)."
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
        self.get_logger().info(
            f"semantic map loaded from {map_path} (map-frame / world coords)"
        )
        return zones

    def _call_go_to_pose(self, x: float, y: float, theta: float) -> tuple[bool, str]:
        if not self._go_to_pose.wait_for_service(timeout_sec=NAV_SERVICE_WAIT_SEC):
            return False, "go_to_pose not available — is Nav2 launched?"
        req = GoToPose.Request()
        req.x = x
        req.y = y
        req.theta = theta
        future = self._go_to_pose.call_async(req)
        deadline = time.monotonic() + NAV_CALL_TIMEOUT_SEC
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                return False, f"navigation timed out after {NAV_CALL_TIMEOUT_SEC:.0f}s"
            time.sleep(0.05)
        res = future.result()
        if res is None:
            return False, "go_to_pose returned no response"
        if not res.success:
            return False, res.message or "go_to_pose failed"
        return True, ""

    def _on_execute(
        self,
        request: ExecuteActionSequence.Request,
        response: ExecuteActionSequence.Response,
    ) -> ExecuteActionSequence.Response:
        for i, action in enumerate(request.actions):
            ok, message = self._run_action(i, action)
            if not ok:
                response.success = False
                response.message = message
                return response
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
        return (
            False,
            f"step {index}: unrecognized type {action.type!r} "
            "(expected navigate|arm_goto|gripper)",
        )

    def _resolve_nav_goal(
        self, action: ActionPrimitive
    ) -> tuple[tuple[float, float, float] | None, str]:
        zone = action.target_zone.strip()
        if zone:
            pose = self._zones.get(zone)
            if pose is None:
                return None, f"navigate target_zone {zone!r} not in semantic map"
            return (pose["x"], pose["y"], pose["theta"]), ""
        if action.use_explicit_pose:
            return (float(action.x), float(action.y), float(action.theta)), ""
        return (
            None,
            "navigate primitive has no target_zone and use_explicit_pose is false",
        )

    def _navigate(
        self, index: int, action: ActionPrimitive
    ) -> tuple[bool, str]:
        goal, err = self._resolve_nav_goal(action)
        if goal is None:
            return False, f"step {index}: {err}"

        gx, gy, gtheta = goal
        label = action.target_zone.strip() or f"({gx:.2f},{gy:.2f},{gtheta:.2f})"
        self.get_logger().info(
            f"navigate to {label} -> map ({gx:.2f}, {gy:.2f}, {gtheta:.2f}) via Nav2"
        )
        ok, message = self._call_go_to_pose(gx, gy, gtheta)
        if not ok:
            return False, f"step {index}: {message}"
        return True, ""

    def _wait_action_result(self, client, goal_msg, label: str, index: int):
        if not client.wait_for_server(timeout_sec=ACTION_SERVER_WAIT_SEC):
            return False, f"step {index}: {label} action server not available"

        send_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(
            self, send_future, timeout_sec=ACTION_SERVER_WAIT_SEC
        )
        if not send_future.done():
            return False, f"step {index}: {label} send timed out"
        try:
            goal_handle = send_future.result()
        except Exception as exc:  # noqa: BLE001
            return False, f"step {index}: {label} send failed: {exc}"

        if goal_handle is None or not goal_handle.accepted:
            return False, f"step {index}: {label} goal rejected"

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=ACTION_RESULT_TIMEOUT_SEC
        )
        if not result_future.done():
            return False, f"step {index}: {label} result timed out"
        try:
            wrapped = result_future.result()
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
