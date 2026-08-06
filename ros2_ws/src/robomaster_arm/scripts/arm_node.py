#!/usr/bin/env python3
"""Cartesian arm + gripper action server.

sim=true  -> JointGroupPositionController commands (serial-equivalent URDF)
sim=false -> robomaster_driver ArmSdk / GripperSdk (shared TCP with wheels)
"""

from __future__ import annotations

import math
import os
import sys
import time

# Sibling module installed next to this script in lib/robomaster_arm.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from arm_kinematics import (
    GRIPPER_CLOSED,
    GRIPPER_OPEN,
    HOME_X,
    HOME_Z,
    fk,
    solve,
)
from robomaster_arm.action import MoveArm, SetGripper
from robomaster_arm.msg import ArmState, GripperState
from robomaster_driver.srv import ArmSdk, GripperSdk


class ArmNode(Node):
    def __init__(self) -> None:
        super().__init__("robomaster_arm")
        self.declare_parameter("sim", True)
        self.declare_parameter("home_x", HOME_X)
        self.declare_parameter("home_z", HOME_Z)
        self.sim = self.get_parameter("sim").get_parameter_value().bool_value
        self._home = (
            self.get_parameter("home_x").value,
            self.get_parameter("home_z").value,
        )

        self._cb = ReentrantCallbackGroup()
        self._arm_1 = 0.0
        self._arm_2 = 0.0
        self._gripper = GRIPPER_OPEN
        self._moving = False
        self._gripper_state = GripperState.OPEN

        self._arm_state_pub = self.create_publisher(ArmState, "~/arm_state", 10)
        self._gripper_state_pub = self.create_publisher(
            GripperState, "~/gripper_state", 10
        )
        self.create_timer(0.05, self._publish_state)

        if self.sim:
            self._arm_cmd = self.create_publisher(
                Float64MultiArray, "/arm_controller/commands", 10
            )
            self._gripper_cmd = self.create_publisher(
                Float64MultiArray, "/gripper_controller/commands", 10
            )
            self.create_subscription(
                JointState, "/joint_states", self._on_joint_states, 10
            )
            self._arm_cli = None
            self._gripper_cli = None
            # Joint zeros put the arm straight up at full extension, where the
            # chain is singular and every outward jog is unreachable. Leave
            # that pose as soon as the controller is listening.
            self._homed = False
            self._home_timer = self.create_timer(0.5, self._try_home)
        else:
            self._arm_cmd = None
            self._gripper_cmd = None
            self._arm_cli = self.create_client(
                ArmSdk, "robomaster_driver/arm_sdk", callback_group=self._cb
            )
            self._gripper_cli = self.create_client(
                GripperSdk, "robomaster_driver/gripper_sdk", callback_group=self._cb
            )

        self._move_server = ActionServer(
            self,
            MoveArm,
            "~/move_arm",
            execute_callback=self._execute_move,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._cb,
        )
        self._gripper_server = ActionServer(
            self,
            SetGripper,
            "~/set_gripper",
            execute_callback=self._execute_gripper,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._cb,
        )
        self.get_logger().info(f"arm node ready (sim={self.sim})")

    def _try_home(self) -> None:
        if self._homed or self._arm_cmd.get_subscription_count() < 1:
            return
        arm_1, arm_2, _ = solve(*self._home)
        self._arm_cmd.publish(Float64MultiArray(data=[arm_1, arm_2]))
        self._gripper_cmd.publish(
            Float64MultiArray(data=[GRIPPER_OPEN, -GRIPPER_OPEN])
        )
        self._homed = True
        self._home_timer.cancel()
        self.get_logger().info(
            f"homed to x={self._home[0]:.3f} z={self._home[1]:.3f}"
        )

    def _accept_goal(self, _goal_request):
        return GoalResponse.ACCEPT

    def _accept_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _on_joint_states(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        if "arm_1_joint" in name_to_pos:
            self._arm_1 = name_to_pos["arm_1_joint"]
        if "arm_2_joint" in name_to_pos:
            self._arm_2 = name_to_pos["arm_2_joint"]
        if "gripper_m_joint" in name_to_pos:
            self._gripper = name_to_pos["gripper_m_joint"]
            mid = 0.5 * (GRIPPER_OPEN + GRIPPER_CLOSED)
            self._gripper_state = (
                GripperState.OPEN if self._gripper >= mid else GripperState.CLOSED
            )

    def _publish_state(self) -> None:
        x, z = fk(self._arm_1, self._arm_2)
        if not self.sim and self._arm_cli is not None and self._arm_cli.service_is_ready():
            # Prefer SDK position when tethered.
            pass
        arm = ArmState()
        arm.header.stamp = self.get_clock().now().to_msg()
        arm.x = x
        arm.z = z
        arm.arm_1 = self._arm_1
        arm.arm_2 = self._arm_2
        arm.moving = self._moving
        self._arm_state_pub.publish(arm)

        grip = GripperState()
        grip.header.stamp = arm.header.stamp
        grip.state = self._gripper_state
        grip.opening = self._gripper
        self._gripper_state_pub.publish(grip)

    def _wait_sdk(self, client, timeout: float = 30.0) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.5):
                return True
        return False

    def _execute_move(self, goal_handle):
        goal = goal_handle.request
        result = MoveArm.Result()
        feedback = MoveArm.Feedback()

        cur_x, cur_z = fk(self._arm_1, self._arm_2)
        if goal.absolute:
            target_x, target_z = goal.x, goal.z
        else:
            target_x, target_z = cur_x + goal.x, cur_z + goal.z

        self._moving = True
        try:
            if self.sim:
                ok, msg = self._move_sim(target_x, target_z, goal_handle, feedback)
            else:
                ok, msg = self._move_tether(
                    target_x, target_z, goal.absolute, goal_handle, feedback
                )
        finally:
            self._moving = False

        cur_x, cur_z = fk(self._arm_1, self._arm_2)
        result.success = ok
        result.message = msg
        result.x = cur_x
        result.z = cur_z
        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _move_sim(self, target_x, target_z, goal_handle, feedback):
        arm_1, arm_2, notes = solve(target_x, target_z)
        self._arm_cmd.publish(Float64MultiArray(data=[arm_1, arm_2]))
        # Settle against the pose actually commanded, not the raw request, so a
        # clamped goal still reports success once it arrives.
        reached_x, reached_z = fk(arm_1, arm_2)
        message = "; ".join(["ok"] + notes)

        deadline = time.monotonic() + 8.0
        last_x, last_z = fk(self._arm_1, self._arm_2)
        last_progress = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False, "canceled"
            x, z = fk(self._arm_1, self._arm_2)
            feedback.x = x
            feedback.z = z
            feedback.moving = True
            goal_handle.publish_feedback(feedback)
            error = math.hypot(x - reached_x, z - reached_z)
            if error < 0.012:
                return True, message
            now = time.monotonic()
            if math.hypot(x - last_x, z - last_z) > 0.001:
                last_x, last_z = x, z
                last_progress = now
            elif now - last_progress > 1.0:
                # Position control is proportional-only, so a pose held against
                # a joint limit or gravity keeps a steady-state offset. Stop
                # waiting on an error that will not shrink; report where it sat.
                return True, f"{message}; stalled {error * 100:.1f} cm short"
            time.sleep(0.05)
        return True, f"{message}; did not settle within 8s"

    def _move_tether(self, target_x, target_z, _absolute, goal_handle, feedback):
        if not self._wait_sdk(self._arm_cli):
            return False, "arm_sdk unavailable"
        # SDK: x forward cm, y up cm. Our API: x forward m, z up m.
        # Caller already resolved relative goals to absolute metres.
        req = ArmSdk.Request()
        req.command = "moveto"
        req.x_cm = target_x * 100.0
        req.y_cm = target_z * 100.0
        future = self._arm_cli.call_async(req)
        while rclpy.ok() and not future.done():
            if goal_handle.is_cancel_requested:
                stop = ArmSdk.Request()
                stop.command = "stop"
                self._arm_cli.call_async(stop)
                goal_handle.canceled()
                return False, "canceled"
            x, z = fk(self._arm_1, self._arm_2)
            feedback.x = x
            feedback.z = z
            feedback.moving = True
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)
        res = future.result()
        if res is None or not res.success:
            return False, (res.message if res else "no response")
        # Refresh FK estimate from SDK position query.
        pos = ArmSdk.Request()
        pos.command = "position"
        pos_fut = self._arm_cli.call_async(pos)
        t0 = time.monotonic()
        while rclpy.ok() and not pos_fut.done() and time.monotonic() - t0 < 2.0:
            time.sleep(0.05)
        if pos_fut.done() and pos_fut.result() and pos_fut.result().success:
            px = pos_fut.result().x_cm / 100.0
            pz = pos_fut.result().y_cm / 100.0
            self._arm_1, self._arm_2, _ = solve(px, pz)
        return True, res.message

    def _execute_gripper(self, goal_handle):
        goal = goal_handle.request
        result = SetGripper.Result()
        open_cmd = goal.command == SetGripper.Goal.OPEN

        if self.sim:
            target = GRIPPER_OPEN if open_cmd else GRIPPER_CLOSED
            self._gripper_cmd.publish(
                Float64MultiArray(data=[target, -target])
            )
            deadline = time.monotonic() + 3.0
            while rclpy.ok() and time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = "canceled"
                    result.state = self._gripper_state
                    return result
                if abs(self._gripper - target) < 0.003:
                    break
                time.sleep(0.05)
            self._gripper_state = (
                GripperState.OPEN if open_cmd else GripperState.CLOSED
            )
            result.success = True
            result.message = "ok"
            result.state = self._gripper_state
            goal_handle.succeed()
            return result

        if not self._wait_sdk(self._gripper_cli):
            result.success = False
            result.message = "gripper_sdk unavailable"
            result.state = GripperState.UNKNOWN
            goal_handle.abort()
            return result
        req = GripperSdk.Request()
        req.command = "open" if open_cmd else "close"
        req.force_level = goal.force_level
        fut = self._gripper_cli.call_async(req)
        while rclpy.ok() and not fut.done():
            time.sleep(0.05)
        res = fut.result()
        if res is None or not res.success:
            result.success = False
            result.message = res.message if res else "no response"
            result.state = GripperState.UNKNOWN
            goal_handle.abort()
            return result
        self._gripper_state = GripperState.OPEN if open_cmd else GripperState.CLOSED
        result.success = True
        result.message = res.message
        result.state = self._gripper_state
        goal_handle.succeed()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
