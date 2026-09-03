#!/usr/bin/env python3
"""Send map-frame goals to Nav2 NavigateToPose; drivetrain's navigation API."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robomaster_drivetrain.srv import GoToPose

GOAL_TIMEOUT_SEC = 180.0
SERVER_WAIT_SEC = 30.0
SEND_GOAL_TIMEOUT_SEC = 10.0


def _quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _wait_for_future(future, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and not future.done():
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.02)
    return future.done()


class GoToPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("go_to_pose")
        self._cb = ReentrantCallbackGroup()
        self._client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self._cb
        )
        self.create_service(
            GoToPose, "~/go_to_pose", self._on_go_to_pose, callback_group=self._cb
        )
        self.get_logger().info("ready on ~/go_to_pose (Nav2 navigate_to_pose client)")

    def _on_go_to_pose(
        self, request: GoToPose.Request, response: GoToPose.Response
    ) -> GoToPose.Response:
        if not self._client.wait_for_server(timeout_sec=SERVER_WAIT_SEC):
            response.success = False
            response.message = "navigate_to_pose action server not available"
            return response

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(request.x)
        goal.pose.pose.position.y = float(request.y)
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = _quat_from_yaw(float(request.theta))

        send_future = self._client.send_goal_async(goal)
        if not _wait_for_future(send_future, SEND_GOAL_TIMEOUT_SEC):
            response.success = False
            response.message = "timed out sending NavigateToPose goal"
            return response

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            response.success = False
            response.message = "NavigateToPose goal rejected"
            return response

        result_future = goal_handle.get_result_async()
        if not _wait_for_future(result_future, GOAL_TIMEOUT_SEC):
            goal_handle.cancel_goal_async()
            response.success = False
            response.message = f"navigation timed out after {GOAL_TIMEOUT_SEC:.0f}s"
            return response

        result = result_future.result()
        status = result.status
        if status == 4:  # SUCCEEDED
            response.success = True
            response.message = "ok"
        else:
            response.success = False
            response.message = f"NavigateToPose finished with status {status}"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoToPoseNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
