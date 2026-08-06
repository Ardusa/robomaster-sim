#!/usr/bin/env python3
"""Prompt -> ActionPrimitive[] service. Swappable generator lives in command_generator."""

from __future__ import annotations

import os
import sys

# Sibling module installed next to this script in lib/robomaster_command.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node

import command_generator
from robomaster_command.srv import GenerateActionSequence


class GroundingNode(Node):
    def __init__(self) -> None:
        super().__init__("robomaster_command_grounding")
        self.declare_parameter("use_sim_time", False)
        self.create_service(
            GenerateActionSequence,
            "~/generate_action_sequence",
            self._on_generate,
        )
        self.get_logger().info("grounding ready on ~/generate_action_sequence")

    def _on_generate(
        self,
        request: GenerateActionSequence.Request,
        response: GenerateActionSequence.Response,
    ) -> GenerateActionSequence.Response:
        try:
            actions = command_generator.generate_action_sequence(request.prompt)
            types = [a.type for a in actions]
            self.get_logger().info(
                f"prompt={request.prompt!r} -> primitives={types}"
            )
            response.actions = actions
            response.success = True
            response.message = ""
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"generate failed: {exc}")
            response.actions = []
            response.success = False
            response.message = str(exc)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
