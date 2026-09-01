#!/usr/bin/env python3
"""Prompt -> ActionPrimitive[] service. Swappable generator lives in command_generator."""

from __future__ import annotations

import os
import sys

# Sibling module installed next to this script in lib/robomaster_command.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
import yaml
from rclpy.node import Node

import command_generator
from robomaster_command.srv import GenerateActionSequence


class GroundingNode(Node):
    def __init__(self) -> None:
        super().__init__("robomaster_command_grounding")
        self.declare_parameter("semantic_map_path", "")

        map_path = self.get_parameter("semantic_map_path").get_parameter_value().string_value
        loaded = self._load_semantic_map(map_path)
        if loaded is None:
            self._zones = None
            self._restricted = None
            return
        self._zones, self._restricted = loaded

        self.create_service(
            GenerateActionSequence,
            "~/generate_action_sequence",
            self._on_generate,
        )
        self.get_logger().info(
            f"grounding ready on ~/generate_action_sequence; "
            f"zones={sorted(self._zones)} restricted={self._restricted}"
        )

    def _load_semantic_map(
        self, map_path: str
    ) -> tuple[dict[str, dict[str, float]], list[str]] | None:
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

        restricted_raw = data.get("restricted", []) if isinstance(data, dict) else []
        if not isinstance(restricted_raw, list):
            self.get_logger().error(
                f"semantic map {map_path}: 'restricted' must be a list"
            )
            return None
        restricted = [str(name) for name in restricted_raw]

        return zones, restricted

    def _on_generate(
        self,
        request: GenerateActionSequence.Request,
        response: GenerateActionSequence.Response,
    ) -> GenerateActionSequence.Response:
        try:
            actions = command_generator.generate_action_sequence(
                request.prompt, self._zones, self._restricted
            )
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
    if node._zones is None:
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
