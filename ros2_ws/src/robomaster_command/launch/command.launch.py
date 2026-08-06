"""Grounding + translator for natural-language robot commands."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_semantic_map() -> str:
    # Same WORLD session knob as sim.launch.py / .env. Bare name or *.sdf
    # both resolve to config/semantic_maps/<stem>.yaml.
    world = os.environ.get("WORLD", "").strip() or "modern_house.sdf"
    stem = os.path.splitext(os.path.basename(world))[0]
    return f"{stem}.yaml"


def _nodes(context, *args, **kwargs):
    semantic_map = LaunchConfiguration("semantic_map").perform(context)
    share = get_package_share_directory("robomaster_command")
    map_path = os.path.join(share, "config", "semantic_maps", semantic_map)

    return [
        Node(
            package="robomaster_command",
            executable="grounding_node.py",
            name="robomaster_command_grounding",
            output="screen",
        ),
        Node(
            package="robomaster_command",
            executable="translator_node.py",
            name="robomaster_command_translator",
            output="screen",
            parameters=[{"semantic_map_path": map_path}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "semantic_map",
                default_value=_default_semantic_map(),
                description=(
                    "Semantic map YAML under "
                    "share/robomaster_command/config/semantic_maps/. "
                    "Defaults from $WORLD (e.g. modern_house.sdf -> "
                    "modern_house.yaml)."
                ),
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
