"""Grounding + translator for natural-language robot commands."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _default_world() -> str:
    # Same WORLD session knob as robomaster_gazebo/sim.launch.py.
    env = os.environ.get("WORLD", "").strip()
    if env:
        return env
    return os.path.join(
        get_package_share_directory("robomaster_gazebo"),
        "worlds",
        "robot_only.sdf",
    )


def _default_sim() -> str:
    return "true" if (os.environ.get("SIM") or "").strip().lower() == "true" else "false"


def _resolve_world(context) -> str:
    gazebo_share = get_package_share_directory("robomaster_gazebo")
    world = LaunchConfiguration("world").perform(context) or _default_world()
    worlds_dir = os.path.join(gazebo_share, "worlds")
    if not os.path.isabs(world):
        shipped = os.path.join(worlds_dir, world)
        if not os.path.isfile(shipped):
            available = sorted(
                entry.name for entry in os.scandir(worlds_dir) if entry.is_file()
            )
            raise RuntimeError(
                f"world '{world}' is not one of the worlds robomaster_gazebo ships "
                f"({', '.join(available)}). Set WORLD in .env to one of those, or "
                f"pass an absolute path."
            )
        world = shipped
    return world


def _nodes(context, *args, **kwargs):
    world = _resolve_world(context)
    stem = os.path.splitext(os.path.basename(world))[0]
    share = get_package_share_directory("robomaster_command")
    map_path = os.path.join(share, "config", "semantic_maps", f"{stem}.yaml")
    if not os.path.isfile(map_path):
        maps_dir = os.path.join(share, "config", "semantic_maps")
        available = sorted(
            entry.name
            for entry in os.scandir(maps_dir)
            if entry.is_file() and entry.name.endswith(".yaml")
        )
        raise RuntimeError(
            f"no semantic map for world '{stem}' at {map_path}. "
            f"Available maps: {', '.join(available) or '(none)'}. "
            f"Add config/semantic_maps/{stem}.yaml or set WORLD to a mapped world."
        )

    sim = LaunchConfiguration("sim")
    use_sim_time = PythonExpression(["'", sim, "' == 'true'"])

    return [
        Node(
            package="robomaster_command",
            executable="grounding_node.py",
            name="robomaster_command_grounding",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="robomaster_command",
            executable="translator_node.py",
            name="robomaster_command_translator",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "semantic_map_path": map_path,
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim",
                default_value=_default_sim(),
                choices=["true", "false"],
                description="Propagate use_sim_time when SIM=true / Gazebo is the clock.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=_default_world(),
                description=(
                    "Gazebo world file (absolute or bare name under "
                    "robomaster_gazebo/worlds). Semantic map is "
                    "config/semantic_maps/<world_stem>.yaml — must exist."
                ),
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
