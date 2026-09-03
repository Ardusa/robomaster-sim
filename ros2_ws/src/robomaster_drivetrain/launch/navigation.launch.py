"""Launch Nav2 + go_to_pose for sim. Nav2 cmd_vel is remapped to /cmd_vel_autonomy."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def _spawn_pose(world_stem: str) -> tuple[float, float, float]:
    """Read map->odom static offset from the semantic map spawn pose."""
    command_pkg = get_package_share_directory("robomaster_command")
    map_path = os.path.join(command_pkg, "config", "semantic_maps", f"{world_stem}.yaml")
    if not os.path.isfile(map_path):
        return 0.0, 0.0, 0.0
    with open(map_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    spawn = data.get("spawn") or {}
    return (
        float(spawn.get("x", 0.0)),
        float(spawn.get("y", 0.0)),
        float(spawn.get("theta", 0.0)),
    )


def _launch_setup(context, *args, **kwargs):
    drivetrain_pkg = get_package_share_directory("robomaster_drivetrain")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    sim = LaunchConfiguration("sim").perform(context)
    use_sim_time = sim == "true"
    world_stem = LaunchConfiguration("world_stem").perform(context) or "modern_house"
    map_override = LaunchConfiguration("map").perform(context).strip()

    if map_override:
        map_yaml = map_override
    else:
        map_yaml = os.path.join(drivetrain_pkg, "config", "maps", f"{world_stem}.yaml")
        if not os.path.isfile(map_yaml):
            map_yaml = os.path.join(drivetrain_pkg, "config", "maps", "modern_house.yaml")

    nav2_params = os.path.join(drivetrain_pkg, "config", "nav2", "nav2_params.yaml")
    bringup_launch = os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
    spawn_x, spawn_y, spawn_yaw = _spawn_pose(world_stem)

    actions = [
        GroupAction(
            actions=[
                SetRemap(src="/cmd_vel", dst="/cmd_vel_autonomy"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(bringup_launch),
                    launch_arguments={
                        "map": map_yaml,
                        "params_file": nav2_params,
                        "use_sim_time": str(use_sim_time).lower(),
                        "autostart": "true",
                    }.items(),
                ),
            ]
        ),
        Node(
            package="robomaster_drivetrain",
            executable="go_to_pose_node.py",
            name="go_to_pose",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    if use_sim_time:
        # Map frame = Gazebo world. Odom starts at spawn with base_link at the
        # origin, so map->odom is fixed at the spawn pose for perfect sim odom.
        actions.insert(
            0,
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_odom_broadcaster",
                output="screen",
                arguments=[
                    str(spawn_x),
                    str(spawn_y),
                    "0",
                    str(spawn_yaw),
                    "0",
                    "0",
                    "map",
                    "odom",
                ],
                parameters=[{"use_sim_time": True}],
            ),
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("sim", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument(
                "world_stem",
                default_value=os.path.splitext(
                    os.environ.get("WORLD", "modern_house.sdf").strip()
                )[0]
                or "modern_house",
                description="Basename of config/maps/<world_stem>.yaml",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Optional occupancy-grid yaml (overrides world_stem).",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
