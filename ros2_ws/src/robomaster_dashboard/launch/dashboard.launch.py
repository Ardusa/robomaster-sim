"""Standalone dashboard launch. bringup.launch.py also starts this node."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _sim_default() -> str:
    value = os.environ.get("SIM", "").strip().lower()
    return value if value in ("true", "false") else "false"


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim",
                default_value=_sim_default(),
                choices=["true", "false"],
                description="Defaults to $SIM. Only affects use_sim_time.",
            ),
            DeclareLaunchArgument("port", default_value="8090"),
            DeclareLaunchArgument(
                "video_base",
                default_value="http://localhost:8080",
                description="Host-facing base URL for web_video_server streams.",
            ),
            Node(
                package="robomaster_dashboard",
                executable="dashboard_node",
                name="robomaster_dashboard",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("sim"), value_type=bool
                        ),
                        "port": ParameterValue(
                            LaunchConfiguration("port"), value_type=int
                        ),
                        "video_base": LaunchConfiguration("video_base"),
                    }
                ],
            ),
        ]
    )
