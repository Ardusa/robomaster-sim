"""Cartesian arm + gripper. Sim spawns position controllers; tether uses SDK."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context, *args, **kwargs):
    sim = LaunchConfiguration("sim").perform(context) == "true"
    arm_pkg = get_package_share_directory("robomaster_arm")
    param_file = os.path.join(arm_pkg, "config", "sim_arm_controllers.yaml")

    actions = [
        Node(
            package="robomaster_arm",
            executable="arm_node.py",
            name="robomaster_arm",
            output="screen",
            parameters=[{"sim": sim, "use_sim_time": sim}],
        )
    ]

    if sim:
        actions.extend(
            [
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "arm_controller",
                        "--controller-manager",
                        "/controller_manager",
                        "--param-file",
                        param_file,
                    ],
                    output="screen",
                    parameters=[{"use_sim_time": True}],
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "gripper_controller",
                        "--controller-manager",
                        "/controller_manager",
                        "--param-file",
                        param_file,
                    ],
                    output="screen",
                    parameters=[{"use_sim_time": True}],
                ),
            ]
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim",
                default_value="true",
                choices=["true", "false"],
                description="Gazebo serial arm vs tether SDK bridge.",
            ),
            OpaqueFunction(function=_nodes),
        ]
    )
