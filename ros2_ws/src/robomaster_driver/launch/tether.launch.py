"""The physical-robot half of the tether backend.

Who owns the control port (one client only):

    control=true   HardwareInterface holds it (wheels + SDK arm/gripper services).
    control=false, arm=true
                   sdk_bridge_node holds it (arm/gripper + optional stream on).
    control=false, arm=false, camera=true
                   camera_node arms the stream itself.

Before running: robot powered on, direct-connection mode, ROBOMASTER_IP in .env.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context, *args, **kwargs):
    camera_pkg = get_package_share_directory("robomaster_camera")
    control = LaunchConfiguration("control").perform(context) == "true"
    camera = LaunchConfiguration("camera").perform(context) == "true"
    arm = LaunchConfiguration("arm").perform(context) == "true"

    actions = []

    if control:
        actions.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[LaunchConfiguration("controllers_file")],
                output="screen",
            )
        )
    elif arm:
        # No wheels — still need one TCP owner for arm/gripper (and video).
        actions.append(
            Node(
                package="robomaster_driver",
                executable="sdk_bridge_node",
                name="robomaster_sdk_bridge",
                output="screen",
                parameters=[
                    {
                        "enable_video": camera,
                        "robot_ip": os.environ.get("ROBOMASTER_IP", ""),
                    }
                ],
            )
        )

    if camera:
        # Arm stream here only when nobody else holds the control port.
        arm_stream = not control and not arm
        start_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(camera_pkg, "launch", "camera.launch.py")
            ),
            launch_arguments={"arm_stream": str(arm_stream).lower()}.items(),
        )
        delay = 5.0 if (control or arm) else 0.0
        actions.append(
            TimerAction(period=delay, actions=[start_camera]) if delay else start_camera
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_file",
                default_value="",
                description=(
                    "controller_manager params from robomaster_drivetrain; "
                    "bringup passes the path."
                ),
            ),
            DeclareLaunchArgument("control", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("camera", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("arm", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("sim", default_value="false", choices=["true", "false"]),
            OpaqueFunction(function=_nodes),
        ]
    )
