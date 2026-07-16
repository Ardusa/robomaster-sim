"""The physical-robot half of the tether backend: controller_manager against
the hardware interface, plus the camera. Nothing here is shared with the sim.

Not an entry point — bringup.launch.py includes this when SIM=false and passes
control/camera down, so a subsystem can be tested on its own.

Who arms the video stream depends on that split. "stream on" goes over the
control port, which takes one client at a time:

    control=true   the hardware interface holds that port all session, so it
                   arms the stream and the camera only reads the video port.
    control=false  no driver, so nothing else can arm it — the camera opens the
                   control port itself.

Before running: robot powered on, in direct-connection mode, this machine on
its Wi-Fi hotspot, ROBOMASTER_IP set in .env. Run `make test-connection` first —
it's a much faster failure signal than the full launch stack.
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

    actions = []

    if control:
        # Params passed down by bringup rather than $(find robomaster_bringup)'d
        # here: bringup includes this file, so reaching back into it would make
        # the two packages depend on each other.
        actions.append(
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[LaunchConfiguration("controllers_file")],
                output="screen",
            )
        )

    if camera:
        start_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(camera_pkg, "launch", "camera.launch.py")),
            launch_arguments={"arm_stream": str(not control).lower()}.items(),
        )
        # When the driver is arming the stream, the video port has nothing to
        # connect to until it has activated. On its own, the camera arms the
        # stream itself and can start immediately.
        actions.append(TimerAction(period=5.0, actions=[start_camera]) if control else start_camera)

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_file",
                default_value="",
                description="controller_manager params (bringup owns it).",
            ),
            DeclareLaunchArgument("control", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("camera", default_value="true", choices=["true", "false"]),
            # Accepted and ignored: bringup passes sim to every include.
            DeclareLaunchArgument("sim", default_value="false", choices=["true", "false"]),
            OpaqueFunction(function=_nodes),
        ]
    )
