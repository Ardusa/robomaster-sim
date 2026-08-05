"""The Gazebo half of the sim backend: Gazebo itself, the robot spawn, and the
bridges. Nothing here is shared with the real robot.

Not an entry point — bringup.launch.py includes this when SIM=true, alongside
the description and control layers. Launching it alone gives you a robot in a
world with no controllers.

No ros2_control_node: the URDF's gz_ros2_control plugin loads
controller_manager inside the Gazebo process, so it exists only once the robot
is spawned. bringup's spawners wait for it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_world() -> str:
    # Session choice lives in .env (WORLD=...), same pattern as SIM. Launch
    # world:= still overrides for one-offs.
    env = os.environ.get("WORLD", "").strip()
    if env:
        return env
    return os.path.join(
        get_package_share_directory("robomaster_gazebo"), "worlds", "robot_only.sdf"
    )


def _gz(context, *args, **kwargs):
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gazebo_share = get_package_share_directory("robomaster_gazebo")

    # Launch propagates configurations into included descriptions, so bringup's
    # own world argument shadows the default declared below and an unset world
    # arrives here as "" — hence the fallback rather than trusting the default.
    world = LaunchConfiguration("world").perform(context) or _default_world()

    # A bare name is a convenience for the worlds we ship. Anything we don't
    # have has to fall through untouched so Gazebo can resolve its own builtins
    # (empty.sdf) rather than being turned into a path that doesn't exist. isfile,
    # not exists: an empty name would otherwise "resolve" to worlds/ itself.
    if not os.path.isabs(world):
        shipped = os.path.join(gazebo_share, "worlds", world)
        if os.path.isfile(shipped):
            world = shipped

    # Engines are per-process, not global: the GUI's Ogre 1.x path aborts on an
    # AxisAlignedBox assertion inside its render thread, which is a black window
    # and an empty entity tree even on empty.sdf, so the GUI needs ogre2. The
    # server stays on ogre, the offscreen path the camera is known to render
    # sensors through with no GPU passthrough in the container.
    gz_args = f"-r --render-engine-server ogre --render-engine-gui ogre2 {world}"

    # headless:=true runs the server with no GUI, which is the only way this is
    # bearable without GPU passthrough (see the Makefile's platform warning).
    # Sensors still render offscreen, so the camera works either way.
    if LaunchConfiguration("headless").perform(context) == "true":
        gz_args += " -s --headless-rendering"

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args}.items(),
        )
    ]


def generate_launch_description():
    gazebo_share = get_package_share_directory("robomaster_gazebo")
    description_share = get_package_share_directory("robomaster_description")

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "robomaster_ep", "-z", "0.1"],
        output="screen",
    )

    # Gazebo publishes the camera on its own transport; these bridge it onto
    # the ROS topics apriltag_node reads. Names match camera_node.py's, so
    # detection doesn't care which backend is running.
    camera_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument(
                "world",
                default_value=_default_world(),
                description=(
                    "Gazebo world file: an absolute path, or a bare name resolved "
                    "against this package's worlds/."
                ),
            ),
            # Accepted and ignored: bringup passes sim to every include.
            DeclareLaunchArgument("sim", default_value="true", choices=["true", "false"]),
            # model:// resolves against each entry directly.
            AppendEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=os.pathsep.join(
                    [description_share, gazebo_share, os.path.join(gazebo_share, "models")]
                ),
            ),
            OpaqueFunction(function=_gz),
            clock_bridge,
            TimerAction(period=4.0, actions=[spawn]),  # let Gazebo come up first
            TimerAction(period=8.0, actions=[camera_bridge, camera_info_bridge]),
        ]
    )
