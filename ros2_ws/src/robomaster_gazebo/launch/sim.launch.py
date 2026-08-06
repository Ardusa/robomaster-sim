"""The Gazebo half of the sim backend: Gazebo itself, the robot spawn, and the
bridges. Nothing here is shared with the real robot.

Not an entry point — bringup.launch.py includes this when SIM=true, alongside
the description and control layers. Launching it alone gives you a robot in a
world with no controllers.

No ros2_control_node: the URDF's gz_ros2_control plugin loads
controller_manager inside the Gazebo process, so it exists only once the robot
is spawned. bringup's spawners wait for it.
"""

import glob
import math
import os
import subprocess
import tempfile
from xml.etree import ElementTree

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
import xacro


IGNITION_XMLNS = "http://gazebosim.org/schema"


def _default_world() -> str:
    # Session choice lives in .env (WORLD=...), same pattern as SIM. Launch
    # world:= still overrides for one-offs.
    env = os.environ.get("WORLD", "").strip()
    if env:
        return env
    return os.path.join(
        get_package_share_directory("robomaster_gazebo"), "worlds", "robot_only.sdf"
    )


def _resolve_world(context) -> str:
    gazebo_share = get_package_share_directory("robomaster_gazebo")

    # Launch propagates configurations into included descriptions, so bringup's
    # own world argument shadows the default declared below and an unset world
    # arrives here as "" — hence the fallback rather than trusting the default.
    world = LaunchConfiguration("world").perform(context) or _default_world()

    # A bare name is resolved against the worlds we ship, and a miss fails here
    # naming itself — same reasoning as SIM. A typo in .env would otherwise reach
    # Gazebo and abort with its own less obvious message, and a Gazebo builtin
    # like empty.sdf would come up declaring no Sensors system, which is a camera
    # that silently never renders. Absolute paths are trusted as deliberate.
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
                f"pass an absolute path to use a world from somewhere else."
            )
        world = shipped

    return world


def _spawn_pose(world: str) -> list:
    """Spawn coordinates from the world's robot_spawn frame, else the origin.

    A world knows where its own floor is clear; the origin is furniture in one of
    ours. Declaring it in the world keeps that with the geometry it depends on,
    rather than in a launch argument that has to be re-set per world.

    A world that declares no such frame falls back to the origin, but one that
    cannot be parsed is an error. Gazebo's XML parser is looser than Python's and
    will happily load a world this cannot read — silently dropping the robot at
    the origin, which is inside the furniture in at least one of our worlds.
    """
    try:
        root = ElementTree.parse(world).getroot()
    except (ElementTree.ParseError, OSError) as exc:
        raise RuntimeError(
            f"could not parse world '{world}' to find its robot_spawn frame: {exc}. "
            f"Gazebo may still load it, so check for XML that only a strict parser "
            f"rejects (a '--' inside a comment is the usual culprit)."
        ) from exc

    pose = root.find("./world/frame[@name='robot_spawn']/pose")
    if pose is None or not pose.text:
        return ["0", "0", "0.1", "0"]

    parts = pose.text.split()
    if len(parts) != 6:
        raise RuntimeError(
            f"world '{world}' has a robot_spawn frame whose pose is "
            f"'{pose.text.strip()}'; it needs six values, 'x y z roll pitch yaw'."
        )
    x, y, z, _roll, _pitch, yaw = parts
    return [x, y, z, yaw]


def _gui_config(world: str) -> str:
    """A gui.config whose opening viewpoint frames the world's spawn point.

    Worlds cannot do this themselves: Fortress ignores the classic
    <gui><camera> pose, and the MinimalScene plugin that does honour a
    camera_pose replaces Gazebo's entire default GUI when it appears in a world,
    which costs the entity tree and the run controls. So Gazebo's own installed
    default is patched instead, leaving every other panel exactly as shipped and
    staying correct across Gazebo versions rather than vendoring a copy.

    Returns "" when the default cannot be found, in which case the caller leaves
    Gazebo to its own configuration.
    """
    default = next(iter(sorted(
        glob.glob("/usr/share/ignition/ignition-gazebo*/gui/gui.config"))), "")
    if not default:
        return ""

    x, y, z, yaw = (float(v) for v in _spawn_pose(world))
    # Behind and above the robot, looking down its own heading, so the view
    # starts on the robot with whatever it faces beyond it.
    back, up = 5.0, 3.5
    camera = (
        f"{x - back * math.cos(yaw):.4g} {y - back * math.sin(yaw):.4g} {z + up:.4g} "
        f"0 {math.atan2(up, back):.4g} {yaw:.4g}"
    )

    try:
        text = open(default, encoding="utf-8").read()
    except OSError:
        return ""

    start = text.find("<camera_pose>")
    if start < 0:
        return ""
    end = text.index("</camera_pose>", start) + len("</camera_pose>")
    patched = text[:start] + f"<camera_pose>{camera}</camera_pose>" + text[end:]

    handle, path = tempfile.mkstemp(prefix="robomaster_gui_", suffix=".config")
    with os.fdopen(handle, "w", encoding="utf-8") as out:
        out.write(patched)
    return path


def _sim_model_file() -> str:
    """Convert the shared URDF to SDF and add mecanum contact physics.

    Fortress' URDF converter drops directional-friction extensions from this
    model after fixed-joint reduction. Patching the generated SDF is explicit,
    testable, and sim-only; robot_state_publisher and the tether backend keep
    consuming the ordinary shared URDF.
    """
    description_share = get_package_share_directory("robomaster_description")
    drivetrain_share = get_package_share_directory("robomaster_drivetrain")
    urdf = xacro.process_file(
        os.path.join(description_share, "urdf", "robomaster_ep.urdf.xacro"),
        mappings={
            "sim": "true",
            "robot_ip": "",
            "sim_controllers_file": os.path.join(
                drivetrain_share, "config", "sim_controllers.yaml"
            ),
        },
    ).toxml()

    urdf_handle, urdf_path = tempfile.mkstemp(
        prefix="robomaster_sim_", suffix=".urdf"
    )
    with os.fdopen(urdf_handle, "w", encoding="utf-8") as out:
        out.write(urdf)

    try:
        converted = subprocess.run(
            ["ign", "sdf", "-p", urdf_path],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError(f"could not convert robot URDF to sim SDF: {exc}") from exc
    finally:
        os.unlink(urdf_path)

    root = ElementTree.fromstring(converted)
    # Same X roller pattern as Gazebo's mecanum_drive example. mu is friction
    # along the roller axis; mu2=0 allows free motion perpendicular to it.
    roller_directions = {
        "front_left_wheel_link": "1 -1 0",
        "front_right_wheel_link": "1 1 0",
        "rear_left_wheel_link": "1 1 0",
        "rear_right_wheel_link": "1 -1 0",
    }
    ElementTree.register_namespace("ignition", IGNITION_XMLNS)
    for link_name, direction in roller_directions.items():
        collision = root.find(f".//link[@name='{link_name}']/collision")
        if collision is None:
            raise RuntimeError(
                f"generated sim SDF has no collision for '{link_name}'"
            )
        surface = collision.find("surface")
        if surface is None:
            surface = ElementTree.SubElement(collision, "surface")
        friction = surface.find("friction")
        if friction is None:
            friction = ElementTree.SubElement(surface, "friction")
        ode = friction.find("ode")
        if ode is None:
            ode = ElementTree.SubElement(friction, "ode")
        ElementTree.SubElement(ode, "mu").text = "1.0"
        ElementTree.SubElement(ode, "mu2").text = "0.0"
        fdir1 = ElementTree.SubElement(ode, "fdir1")
        fdir1.set(f"{{{IGNITION_XMLNS}}}expressed_in", "base_link")
        fdir1.text = direction

    sdf_handle, sdf_path = tempfile.mkstemp(
        prefix="robomaster_sim_", suffix=".sdf"
    )
    with os.fdopen(sdf_handle, "wb") as out:
        ElementTree.ElementTree(root).write(
            out, encoding="utf-8", xml_declaration=True
        )
    return sdf_path


def _spawn(context, *args, **kwargs):
    x, y, z, yaw = _spawn_pose(_resolve_world(context))
    return [
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-file", _sim_model_file(),
                "-name", "robomaster_ep",
                "-x", x, "-y", y, "-z", z, "-Y", yaw,
            ],
            output="screen",
        )
    ]


def _gz(context, *args, **kwargs):
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    world = _resolve_world(context)

    # Engines are per-process, not global: the GUI's Ogre 1.x path aborts on an
    # AxisAlignedBox assertion inside its render thread, which is a black window
    # and an empty entity tree even on empty.sdf, so the GUI needs ogre2. The
    # server stays on ogre, the offscreen path the camera is known to render
    # sensors through with no GPU passthrough in the container.
    gz_args = "-r --render-engine-server ogre --render-engine-gui ogre2"

    # headless:=true runs the server with no GUI, which is the only way this is
    # bearable without GPU passthrough (see the Makefile's platform warning).
    # Sensors still render offscreen, so the camera works either way.
    if LaunchConfiguration("headless").perform(context) == "true":
        gz_args += " -s --headless-rendering"
    else:
        gui_config = _gui_config(world)
        if gui_config:
            gz_args += f" --gui-config {gui_config}"

    gz_args += f" {world}"

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args}.items(),
        )
    ]


def generate_launch_description():
    gazebo_share = get_package_share_directory("robomaster_gazebo")
    description_share = get_package_share_directory("robomaster_description")

    # Gazebo publishes cameras on its own transport; these bridge them onto
    # ROS topics. Robot cam names match camera_node.py so detection can't tell
    # backends apart. /camera/overview is the dashboard top-down feed.
    camera_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw", "/camera/overview"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
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
            # Let Gazebo come up first; the spawn pose comes from the world.
            TimerAction(period=4.0, actions=[OpaqueFunction(function=_spawn)]),
            TimerAction(period=8.0, actions=[camera_bridge, camera_info_bridge]),
        ]
    )
