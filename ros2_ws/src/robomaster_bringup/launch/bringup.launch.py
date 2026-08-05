"""The entry point. Composes subsystems; does not implement them.

    SIM=true   -> Gazebo + gz_ros2_control
    SIM=false  -> the physical EP over its plaintext SDK

Either way you get the same TF tree, the same mecanum controller (via
robomaster_drivetrain), the same /cmd_vel_teleop and /cmd_vel_autonomy inputs,
and the same AprilTag topics. The backend is the only thing that changes.

The control/camera/detection args exist so a subsystem can be brought up on its
own, which is what the make targets use to test one thing at a time:

    make bringup             # everything (control + camera + detection)
    make bringup-teleop      # control only, then hands you the keyboard
    make bringup-camera      # camera only — is the camera alive?
    make bringup-detection   # camera + detection — are tags being found?

Keyboard teleop is deliberately NOT a node here: teleop_twist_keyboard reads raw
stdin, and a launch child process has no terminal, so it would capture no keys.
Run robomaster_teleop in its own foreground shell — see the Makefile.

SIM is read from the environment (set it in .env) and has no default: an unset
or misspelled value fails here, naming itself, rather than silently booting the
wrong backend. Same reasoning as ROBOMASTER_IP, which description.launch.py
requires when SIM=false.

Video is served over HTTP on :8080, not through an X11 GUI — watch it in a
browser.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _sim_from_env() -> str:
    raw = os.environ.get("SIM")
    if raw is None or raw == "":
        raise RuntimeError(
            "SIM is not set. Set it in .env: SIM=true for Gazebo, SIM=false for "
            "the physical robot (which also needs ROBOMASTER_IP)."
        )
    value = raw.strip().lower()
    if value not in ("true", "false"):
        raise RuntimeError(f"SIM must be 'true' or 'false', got '{raw}'.")
    return value


def _backends(context, *args, **kwargs):
    sim = _sim_from_env()
    drivetrain_pkg = get_package_share_directory("robomaster_drivetrain")

    def flag(name):
        return LaunchConfiguration(name).perform(context) == "true"

    control, camera, detection = flag("control"), flag("camera"), flag("detection")

    def include(pkg, launch_file, **launch_args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(pkg), "launch", launch_file)
            ),
            launch_arguments={**launch_args, "sim": sim}.items(),
        )

    # Always: everything downstream needs the URDF and the TF tree.
    actions = [include("robomaster_bringup", "description.launch.py")]

    if control:
        actions.append(include("robomaster_drivetrain", "control.launch.py"))

    if sim == "true":
        # Gazebo is the sim's camera *and* its physics, so it comes up either
        # way — camera-only just means no controllers are spawned against it.
        sim_args = {"headless": LaunchConfiguration("headless")}
        # Empty means "let sim.launch.py pick its default": passing the empty
        # string through would hand Gazebo a world named "".
        world = LaunchConfiguration("world").perform(context)
        if world:
            sim_args["world"] = world
        actions.append(include("robomaster_gazebo", "sim.launch.py", **sim_args))
    else:
        # Path injected here so driver does not find bringup (or cycle).
        # Controllers YAML is owned by drivetrain.
        actions.append(
            include(
                "robomaster_driver",
                "tether.launch.py",
                control=str(control).lower(),
                camera=str(camera).lower(),
                controllers_file=os.path.join(
                    drivetrain_pkg, "config", "tether_controllers.yaml"
                ),
            )
        )

    if detection:
        actions.append(include("robomaster_detection", "detection.launch.py"))

    # Idle until something opens a stream, and it's the only way to see video on
    # a Mac (XQuartz can't render rqt here — GL has no working driver inside the
    # emulated x86 image). Browse http://localhost:8080.
    if flag("video_server") and (camera or detection):
        actions.append(
            Node(
                package="web_video_server",
                executable="web_video_server",
                name="web_video_server",
                output="screen",
                parameters=[{"port": 8080, "use_sim_time": sim == "true"}],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "control",
                default_value="true",
                choices=["true", "false"],
                description="Include robomaster_drivetrain (controllers + twist mux).",
            ),
            DeclareLaunchArgument(
                "camera",
                default_value="true",
                choices=["true", "false"],
                description="Camera feed. Ignored when SIM=true (Gazebo owns it).",
            ),
            DeclareLaunchArgument(
                "detection",
                default_value="true",
                choices=["true", "false"],
                description="AprilTag detection + overlay. Implies camera.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                choices=["true", "false"],
                description="Gazebo with no GUI. Ignored when SIM=false.",
            ),
            DeclareLaunchArgument(
                "world",
                # Same route SIM takes: set it in .env, which compose passes into
                # the container. An explicit world:= still wins, for a one-off.
                default_value=os.environ.get("WORLD", ""),
                description=(
                    "Gazebo world file, absolute or bare name under "
                    "robomaster_gazebo/worlds. Empty uses $WORLD from .env "
                    "(then robot_only.sdf). Ignored when SIM=false."
                ),
            ),
            DeclareLaunchArgument(
                "video_server",
                default_value="true",
                choices=["true", "false"],
                description="Serve the camera topics over HTTP on :8080.",
            ),
            OpaqueFunction(function=_backends),
        ]
    )
