"""The entry point. Composes subsystems; does not implement them.

    SIM=true   -> Gazebo + gz_ros2_control
    SIM=false  -> the physical EP over its plaintext SDK

Either way you get the same TF tree and the same subsystem interfaces. Gazebo
or tether is always the foundational backend when SIM is set — not a separate
debug target.

    make bringup             # full stack + web dashboard
    make bringup-teleop      # drivetrain + arm + keyboard teleop fallback
    make bringup-detection   # camera + detection

Keyboard teleop is deliberately NOT a node here: it needs a real TTY.
bringup-teleop runs it in the foreground; full bringup uses the dashboard.

SIM is read from the environment (set it in .env) and has no default.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _headless_default() -> str:
    """Headless unless GUI=true in .env — the dashboard is the normal viewport.

    Same pattern as SIM/WORLD: session preference lives in .env, launch
    headless:= still overrides for one-offs. The Makefile forces headless:=true
    on hosts with no display path (Mac, Windows) regardless of this.
    """
    gui = os.environ.get("GUI", "").strip().lower()
    return "false" if gui == "true" else "true"


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

    control = flag("control")
    camera = flag("camera")
    detection = flag("detection")
    arm = flag("arm")
    dashboard = flag("dashboard")
    command = flag("command")

    # Detection needs a feed; on the real robot that means the camera node.
    if detection and sim == "false":
        camera = True

    def include(pkg, launch_file, **launch_args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(pkg), "launch", launch_file)
            ),
            launch_arguments={**launch_args, "sim": sim}.items(),
        )

    actions = [include("robomaster_bringup", "description.launch.py")]

    if control:
        actions.append(include("robomaster_drivetrain", "control.launch.py"))

    if control and sim == "true":
        world = LaunchConfiguration("world").perform(context)
        world_stem = os.path.splitext(os.path.basename(world))[0] if world else "modern_house"
        actions.append(
            TimerAction(
                period=12.0,
                actions=[
                    include(
                        "robomaster_drivetrain",
                        "navigation.launch.py",
                        world_stem=world_stem,
                    )
                ],
            )
        )

    if arm:
        actions.append(include("robomaster_arm", "arm.launch.py"))

    if sim == "true":
        # Foundational: physics + sim camera whenever SIM=true.
        sim_args = {"headless": LaunchConfiguration("headless")}
        world = LaunchConfiguration("world").perform(context)
        if world:
            sim_args["world"] = world
        actions.append(include("robomaster_gazebo", "sim.launch.py", **sim_args))
    else:
        # Foundational tether: wheels and/or SDK bridge and/or camera.
        actions.append(
            include(
                "robomaster_tether",
                "tether.launch.py",
                control=str(control).lower(),
                camera=str(camera).lower(),
                arm=str(arm).lower(),
                controllers_file=os.path.join(
                    drivetrain_pkg, "config", "tether_controllers.yaml"
                ),
            )
        )

    if detection:
        actions.append(include("robomaster_detection", "detection.launch.py"))

    if command:
        # Same world as Gazebo so zone coords match the loaded scenery.
        cmd_args = {}
        world = LaunchConfiguration("world").perform(context)
        if world:
            cmd_args["world"] = world
        actions.append(include("robomaster_command", "command.launch.py", **cmd_args))

    want_video = flag("video_server") and (camera or detection or dashboard)
    if want_video:
        # bringup-teleop stays quiet on :8080; dashboard needs MJPEG streams.
        actions.append(
            Node(
                package="web_video_server",
                executable="web_video_server",
                name="web_video_server",
                output="log",
                arguments=["--ros-args", "--log-level", "warn"],
                parameters=[{"port": 8080, "use_sim_time": sim == "true"}],
            )
        )

    if dashboard:
        actions.append(include("robomaster_dashboard", "dashboard.launch.py"))

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
                "arm",
                default_value="true",
                choices=["true", "false"],
                description="Include robomaster_arm (Cartesian actions + sim controllers).",
            ),
            DeclareLaunchArgument(
                "camera",
                default_value="true",
                choices=["true", "false"],
                description="Camera feed. Forced on when detection:=true and SIM=false.",
            ),
            DeclareLaunchArgument(
                "detection",
                default_value="true",
                choices=["true", "false"],
                description="COCO object detection + overlay.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value=_headless_default(),
                choices=["true", "false"],
                description=(
                    "Gazebo with no GUI. Defaults from $GUI in .env "
                    "(GUI=true -> headless=false). Ignored when SIM=false."
                ),
            ),
            DeclareLaunchArgument(
                "world",
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
            DeclareLaunchArgument(
                "dashboard",
                default_value="false",
                choices=["true", "false"],
                description="Operator web UI on :8090 (chassis + arm teleop).",
            ),
            DeclareLaunchArgument(
                "command",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Natural-language command grounding + translator "
                    "(robomaster_command). Needed by the dashboard Command panel."
                ),
            ),
            OpaqueFunction(function=_backends),
        ]
    )
