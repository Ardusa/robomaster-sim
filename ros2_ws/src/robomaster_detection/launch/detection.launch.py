"""COCO object detection + debug overlay.

Interprets pixels; does not acquire them. Runs against whatever publishes
/camera/image_raw — robomaster_camera on the real robot, the Gazebo sensor
in sim — and cannot tell the two apart.

`make bringup` already includes this; run it standalone to restart detection
without restarting the robot.

    /camera/image_raw --> object_detector --> /object_detections
            |                                        |
            +--------> detection_overlay <-----------+
                              |
                    /camera/image_annotated
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _sim_default() -> str:
    """SIM from the environment, so this is runnable standalone.

    bringup.launch.py passes sim:= explicitly; when it doesn't (you ran this on
    its own), fall back to the same env var rather than guessing a backend. Only
    use_sim_time depends on it, so an unset SIM is a warning, not a hard failure
    like it is in bringup — detection still works either way, the clock is just
    wrong.
    """
    value = os.environ.get("SIM", "").strip().lower()
    return value if value in ("true", "false") else "false"


def generate_launch_description():
    pkg = get_package_share_directory("robomaster_detection")
    detector_config = os.path.join(pkg, "config", "object_detector.yaml")

    sim = LaunchConfiguration("sim")
    use_sim_time = PythonExpression(["'", sim, "' == 'true'"])

    detector = Node(
        package="robomaster_detection",
        executable="object_detector_node",
        name="object_detector",
        output="screen",
        parameters=[detector_config, {"use_sim_time": use_sim_time}],
    )

    overlay = Node(
        package="robomaster_detection",
        executable="detection_overlay_node",
        name="detection_overlay",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim",
                default_value=_sim_default(),
                choices=["true", "false"],
                description="Defaults to $SIM. Only affects use_sim_time.",
            ),
            detector,
            overlay,
        ]
    )
