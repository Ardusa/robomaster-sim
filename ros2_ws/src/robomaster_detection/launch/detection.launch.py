"""AprilTag detection + debug overlay.

Interprets pixels; does not acquire them. Runs against whatever publishes
/camera/image_raw and /camera/camera_info — robomaster_camera on the real
robot, the Gazebo sensor in sim — and cannot tell the two apart.

`make bringup` already includes this; run it standalone to restart detection
without restarting the robot.

    /camera/image_raw --> rectify --> /camera/image_rect --> apriltag_node
                                                                  |
                                                            /detections + /tf
                                                                  |
                                       /camera/image_raw --> tag_overlay -->
                                                       /camera/image_annotated

rectify is not optional: apriltag_node needs a rectified image for pose to be
right. In sim it's an identity pass (no distortion) but stays in the graph so
both backends run the same nodes.
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
    tags_config = os.path.join(pkg, "config", "tags_36h11.yaml")

    sim = LaunchConfiguration("sim")
    use_sim_time = PythonExpression(["'", sim, "' == 'true'"])

    rectify = Node(
        package="image_proc",
        executable="rectify_node",
        name="rectify",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("image", "/camera/image_raw"),
            ("camera_info", "/camera/camera_info"),
            ("image_rect", "/camera/image_rect"),
        ],
    )

    apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        output="screen",
        parameters=[tags_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("image_rect", "/camera/image_rect"),
            ("camera_info", "/camera/camera_info"),
            ("detections", "/detections"),
        ],
    )

    # Draws on image_raw rather than image_rect purely because it's the feed
    # you recognise; corners come from the rectified image, so boxes drift
    # slightly at the frame edges once real distortion is calibrated in.
    overlay = Node(
        package="robomaster_detection",
        executable="tag_overlay_node",
        name="tag_overlay",
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
            rectify,
            apriltag,
            overlay,
        ]
    )
