"""Real-robot camera: H.264 stream -> /camera/image_raw + /camera/camera_info.

Included by tether.launch.py with arm_stream:=false (robomaster_tether owns the
control port and arms the stream itself). Run standalone with the default
arm_stream:=true to get video with no tether backend — useful for checking the camera
or calibrating without driving.

Pair with robomaster_detection's detection.launch.py for object detection.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera = Node(
        package="robomaster_camera",
        executable="camera_node",
        name="camera_node",
        output="screen",
        parameters=[{"arm_stream": LaunchConfiguration("arm_stream")}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "arm_stream",
                default_value="true",
                choices=["true", "false"],
                description="Send 'stream on' from here. False when robomaster_tether already did.",
            ),
            camera,
        ]
    )
