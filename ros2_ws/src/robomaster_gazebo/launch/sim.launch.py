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
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gazebo_share = get_package_share_directory("robomaster_gazebo")
    description_share = get_package_share_directory("robomaster_description")

    # headless:=true runs the server with no GUI, which is the only way this is
    # bearable without GPU passthrough (see the Makefile's platform warning).
    # Sensors still render offscreen, so the camera works either way.
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": PythonExpression(
                [
                    "'-r --render-engine ogre ' + '",
                    LaunchConfiguration("world"),
                    "' + ",
                    "(' -s --headless-rendering' if '",
                    LaunchConfiguration("headless"),
                    "' == 'true' else '')",
                ]
            )
        }.items(),
    )

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
                default_value=os.path.join(gazebo_share, "worlds", "small_house.world"),
                description="Gazebo world file.",
            ),
            # Accepted and ignored: bringup passes sim to every include.
            DeclareLaunchArgument("sim", default_value="true", choices=["true", "false"]),
            AppendEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=os.pathsep.join([description_share, gazebo_share]),
            ),
            gz,
            clock_bridge,
            TimerAction(period=4.0, actions=[spawn]),  # let Gazebo come up first
            TimerAction(period=8.0, actions=[camera_bridge, camera_info_bridge]),
        ]
    )
