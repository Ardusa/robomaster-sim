"""robot_state_publisher (URDF + TF) and joint_state_publisher.

Shared by both backends. `sim` picks which <ros2_control> block the URDF emits
— see robomaster_description/urdf/ros2_control.urdf.xacro. It's passed down
from bringup.launch.py, which is the one place SIM is read.

No controller_manager here: the backends load it differently (sim in-process
via the Gazebo plugin, tether via ros2_control_node), and the spawners would
hang this if it were run on its own. See robomaster_drivetrain/control.launch.py.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _nodes(context, *args, **kwargs):
    desc_pkg = get_package_share_directory("robomaster_description")
    drivetrain_pkg = get_package_share_directory("robomaster_drivetrain")
    xacro_file = os.path.join(desc_pkg, "urdf", "robomaster_ep.urdf.xacro")

    sim = LaunchConfiguration("sim").perform(context)
    use_sim_time = sim == "true"

    # Required only on the tether path — sim never dials the robot. Fail here
    # with the reason rather than letting an unset var reach the driver as a
    # connection timeout.
    robot_ip = os.environ.get("ROBOMASTER_IP", "")
    if not use_sim_time and not robot_ip:
        raise RuntimeError(
            "ROBOMASTER_IP is not set. Set it in .env (direct-connect AP mode "
            "is usually 192.168.2.1). Required when SIM=false."
        )

    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            "sim": sim,
            "robot_ip": robot_ip,
            # Passed in so description doesn't have to $(find ...) it itself.
            # Controllers YAML lives in drivetrain, not here.
            "sim_controllers_file": os.path.join(
                drivetrain_pkg, "config", "sim_controllers.yaml"
            ),
        },
    ).toxml()

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )

    actions = [rsp]
    if not use_sim_time:
        # Tether has no joint_state_broadcaster for the full chain; JSP fills in.
        # In sim, JSB already owns /joint_states — a second JSP publisher zeros
        # the arm/gripper and fights the controllers (and flickers wheel TF).
        actions.append(
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("sim", default_value="false", choices=["true", "false"]),
            OpaqueFunction(function=_nodes),
        ]
    )
