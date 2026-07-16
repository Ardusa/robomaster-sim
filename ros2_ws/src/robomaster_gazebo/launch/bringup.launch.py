"""Shared ROS2 bringup — everything common to sim and tether.

Right now that's just robot_state_publisher (publishes the URDF + TF tree).
When ros2_control lands, the controller_manager + spawners go here too, since
both the sim (gz_ros2_control) and the real robot (your DJI driver) need the
same controller interface on top.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    desc_pkg = get_package_share_directory('robomaster_description')
    xacro_file = os.path.join(desc_pkg, 'urdf', 'robomaster_ep.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    return LaunchDescription([rsp])