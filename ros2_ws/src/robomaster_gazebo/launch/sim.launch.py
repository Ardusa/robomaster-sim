"""Simulation backend: shared bringup + Gazebo + spawn.

Includes bringup.launch.py (rsp/TF), then adds the sim-only pieces:
the Gazebo resource path, the Gazebo server/GUI, and spawning the robot
from the robot_description topic.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gz_pkg = get_package_share_directory('robomaster_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'bringup.launch.py')))

    set_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value='/root/ros2_ws/install/robomaster_description/share')

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r --render-engine ogre empty.sdf'}.items(),
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'robomaster_ep', '-z', '0.1'],
        output='screen',
    )

    return LaunchDescription([
        set_resource_path,
        bringup,
        gz,
        TimerAction(period=4.0, actions=[spawn]),  # let Gazebo come up first
    ])