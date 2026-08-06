"""Shared drivetrain — controller spawners + twist arbitration.

Included by both backends unchanged: each stands up a controller_manager under
the same name, so the spawners don't care which one is there.

    teleop   -> /cmd_vel_teleop   --\\
                                     cmd_vel_mux -> mecanum_drive_controller
    autonomy -> /cmd_vel_autonomy --/

To add a new way to drive the robot, add a topic to config/twist_mux.yaml —
the controller and driver stay untouched.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    drivetrain_pkg = get_package_share_directory("robomaster_drivetrain")
    twist_mux_config = os.path.join(drivetrain_pkg, "config", "twist_mux.yaml")

    sim = LaunchConfiguration("sim")
    use_sim_time = PythonExpression(["'", sim, "' == 'true'"])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Remapped from this end because a spawner can't remap a controller's
    # topics. reference_unstamped is the plain-Twist input, which exists only
    # because both controller yamls set use_stamped_vel: false.
    #
    # Our own mux, not the twist_mux package: that binary links
    # libdiagnostic_updater.so, which no ros-humble-diagnostic-updater in the
    # repos ships. Same config schema, so it can be swapped back if fixed.
    cmd_vel_mux = Node(
        package="robomaster_drivetrain",
        executable="cmd_vel_mux.py",
        name="cmd_vel_mux",
        output="screen",
        parameters=[twist_mux_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("cmd_vel_out", "/mecanum_drive_controller/reference_unstamped"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("sim", default_value="false", choices=["true", "false"]),
            joint_state_broadcaster_spawner,
            mecanum_drive_controller_spawner,
            cmd_vel_mux,
        ]
    )
