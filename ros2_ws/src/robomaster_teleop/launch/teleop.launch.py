"""Keyboard teleop for the RoboMaster EP.

Publishes Twist on /cmd_vel_teleop. robomaster_drivetrain's mux picks that up
and feeds the mecanum controller — this package only owns the human input side.

Must run in a foreground terminal: teleop_twist_keyboard reads raw stdin, and a
launch child under bringup has no TTY, so keys would never arrive. The Makefile
starts the stack in the background and this launch in the foreground for that
reason.

Future arm (and other) keybinds belong here as additional nodes/remaps; keep
them publishing to subsystem topics rather than calling controllers directly.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    # ExecuteProcess (not a Node) so stdin from `docker compose exec` / a real
    # terminal reaches teleop_twist_keyboard. A Node under launch does not.
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "teleop_twist_keyboard",
                    "teleop_twist_keyboard",
                    "--ros-args",
                    "-r",
                    "/cmd_vel:=/cmd_vel_teleop",
                ],
                output="screen",
                emulate_tty=True,
            )
        ]
    )
