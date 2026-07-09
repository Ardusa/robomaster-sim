import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    desc_pkg = get_package_share_directory('robomaster_description')
    xacro_file = os.path.join(desc_pkg, 'urdf', 'robomaster_ep.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # let Gazebo resolve the package:// mesh URIs
    resource_path = SetEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', os.path.dirname(desc_pkg))

    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r --render-engine ogre empty.sdf'}.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'robomaster_ep', '-z', '0.1'],
        output='screen',
    )

    return LaunchDescription([
        resource_path,
        gz,
        rsp,
        TimerAction(period=4.0, actions=[spawn]),  # let Gazebo come up first
    ])