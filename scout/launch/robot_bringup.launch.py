import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('scout')

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot_description.launch.py')
        )
    )

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sensors.launch.py')
        )
    )

    return LaunchDescription([
        robot_description,
        sensors,
        Node(
            package='scout',
            executable='motor_driver',
        ),
        Node(
            package='scout',
            executable='joystick_teleop',
        ),
    ])
