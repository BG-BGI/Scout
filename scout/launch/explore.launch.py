"""Bring up explore_lite for autonomous frontier mapping.

    ros2 launch scout explore.launch.py
    ros2 launch scout explore.launch.py profile:=tight_tunnel
"""

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription
from scout.robot_profile import merged_params


def _setup(context, *args, **kwargs):
    profile = LaunchConfiguration('profile').perform(context)
    return [
        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[merged_params('explore.yaml', profile)],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile', default_value='default',
            description='Config profile (default | tight_tunnel).'),
        OpaqueFunction(function=_setup),
    ])
