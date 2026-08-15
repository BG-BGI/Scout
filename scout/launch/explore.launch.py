"""Bring up explore_lite for autonomous frontier mapping (tight-tunnel profile).

    ros2 launch scout explore.launch.py
"""

import os

from launch_ros.actions import Node

from launch import LaunchDescription
from scout.robot_profile import resolve_config_dir


def generate_launch_description():
    # Bind-mount-wins config resolution, owned by scout.robot_profile.
    config = resolve_config_dir()

    return LaunchDescription([
        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[os.path.join(config, 'explore_tight_tunnel.yaml')],
        ),
    ])
