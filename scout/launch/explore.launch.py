"""Bring up explore_lite for autonomous frontier mapping (tight-tunnel profile).

    ros2 launch scout explore.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    scout_share = get_package_share_directory('scout')
    config = os.path.join(scout_share, 'config')
    src_config = '/ros_ws/src/scout/config'
    if os.path.isdir(src_config):
        config = src_config

    return LaunchDescription([
        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[os.path.join(config, 'explore_tight_tunnel.yaml')],
        ),
    ])
