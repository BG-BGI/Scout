"""Nav2 bringup with the scout profile overlay + a depth/pointcloud guard.

Wraps upstream nav2_bringup/navigation_launch.py so the selected profile's
nav2.yaml overlay is merged in (ADR-0010), and so the depth_layer<->pointcloud
coupling (ADR-0002) fails loudly at launch instead of silently starving the
costmap. Gives nav2 a scout launch file (the others already had one).

    ros2 launch scout nav2.launch.py                       # default profile
    ros2 launch scout nav2.launch.py profile:=tight_tunnel
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch import LaunchDescription
from scout.robot_profile import merged_params


def _plugins_have_depth(params):
    for cm in ('local_costmap', 'global_costmap'):
        node = params.get(cm, {}).get(cm, {}).get('ros__parameters', {})
        if 'depth_layer' in (node.get('plugins') or []):
            return True
    return False


def _launch_setup(context, *args, **kwargs):
    profile = LaunchConfiguration('profile').perform(context)
    params_file = merged_params('nav2.yaml', profile)

    # Coupling guard (ADR-0002): if a costmap still marks via depth_layer, this
    # profile's camera MUST publish a pointcloud, or the layer starves silently.
    with open(params_file) as f:
        nav2 = yaml.safe_load(f) or {}
    if _plugins_have_depth(nav2):
        with open(merged_params('realsense.yaml', profile)) as f:
            cam = yaml.safe_load(f) or {}
        if cam.get('pointcloud.enable') is False:
            raise RuntimeError(
                'profile %r keeps nav2 depth_layer but disables the realsense '
                'pointcloud — the depth costmap layer would starve (ADR-0002)'
                % profile)

    nav_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch',
        'navigation_launch.py')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': 'false',
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile', default_value='default',
            description='Config profile (default | tight_tunnel)'),
        OpaqueFunction(function=_launch_setup),
    ])
