"""Nav2 bringup with the scout profile overlay + a depth/pointcloud guard.

Wraps upstream nav2_bringup/navigation_launch.py so the selected profile's
nav2.yaml overlay is merged in (ADR-0010), and so the stvl_layer<->pointcloud
coupling (ADR-0002) fails loudly at launch instead of silently starving the
costmap. Gives nav2 a scout launch file (the others already had one).

(The keepout/speed zone filter wiring — ADR-0019 — was removed 2026-08-24
along with zone_manager; see git history if zones ever come back.)

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
from launch_ros.actions import Node

from launch import LaunchDescription
from scout.robot_profile import merged_params


def _plugins_have_depth(params):
    for cm in ('local_costmap', 'global_costmap'):
        node = params.get(cm, {}).get(cm, {}).get('ros__parameters', {})
        if 'stvl_layer' in (node.get('plugins') or []):
            return True
    return False


def _launch_setup(context, *args, **kwargs):
    profile = LaunchConfiguration('profile').perform(context)
    params_file = merged_params('nav2.yaml', profile)

    # Coupling guard (ADR-0002): if a costmap still marks via stvl_layer, this
    # profile's camera MUST publish a pointcloud, or the layer starves silently.
    with open(params_file) as f:
        nav2 = yaml.safe_load(f) or {}
    if _plugins_have_depth(nav2):
        with open(merged_params('realsense.yaml', profile)) as f:
            cam = yaml.safe_load(f) or {}
        if cam.get('pointcloud.enable') is False:
            raise RuntimeError(
                'profile %r keeps nav2 stvl_layer but disables the realsense '
                'pointcloud — the depth costmap layer would starve (ADR-0002)'
                % profile)

    nav_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch',
        'navigation_launch.py')
    use_composition = (
        LaunchConfiguration('use_composition').perform(context).lower()
        in ('true', '1'))

    actions = []
    if use_composition:
        # Composed bring-up: all 8 nav2 nodes as components in one process --
        # one executor, intra-process comms, 1 DDS participant instead of 8.
        # navigation_launch.py only LOADS components; the container normally
        # comes from bringup_launch.py (which we skip -- no amcl/map_server),
        # so create it here, mirroring bringup_launch.py's container node.
        actions.append(Node(
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[params_file, {'autostart': True}],
            output='screen'))
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': 'false',
                'use_composition': str(use_composition),
                'container_name': 'nav2_container',
            }.items(),
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile', default_value='default',
            description='Config profile (default | tight_tunnel)'),
        DeclareLaunchArgument(
            'use_composition', default_value='true',
            description='Load the 8 nav2 nodes into one component container '
                        '(false = one process per node, the pre-2026-08-15 '
                        'behavior; keep available for CPU A/B on the Pi)'),
        OpaqueFunction(function=_launch_setup),
    ])
