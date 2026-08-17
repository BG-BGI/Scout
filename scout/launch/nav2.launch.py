"""Nav2 bringup with the scout profile overlay + a depth/pointcloud guard.

Wraps upstream nav2_bringup/navigation_launch.py so the selected profile's
nav2.yaml overlay is merged in (ADR-0010), and so the depth_layer<->pointcloud
coupling (ADR-0002) fails loudly at launch instead of silently starving the
costmap. Gives nav2 a scout launch file (the others already had one).

Also wires the keepout/speed zone filters (ADR-0019) — mask servers, filter
info servers and the costmap `filters` entries — but ONLY when zone_manager
has rendered mask files into maps/. With no zones drawn nav2 is byte-identical
to before; the first zone ever drawn needs one nav2 restart to pick this up,
after which zone edits hot-reload through the mask servers.

    ros2 launch scout nav2.launch.py                       # default profile
    ros2 launch scout nav2.launch.py profile:=tight_tunnel
"""

import os
import tempfile

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
from scout.robot_profile import deep_merge, merged_params

# zone_manager's mask output dir (same repo-root bind convention as its
# masks_dir parameter default — keep the two in step).
_MASKS_DIR = '/ros_ws/src/maps'

# Costmap filter entries injected only when the masks exist. `filters` is
# Costmap2DROS's separate filter-plugin list (kept apart from `plugins` so
# filters run after the layers). Keepout goes in BOTH costmaps (planner avoids
# it AND the controller refuses trajectories into it); speed only makes sense
# globally — SpeedFilter publishes /speed_limit, which controller_server
# already subscribes by default.
_ZONE_PARAMS = {
    'global_costmap': {'global_costmap': {'ros__parameters': {
        'filters': ['keepout_filter', 'speed_filter'],
        'keepout_filter': {
            'plugin': 'nav2_costmap_2d::KeepoutFilter',
            'enabled': True,
            'filter_info_topic': '/keepout_filter_info'},
        'speed_filter': {
            'plugin': 'nav2_costmap_2d::SpeedFilter',
            'enabled': True,
            'filter_info_topic': '/speed_filter_info',
            'speed_limit_topic': '/speed_limit'}}}},
    'local_costmap': {'local_costmap': {'ros__parameters': {
        'filters': ['keepout_filter'],
        'keepout_filter': {
            'plugin': 'nav2_costmap_2d::KeepoutFilter',
            'enabled': True,
            'filter_info_topic': '/keepout_filter_info'}}}},
}


def _zone_actions():
    """Mask server + filter info server per filter, one lifecycle manager.
    CostmapFilterInfo type: 0 = keepout, 1 = speed limit in PERCENT (the
    zones store speed_pct); base 0 / multiplier 1 = mask value used as-is."""
    actions = []
    names = []
    for kind, ftype in (('keepout', 0), ('speed', 1)):
        mask_yaml = os.path.join(_MASKS_DIR, 'zone_%s.yaml' % kind)
        actions.append(Node(
            package='nav2_map_server',
            executable='map_server',
            name='%s_mask_server' % kind,
            output='screen',
            parameters=[{'yaml_filename': mask_yaml,
                         'topic_name': '/%s_mask' % kind,
                         'frame_id': 'map'}]))
        actions.append(Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='%s_filter_info_server' % kind,
            output='screen',
            parameters=[{'type': ftype,
                         'filter_info_topic': '/%s_filter_info' % kind,
                         'mask_topic': '/%s_mask' % kind,
                         'base': 0.0,
                         'multiplier': 1.0}]))
        names += ['%s_mask_server' % kind, '%s_filter_info_server' % kind]
    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_zones',
        output='screen',
        parameters=[{'autostart': True, 'node_names': names}]))
    return actions


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

    # Zone filters (ADR-0019): only when zone_manager has rendered masks —
    # with none drawn, nav2 params and process set are unchanged.
    zones_on = all(
        os.path.isfile(os.path.join(_MASKS_DIR, 'zone_%s.yaml' % k))
        for k in ('keepout', 'speed'))
    if zones_on:
        merged = deep_merge(nav2, _ZONE_PARAMS)
        out_dir = os.path.join(tempfile.gettempdir(), 'scout_profile')
        os.makedirs(out_dir, exist_ok=True)
        params_file = os.path.join(out_dir, 'zones-%s-nav2.yaml' % profile)
        with open(params_file, 'w') as f:
            yaml.safe_dump(merged, f)

    nav_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch',
        'navigation_launch.py')
    use_composition = (
        LaunchConfiguration('use_composition').perform(context).lower()
        in ('true', '1'))

    actions = _zone_actions() if zones_on else []
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
