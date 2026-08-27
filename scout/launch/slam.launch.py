"""Bring up the mapping/localization stack in one of four map modes.

    mode:=site          (compose default) read sites/active/site.json and
                        resolve to one of the three modes below (ADR-0023)
    mode:=new           start a fresh map (slam_toolbox)
    mode:=localization  load a saved map and localize in it, adding nothing
                        (amcl + map_server, ADR-0028 -- NOT slam_toolbox)
    mode:=continue      load a saved map and keep building on top of it
                        (slam_toolbox)

A launch file rather than a bare `ros2 run` because the slam_toolbox mode is not
a parameter -- see the block comment on MODES below.

Examples:
    ros2 launch scout slam.launch.py mode:=site
    ros2 launch scout slam.launch.py mode:=continue map:=house
    ros2 launch scout slam.launch.py mode:=localization map:=house \
        map_start_pose:=1.5,0.0,3.14159
    ros2 launch scout slam.launch.py mode:=new profile:=tight_tunnel
"""

import os

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription
from scout.core import sites
from scout.robot_profile import merged_params

# Serialized pose graphs live in the active site's bundle in the repo, which is
# bind-mounted at /ros_ws/src, so they land on the host where they can be inspected
# and copied off. Deliberately not the package share directory: that is inside the
# image/volume and is rebuilt. `sites/active` is a relative symlink repointed by
# fleet_status on a site switch (ADR-0023); resolved fresh at every launch, which is
# exactly when this container restarts.
SITES_ROOT = os.environ.get('SCOUT_SITES_ROOT', '/ros_ws/src/sites')
MAPS_DIR = os.path.join(SITES_ROOT, 'active', 'maps')

# ⚠ THE MODE IS NOT A PARAMETER. Every upstream slam_toolbox config and nearly every
# tutorial carries `mode: mapping` / `mode: localization`, but that key is DEAD: no
# declare_parameter("mode", ...) exists anywhere in slam_toolbox_common.cpp,
# slam_mapper.cpp or the karto Mapper. Setting it does nothing at all. The real switch
# is which executable runs -- async_slam_toolbox_node leaves processor_type_ at PROCESS
# while localization_slam_toolbox_node sets PROCESS_LOCALIZATION in its constructor,
# and only the latter refuses to add scans to the graph. Hence this table.
#
# ⚠ The map-loading keys must be ABSENT, not false. SlamToolbox::shouldStartWithPoseGraph
# needs map_file_name AND one of map_start_pose / map_start_at_dock, and it tests them
# with `get_type() != PARAMETER_NOT_SET`. So `map_start_at_dock: false` reads as SET and
# silently takes the start-at-a-pose branch instead. That is why these live here, built
# per mode, and not in slam.yaml where they would always exist.
#
# Localization mode no longer touches slam_toolbox at all (ADR-0028): it brings up
# nav2's amcl + map_server + a lifecycle manager on the GRID map (<name>.yaml/.pgm
# from /slam_toolbox/save_map -- the webui Save Map button writes both pairs). amcl
# searches globally via its particle cloud, reports real confidence in /amcl_pose
# covariance, takes /initialpose natively (tag_relocalizer's seed), and exposes
# reinitialize_global_localization -- all the things the scan-matcher's silent
# local-only lock could not do.
MODES = ('site', 'new', 'localization', 'continue')


def _saved_maps(suffix='.posegraph'):
    """Basenames of the maps that are actually loadable, for error messages."""
    if not os.path.isdir(MAPS_DIR):
        return []
    return sorted(f[:-len(suffix)] for f in os.listdir(MAPS_DIR)
                  if f.endswith(suffix))


def _map_params(map_name):
    """Resolve a map basename to a map_file_name, failing loudly if it is missing."""
    path = os.path.join(MAPS_DIR, map_name)
    # ⚠ A missing map is NOT fatal to slam_toolbox, which is the dangerous part:
    # deserializePoseGraphCallback logs "Failed to read file" and then returns true, so
    # the node carries on with an empty graph while looking perfectly healthy. In
    # localization mode that means localizing against nothing. Refuse at launch instead.
    if not os.path.exists(path + '.posegraph'):
        available = ', '.join(_saved_maps()) or '(none)'
        raise RuntimeError(
            f"No saved map named '{map_name}': {path}.posegraph does not exist. "
            f'Maps available in {MAPS_DIR}: {available}. '
            'Save one from a running mapping session with the '
            '/slam_toolbox/serialize_map service (see NOTES.md).'
        )
    # No extension: serialization::write/read append .posegraph and .data themselves.
    return {'map_file_name': path}


def _grid_map_yaml(map_name):
    """Absolute path of the grid map YAML for amcl, failing loudly if missing.

    Localization loads the .yaml/.pgm pair, not the posegraph -- a site mapped
    before ADR-0028 has only the posegraph, and map_server would otherwise die
    at activation with a much less helpful error."""
    path = os.path.join(MAPS_DIR, map_name + '.yaml')
    if not os.path.exists(path):
        available = ', '.join(_saved_maps('.yaml')) or '(none)'
        raise RuntimeError(
            f"No grid map named '{map_name}': {path} does not exist. "
            f'Grid maps available in {MAPS_DIR}: {available}. '
            'Localization mode runs amcl on the .yaml/.pgm pair, not the '
            'posegraph. Save one from a running mapping session with the webui '
            'Save Map button (writes both pairs) or the /slam_toolbox/save_map '
            'service.'
        )
    return path


def _launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    map_name = LaunchConfiguration('map').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    profile = LaunchConfiguration('profile').perform(context)

    if mode not in MODES:
        raise RuntimeError(
            f"Unknown mode '{mode}'. Expected one of: {', '.join(MODES)}."
        )

    if mode == 'site':
        # Resolve the active site's policy to one of the three real modes below.
        # Only computes (mode, map, pose) — the executable table stays the switch.
        active = sites.active_site_name(SITES_ROOT)
        if active is None:
            raise RuntimeError(
                f'No active site: {SITES_ROOT}/active is missing or broken. '
                'Run `python3 scripts/migrate_sites.py` on the Pi once, or '
                'create a site from the webui Site panel.'
            )
        try:
            site = sites.load_site(os.path.join(SITES_ROOT, active))
            mode, map_name, start_pose = sites.resolve_slam(site, MAPS_DIR)
        except (OSError, ValueError) as e:
            raise RuntimeError(f"Bad site '{active}': {e}") from e
    else:
        pose = LaunchConfiguration('map_start_pose').perform(context)
        try:
            start_pose = [float(v) for v in pose.split(',')]
        except ValueError:
            start_pose = []
        if len(start_pose) != 3:
            raise RuntimeError(
                f"map_start_pose must be three comma-separated numbers 'x,y,theta', "
                f"got '{pose}'."
            )

    if mode == 'localization':
        return _localization_nodes(map_name, start_pose, profile)

    if mode == 'new':
        mode_params = {}
    else:  # continue
        # Resume at the loaded graph's first node and keep adding to it.
        mode_params = {**_map_params(map_name), 'map_start_at_dock': True}

    # Base params + the profile's slam overlay (tight_tunnel = finer grid).
    config = merged_params(params_file, profile)

    return [
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            # slam.yaml is keyed on this name, which every executable hardcodes anyway.
            name='slam_toolbox',
            output='screen',
            parameters=[config, mode_params],
        ),
    ]


def _localization_nodes(map_name, start_pose, profile):
    """amcl + map_server + lifecycle manager on the saved grid map (ADR-0028).

    Together these own exactly what localization_slam_toolbox_node owned: /map
    (map_server, latched) and map->odom (amcl). tag_relocalizer's /initialpose
    seed is consumed by amcl natively; initial_pose.* below is only the
    pre-seed guess from the site policy."""
    config = merged_params('amcl.yaml', profile)
    map_yaml = _grid_map_yaml(map_name)
    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[config, {'yaml_filename': map_yaml}],
        ),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[config, {
                'initial_pose.x': start_pose[0],
                'initial_pose.y': start_pose[1],
                'initial_pose.yaw': start_pose[2],
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[config],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='new',
            description='site (resolve from sites/active/site.json) | new (fresh '
                        'map) | localization (load, do not extend) | continue '
                        '(load and keep mapping)',
        ),
        DeclareLaunchArgument(
            'map',
            default_value='map',
            description=f'Basename of the saved map in {MAPS_DIR}, no extension '
                        '(continue loads <map>.posegraph, localization loads '
                        '<map>.yaml). Ignored by mode:=new and mode:=site.',
        ),
        DeclareLaunchArgument(
            'map_start_pose',
            default_value='0.0,0.0,0.0',
            description='Where in the loaded map the robot is starting, as '
                        '"x,y,theta". Used by mode:=localization (amcl '
                        'initial_pose); refined later by /initialpose — '
                        'tag_relocalizer publishes it on the first registered-'
                        'tag sighting.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='slam.yaml',
            description='Basename (or absolute path) of the slam_toolbox params '
                        'YAML under scout/config (the profile overlay merges on '
                        'top). Ignored by mode:=localization, which uses amcl.yaml.',
        ),
        DeclareLaunchArgument(
            'profile',
            default_value='default',
            description='Config profile (default | tight_tunnel).',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
