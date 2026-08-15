"""Bring up slam_toolbox in one of three map modes.

    mode:=new           (default) start a fresh map
    mode:=localization  load a saved map and localize in it, adding nothing
    mode:=continue      load a saved map and keep building on top of it

A launch file rather than a bare `ros2 run` because the mode is not a parameter --
see the block comment on MODES below.

Examples:
    ros2 launch scout slam.launch.py
    ros2 launch scout slam.launch.py mode:=continue map:=house
    ros2 launch scout slam.launch.py mode:=localization map:=house \
        map_start_pose:=1.5,0.0,3.14159
    ros2 launch scout slam.launch.py mode:=new params_file:=slam_tight_tunnel.yaml
"""

import os

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription
from scout.robot_profile import resolve_config

# Serialized pose graphs live in the repo, which is bind-mounted at /ros_ws/src, so
# they land on the host where they can be inspected and copied off. Deliberately not
# the package share directory: that is inside the image/volume and is rebuilt.
MAPS_DIR = '/ros_ws/src/maps'

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
# ⚠ map_start_at_dock is unusable in localization mode. LocalizationSlamToolbox
# overrides loadPoseGraphByParams and warns "Starting localization at first node (dock)
# is correctly not supported", then localizes at the pose anyway -- so localization must
# be given a pose and continue is the only mode that can use the dock.
MODES = ('new', 'localization', 'continue')


def _saved_maps():
    """Basenames of the maps that are actually loadable, for error messages."""
    if not os.path.isdir(MAPS_DIR):
        return []
    suffix = '.posegraph'
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


def _resolve_params_file(name):
    """Basename under scout/config; bind-mount wins over install share
    (one policy, owned by scout.robot_profile — ADR-0013)."""
    return resolve_config(name)


def _launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    map_name = LaunchConfiguration('map').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    if mode not in MODES:
        raise RuntimeError(
            f"Unknown mode '{mode}'. Expected one of: {', '.join(MODES)}."
        )

    if mode == 'new':
        executable = 'async_slam_toolbox_node'
        mode_params = {}
    elif mode == 'continue':
        executable = 'async_slam_toolbox_node'
        # Resume at the loaded graph's first node and keep adding to it.
        mode_params = {**_map_params(map_name), 'map_start_at_dock': True}
    else:
        executable = 'localization_slam_toolbox_node'
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
        mode_params = {
            **_map_params(map_name),
            'map_start_pose': start_pose,
            # Mirrors upstream's localization config: the buffer holds the rolling band
            # of recent scans matched against the fixed graph, and does not need the
            # depth that graph building does.
            'scan_buffer_size': 3,
        }

    config = _resolve_params_file(params_file)

    return [
        Node(
            package='slam_toolbox',
            executable=executable,
            # slam.yaml is keyed on this name, which every executable hardcodes anyway.
            name='slam_toolbox',
            output='screen',
            parameters=[config, mode_params],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='new',
            description='new (fresh map) | localization (load, do not extend) | '
                        'continue (load and keep mapping)',
        ),
        DeclareLaunchArgument(
            'map',
            default_value='map',
            description=f'Basename of the saved pose graph in {MAPS_DIR}, no '
                        'extension. Ignored by mode:=new.',
        ),
        DeclareLaunchArgument(
            'map_start_pose',
            default_value='0.0,0.0,0.0',
            description='Where in the loaded map the robot is starting, as '
                        '"x,y,theta". Used by mode:=localization; refine later by '
                        'publishing /initialpose.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='slam.yaml',
            description='Basename (or absolute path) of the slam_toolbox params '
                        'YAML under scout/config. Use slam_tight_tunnel.yaml for pipes.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
