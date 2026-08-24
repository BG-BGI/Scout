"""App-behavior nodes, split out of robot.launch.py for fault isolation.

Trick macros, follow-me, clutter mapping, patrol capture, zone manager: all
inert until triggered by a service/topic call, none touch the drivetrain or
sensor stack directly. Crashing or restarting this container does not disturb
roboclaw_driver, the camera, lidar, EKF, or tilt_monitor in `robot`.

This container is also in the site switch's restart set (ADR-0023): all its
site-scoped state (zones map_name, clutter file, patrol waypoints) is re-read
here at launch, which fleet_status triggers after repointing sites/active.

    ros2 launch scout behaviors.launch.py
"""

import os

from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from launch import LaunchDescription
from scout.core import sites

SITES_ROOT = os.environ.get('SCOUT_SITES_ROOT', '/ros_ws/src/sites')
SITE_MAPS = os.path.join(SITES_ROOT, 'active', 'maps')


def _site():
    """Active site.json, or None. Behaviors is ADR-0015 tier 2 — a missing or
    broken site degrades to safe defaults instead of crash-looping."""
    active = sites.active_site_name(SITES_ROOT)
    if active is None:
        return None
    try:
        return sites.load_site(os.path.join(SITES_ROOT, active))
    except (OSError, ValueError):
        return None


def _launch_setup(context, *args, **kwargs):
    site = _site()
    default_map = (site or {}).get('default_map')

    # Clutter persistence is only meaningful when slam runs on a saved map
    # (localization/continue): under mode:=new the map frame resets every boot
    # and a loaded clutter file paints phantom obstacles at wrong coordinates
    # (poisons nav2 planning). With sites this follows the same signal slam's
    # auto mode uses — a default_map means a persistent frame exists.
    clutter_file = os.path.join(SITE_MAPS, 'clutter.npz') if default_map else ''

    return [
        # Persistent under-lidar clutter layer (chair bases, shoes). Idles
        # until slam provides map->base_link.
        Node(
            package='scout',
            executable='clutter_mapper',
            output='screen',
            # process_period 1.0 (up from the 0.3 default): furniture dwells,
            # so 1 Hz marking loses nothing and the numpy/cell work is the
            # node's main CPU cost.
            parameters=[{'file': clutter_file, 'process_period': 1.0}],
        ),

        # Keepout/speed zones: /zone_cmd (webui polygons) -> the site's
        # zones.json -> derived filter masks + hot-reload of nav2's mask
        # servers (ADR-0019). map_name derives from site.json, which kills the
        # old "keep map_name in step with slam's map:=" manual coupling.
        Node(
            package='scout',
            executable='zone_manager',
            output='screen',
            parameters=[{
                'zones_file': os.path.join(SITE_MAPS, 'zones.json'),
                'masks_dir': SITE_MAPS,
                'map_name': default_map or 'default',
            }],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        # Trick macros (web UI). Inert until /play_trick is called.
        Node(
            package='scout',
            executable='trick_player',
            output='screen',
        ),

        # Lidar follow-me. Inert until /follow_me/start is called.
        Node(
            package='scout',
            executable='follow_me',
            output='screen',
        ),

        # Waypoint patrol + pose-stamped photo capture (progress docs).
        # Inert until /patrol/start; needs slam + nav2 for motion. Site paths
        # come from its node defaults (re-resolved per patrol/run).
        Node(
            package='scout',
            executable='patrol_capture',
            output='screen',
        ),

        # Site-dependent nodes (clutter_mapper, zone_manager) — parameters are
        # computed from sites/active/site.json at launch time.
        OpaqueFunction(function=_launch_setup),
    ])
