"""Location site store (ADR-0023). Pure: stdlib only.

A *site* is one physical location's full state bundle under `sites/<name>/`:
maps (posegraph, waypoints, tags) and captures. The
relative symlink `sites/active -> <name>` is the single switch point — it
resolves identically through every bind mount of the parent dir, and nodes
that open files per operation pick up a switch with no restart at all.

`site.json` (schema v1):

    {"version": 1,
     "display_name": "Office A",
     "default_map": "office",           # basename in the site's maps/, or null
     "slam_mode": "auto",               # auto | new | localization | continue
     "map_start_pose": [0.0, 0.0, 0.0], # localization only
     "created": "2026-08-22T15:04:05Z"}

`slam_mode: auto` resolves to `continue` when the default map's .posegraph
exists, else `new` — deliberately never `localization`, because serialize_map
silently no-ops there (reports SUCCESS, writes nothing) and a revisited site
wants its graph extended anyway.

The name regex is a shared contract with fleet-status (which duplicates it —
share the schema, not code, per ADR-0011).
"""

import json
import os
import re

VERSION = 1
SITE_NAME_RE = re.compile(r'^[a-z0-9][a-z0-9_-]{0,31}$')
SLAM_MODES = ('auto', 'new', 'localization', 'continue')

_DEFAULTS = {
    'version': VERSION,
    'display_name': '',
    'default_map': None,
    'slam_mode': 'auto',
    'map_start_pose': [0.0, 0.0, 0.0],
}


def valid_name(name):
    # `active` is the symlink, never a site.
    return bool(SITE_NAME_RE.match(name or '')) and name != 'active'


def active_site_name(sites_root):
    """Name the `active` symlink points at, or None if absent/broken."""
    link = os.path.join(sites_root, 'active')
    try:
        return os.path.basename(os.readlink(link))
    except OSError:
        return None


def load_site(site_dir):
    """Read site.json merged over defaults. Raises on unreadable/invalid JSON;
    a missing file raises FileNotFoundError (callers decide how loud to be)."""
    with open(os.path.join(site_dir, 'site.json')) as f:
        data = json.load(f)
    site = dict(_DEFAULTS)
    if isinstance(data, dict):
        for key in site:
            if key in data and data[key] is not None:
                site[key] = data[key]
        if 'created' in data:
            site['created'] = data['created']
    if site['slam_mode'] not in SLAM_MODES:
        raise ValueError(
            f"site.json slam_mode '{site['slam_mode']}' not one of "
            f"{', '.join(SLAM_MODES)}")
    return site


def resolve_slam(site, maps_dir):
    """Site policy -> (mode, map_name, start_pose) for slam.launch.py's
    existing three-way executable table (ADR-0003 untouched — this only
    computes its inputs)."""
    mode = site['slam_mode']
    map_name = site['default_map']
    if mode == 'auto':
        has_map = bool(map_name) and os.path.exists(
            os.path.join(maps_dir, map_name + '.posegraph'))
        mode = 'continue' if has_map else 'new'
    if mode != 'new' and not map_name:
        raise ValueError(
            f"site.json slam_mode '{site['slam_mode']}' needs a default_map")
    pose = [float(v) for v in site['map_start_pose']]
    if len(pose) != 3:
        raise ValueError(
            f'site.json map_start_pose must be [x, y, theta], got {pose}')
    return mode, map_name, pose
