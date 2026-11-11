"""Location site store (ADR-0023, multi-map v2 ADR-0029). Pure: stdlib only.

A *site* is one physical location's full state bundle under `sites/<name>/`:
maps (posegraph, waypoints, tags) and captures. The
relative symlink `sites/active -> <name>` is the single switch point — it
resolves identically through every bind mount of the parent dir, and nodes
that open files per operation pick up a switch with no restart at all.

`site.json` (schema v2 — a site holds multiple labeled maps, e.g. one per
floor of a building; `active_map` is the one slam/amcl runs on):

    {"version": 2,
     "display_name": "Office A",
     "active_map": "floor1",            # key of maps, or null
     "slam_mode": "auto",               # auto | new | localization | continue
     "maps": {
       "floor1": {"label": "Lobby",     # human label, defaults to the name
                  "floor": 1,           # int floor number, or null
                  "map_start_pose": [0.0, 0.0, 0.0]},  # localization only
       "yard":   {"label": "Yard", "floor": null,
                  "map_start_pose": [0.0, 0.0, 0.0]}},
     "created": "2026-08-22T15:04:05Z"}

v1 files (`default_map` + top-level `map_start_pose`) are normalized to v2 in
memory here; fleet-status write-upgrades them on its next write. Map files
stay flat in the site's `maps/` as `<name>.{posegraph,data,yaml,pgm}` — the
schema change moves nothing on disk.

`slam_mode: auto` resolves to `continue` when the active map's .posegraph
exists, else `new` — deliberately never `localization`, because in that mode
slam_toolbox is not running at all (amcl + map_server localize on the saved
grid, ADR-0028) so nothing can be saved, and a revisited site wants its graph
extended anyway.

The name regex is a shared contract with fleet-status (which duplicates it —
share the schema, not code, per ADR-0011). Map names use the same regex.
"""

import json
import os
import re

VERSION = 2
SITE_NAME_RE = re.compile(r'^[a-z0-9][a-z0-9_-]{0,31}$')
MAP_NAME_RE = SITE_NAME_RE
SLAM_MODES = ('auto', 'new', 'localization', 'continue')

_DEFAULTS = {
    'version': VERSION,
    'display_name': '',
    'active_map': None,
    'slam_mode': 'auto',
    'maps': {},
}

_MAP_DEFAULTS = {
    'label': '',
    'floor': None,
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


def _norm_map_entry(name, entry):
    m = dict(_MAP_DEFAULTS)
    if isinstance(entry, dict):
        for key in m:
            if key in entry and entry[key] is not None:
                m[key] = entry[key]
    if not m['label']:
        m['label'] = name
    return m


def load_site(site_dir):
    """Read site.json merged over defaults, normalizing v1 -> v2 in memory.
    Raises on unreadable/invalid JSON; a missing file raises FileNotFoundError
    (callers decide how loud to be)."""
    with open(os.path.join(site_dir, 'site.json')) as f:
        data = json.load(f)
    site = dict(_DEFAULTS)
    if isinstance(data, dict):
        for key in ('display_name', 'slam_mode'):
            if key in data and data[key] is not None:
                site[key] = data[key]
        if 'created' in data:
            site['created'] = data['created']
        if isinstance(data.get('maps'), dict):
            site['maps'] = {n: _norm_map_entry(n, e)
                            for n, e in data['maps'].items()}
            if data.get('active_map') is not None:
                site['active_map'] = data['active_map']
        elif data.get('default_map'):
            # v1: one map, its start pose at the top level.
            name = data['default_map']
            site['maps'] = {name: _norm_map_entry(
                name, {'map_start_pose': data.get('map_start_pose')})}
            site['active_map'] = name
    if site['slam_mode'] not in SLAM_MODES:
        raise ValueError(
            f"site.json slam_mode '{site['slam_mode']}' not one of "
            f"{', '.join(SLAM_MODES)}")
    if site['active_map'] is not None and site['active_map'] not in site['maps']:
        raise ValueError(
            f"site.json active_map '{site['active_map']}' not in maps "
            f"({', '.join(sorted(site['maps'])) or 'none'})")
    return site


def resolve_slam(site, maps_dir):
    """Site policy -> (mode, map_name, start_pose) for slam.launch.py's
    existing three-way executable table (ADR-0003 untouched — this only
    computes its inputs)."""
    mode = site['slam_mode']
    map_name = site['active_map']
    if mode == 'auto':
        has_map = bool(map_name) and os.path.exists(
            os.path.join(maps_dir, map_name + '.posegraph'))
        mode = 'continue' if has_map else 'new'
    if mode != 'new' and not map_name:
        raise ValueError(
            f"site.json slam_mode '{site['slam_mode']}' needs an active_map")
    raw = (site['maps'][map_name]['map_start_pose']
           if map_name else _MAP_DEFAULTS['map_start_pose'])
    pose = [float(v) for v in raw]
    if len(pose) != 3:
        raise ValueError(
            f'site.json map_start_pose must be [x, y, theta], got {pose}')
    return mode, map_name, pose
