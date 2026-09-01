"""Waypoint + route store, schema v2 — see ADR-0011. Pure: json + stdlib only.

One store (`maps/waypoints.json`) shared by patrol_capture and the scout-skills
server (which vendors the same schema, not this code — separate container).

    {"version": 2,
     "waypoints": {"kitchen": {"x":1.2,"y":3.4,"yaw":0.5,"saved":"...","source":"operator"}},
     "routes": {"patrol": ["kitchen", {"x":6.0,"y":1.0,"yaw":3.14}]}}

Route items are waypoint NAMES (deref at run time, so a tag-refreshed waypoint
is picked up automatically) or inline pose dicts (coverage's generated points,
kept out of the name namespace). `source` in (operator|tag|mark|coverage).
"""

import json
import os

VERSION = 2


def blank():
    """An empty v2 store."""
    return {'version': VERSION, 'waypoints': {}, 'routes': {}}


def migrate(data):
    """Normalize any legacy shape to v2.

    Accepts a v2 store (filled out), a flat ``{name: pose}`` (legacy skills
    waypoints.json), or ``{'waypoints': [pose, ...]}`` (legacy patrol_route.yaml
    parsed to a dict) -> the ordered poses become the inline ``patrol`` route.
    """
    store = blank()
    if not isinstance(data, dict):
        return store
    if data.get('version') == VERSION:
        store['waypoints'] = dict(data.get('waypoints') or {})
        store['routes'] = dict(data.get('routes') or {})
        return store
    wps = data.get('waypoints')
    if isinstance(wps, list):   # patrol_route.yaml: ordered inline poses
        store['routes']['patrol'] = [
            {'x': float(p['x']), 'y': float(p['y']), 'yaw': float(p.get('yaw', 0.0))}
            for p in wps if isinstance(p, dict) and 'x' in p]
        return store
    for name, pose in data.items():   # flat {name: pose} legacy skills file
        if isinstance(pose, dict) and 'x' in pose:
            store['waypoints'][name] = dict(pose)
    return store


def load(path):
    """Load and normalize the store at `path`; a missing file yields blank()."""
    try:
        with open(path) as f:
            return migrate(json.load(f))
    except FileNotFoundError:
        return blank()


def save(path, store):
    """Atomically write the store (tmp + os.replace so a crash can't truncate)."""
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(store, f, indent=2, sort_keys=True)
    os.replace(tmp, path)


def set_waypoint(store, name, pose, source, saved=None, map=None):
    """Insert/replace a named waypoint (pure; caller reloads+saves around it).
    `map` stamps which site map the pose belongs to (ADR-0029); absent =
    legacy = assume the active map."""
    wp = {'x': round(float(pose[0]), 3), 'y': round(float(pose[1]), 3),
          'yaw': round(float(pose[2]), 3), 'source': source}
    if saved is not None:
        wp['saved'] = saved
    if map is not None:
        wp['map'] = map
    store['waypoints'][name] = wp


def resolve_route(store, name):
    """[{x, y, yaw}, ...] for route `name`; string items deref to waypoints.

    Raises KeyError if the route is absent or references missing waypoints.
    """
    items = store.get('routes', {}).get(name)
    if items is None:
        raise KeyError('no route %r (have: %s)'
                       % (name, sorted(store.get('routes', {}))))
    poses, missing = [], []
    for it in items:
        if isinstance(it, str):
            wp = store.get('waypoints', {}).get(it)
            if wp is None:
                missing.append(it)
            else:
                poses.append({'x': wp['x'], 'y': wp['y'], 'yaw': wp['yaw']})
        else:
            poses.append({'x': float(it['x']), 'y': float(it['y']),
                          'yaw': float(it.get('yaw', 0.0))})
    if missing:
        raise KeyError('route %r references missing waypoints: %s' % (name, missing))
    return poses
