"""Keepout / speed zone store + mask rasterizer (ADR-0019). Pure: stdlib+numpy.

Source of truth is `maps/zones.json` — named polygons per map, same pattern as
the waypoint store (ADR-0011). The nav2 costmap-filter mask PGM+yaml pairs are
DERIVED artifacts, re-rendered by zone_manager on every edit:

    {"version": 1,
     "maps": {"house": {"zones": {
         "keepout-1": {"type": "keepout", "polygon": [[1.0, 2.0], ...]},
         "speed-1":   {"type": "speed", "speed_pct": 40.0, "polygon": [...]}}}}}

The webui edits zones over rosbridge with a |-grammar command string
(/zone_cmd), frozen here + in test_zones.py like the status formats:

    add|<type>|<speed_pct>|x1,y1;x2,y2;x3,y3   (>=3 vertices, map frame)
    delete|<name>
    clear|

Mask geometry is self-sized from the zones' bounding box (+pad): costmap
filters transform mask->costmap through TF, so the mask does NOT need the
slam map's origin/size, and cells outside the mask are simply unfiltered.

PGM encoding: gray = 255 - round(occ * 255 / 100). Keepout masks carry only
0/100 so the default trinary map_server thresholds read them exactly; speed
masks carry intermediate values, so their yaml uses mode: scale with
free_thresh 0.0 / occupied_thresh 0.996 (gray 0 still reads as 100), which
reproduces the percentage within ~1% quantization — fine for a speed limit.
"""

import json
import os

import numpy as np

VERSION = 1
ZONE_TYPES = ('keepout', 'speed')


# --- store (mirrors core.waypoints) -------------------------------------------

def blank():
    return {'version': VERSION, 'maps': {}}


def migrate(data):
    store = blank()
    if isinstance(data, dict) and data.get('version') == VERSION:
        store['maps'] = dict(data.get('maps') or {})
    return store


def load(path):
    try:
        with open(path) as f:
            return migrate(json.load(f))
    except FileNotFoundError:
        return blank()


def save(path, store):
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(store, f, indent=2, sort_keys=True)
    os.replace(tmp, path)


def zones_for(store, map_name):
    """The {name: zone} dict for `map_name` (empty if none drawn yet)."""
    return store['maps'].get(map_name, {}).get('zones', {})


def next_name(zones, ztype):
    """First free '<type>-<n>' (n from 1) — webui zones are auto-named."""
    n = 1
    while '%s-%d' % (ztype, n) in zones:
        n += 1
    return '%s-%d' % (ztype, n)


def set_zone(store, map_name, name, ztype, polygon, speed_pct=None):
    """Insert/replace a zone (pure; caller reloads+saves around it). Raises
    ValueError on a bad type, <3 vertices, or a speed zone without a valid
    percentage in (0, 100]."""
    if ztype not in ZONE_TYPES:
        raise ValueError('zone type must be one of %s: %r' % (ZONE_TYPES, ztype))
    poly = [[round(float(x), 3), round(float(y), 3)] for x, y in polygon]
    if len(poly) < 3:
        raise ValueError('zone polygon needs >= 3 vertices, got %d' % len(poly))
    zone = {'type': ztype, 'polygon': poly}
    if ztype == 'speed':
        if speed_pct is None or not 0.0 < float(speed_pct) <= 100.0:
            raise ValueError('speed zone needs speed_pct in (0, 100], got %r'
                             % speed_pct)
        zone['speed_pct'] = round(float(speed_pct), 1)
    store['maps'].setdefault(map_name, {}).setdefault('zones', {})[name] = zone


def delete_zone(store, map_name, name):
    """Remove a zone; True if it existed."""
    return zones_for(store, map_name).pop(name, None) is not None


# --- /zone_cmd grammar (frozen — same contract style as core.status) -----------

def format_zone_cmd(op, ztype='', speed_pct=None, polygon=(), name=''):
    """'add|<type>|<speed_pct or empty>|x,y;x,y;...' | 'delete|<name>' | 'clear|'."""
    if op == 'add':
        pts = ';'.join('%.3f,%.3f' % (float(x), float(y)) for x, y in polygon)
        spd = '' if speed_pct is None else '%g' % float(speed_pct)
        return 'add|%s|%s|%s' % (ztype, spd, pts)
    if op == 'delete':
        return 'delete|%s' % name
    if op == 'clear':
        return 'clear|'
    raise ValueError('unknown zone op %r' % op)


def parse_zone_cmd(data):
    """-> ('add', ztype, speed_pct or None, [[x,y],...]) | ('delete', name)
    | ('clear',). Raises ValueError on garbage — the wire crosses rosbridge."""
    parts = data.split('|')
    if parts[0] == 'add' and len(parts) == 4:
        ztype, spd, pts = parts[1], parts[2], parts[3]
        polygon = [[float(a) for a in p.split(',')]
                   for p in pts.split(';') if p]
        return ('add', ztype, float(spd) if spd else None, polygon)
    if parts[0] == 'delete' and len(parts) == 2:
        return ('delete', parts[1])
    if parts[0] == 'clear':
        return ('clear',)
    raise ValueError('bad zone command: %r' % data)


# --- rasterization --------------------------------------------------------------

def _points_in_polygon(xs, ys, poly):
    """Even-odd crossing test, vectorized over flat coord arrays."""
    inside = np.zeros(xs.shape, dtype=bool)
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        cross = ((yi > ys) != (yj > ys)) & (
            xs < (xj - xi) * (ys - yi) / ((yj - yi) or 1e-12) + xi)
        inside ^= cross
        j = i
    return inside


def rasterize(zones, resolution=0.05, pad_m=1.0):
    """Render {name: zone} into two occupancy grids (uint8, 0-100, row 0 =
    bottom row, OccupancyGrid convention): keepout cells = 100, speed cells =
    their speed_pct (overlaps keep the SLOWEST limit). Self-sized to the
    zones' bounding box + pad. Returns (keepout, speed, origin_xy) or
    (None, None, None) when there are no zones."""
    polys = [(z['type'], z) for z in zones.values()]
    if not polys:
        return None, None, None
    pts = np.array([p for _, z in polys for p in z['polygon']], dtype=float)
    lo = pts.min(axis=0) - pad_m
    hi = pts.max(axis=0) + pad_m
    width = max(1, int(np.ceil((hi[0] - lo[0]) / resolution)))
    height = max(1, int(np.ceil((hi[1] - lo[1]) / resolution)))
    # Cell-center world coordinates.
    xs = lo[0] + (np.arange(width) + 0.5) * resolution
    ys = lo[1] + (np.arange(height) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys)
    gx, gy = gx.ravel(), gy.ravel()
    keepout = np.zeros((height, width), dtype=np.uint8)
    speed = np.zeros((height, width), dtype=np.uint8)
    for ztype, z in polys:
        mask = _points_in_polygon(gx, gy, z['polygon']).reshape(height, width)
        if ztype == 'keepout':
            keepout[mask] = 100
        else:
            pct = int(round(z['speed_pct']))
            # Overlapping speed zones: the slowest wins.
            cur = speed[mask]
            speed[mask] = np.where(cur == 0, pct, np.minimum(cur, pct))
    return keepout, speed, (float(lo[0]), float(lo[1]))


# --- derived artifacts (PGM + map_server yaml) ------------------------------------

def to_pgm(grid):
    """Binary P5 PGM: gray = 255 - round(occ*255/100); row 0 of the grid is the
    BOTTOM image row (PGM row 0 is the top), so the image is written flipped."""
    h, w = grid.shape
    gray = (255 - np.round(grid.astype(float) * 255.0 / 100.0)).astype(np.uint8)
    header = b'P5\n%d %d\n255\n' % (w, h)
    return header + gray[::-1].tobytes()


def mask_yaml(image, resolution, origin_xy, mode):
    """map_server yaml for a mask. 'trinary' (keepout: only 0/100 cells) or
    'scale' (speed: intermediate grays must survive; occupied_thresh 0.996 so
    gray 0 still reads exactly 100)."""
    if mode == 'trinary':
        thresholds = 'occupied_thresh: 0.65\nfree_thresh: 0.196'
    elif mode == 'scale':
        thresholds = 'occupied_thresh: 0.996\nfree_thresh: 0.0'
    else:
        raise ValueError('mask mode must be trinary|scale: %r' % mode)
    return ('image: %s\nmode: %s\nresolution: %g\n'
            'origin: [%.3f, %.3f, 0.0]\nnegate: 0\n%s\n'
            % (image, mode, resolution, origin_xy[0], origin_xy[1], thresholds))
