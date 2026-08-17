"""The frozen `|`-delimited status wire formats (CONTEXT.md, ADR-0012/0013).

/trick_status, /follow_status and /patrol_status are stringly-typed
std_msgs/String contracts consumed across process and container boundaries
(led_status, webui/app.js, scout-skills server) — deliberately NOT .msg types.
These formatters/parsers are the single source of the grammar; test_status.py
freezes the exact strings. Change a format here and every consumer breaks
loudly in one place instead of silently on the wire.
"""


def format_follow_status(status, range_m=None, bearing_deg=None):
    """'locked|1.41|45' while tracking (range 2dp, bearing whole degrees);
    bare state string ('seeking', 'idle', ...) otherwise."""
    if status in ('locked', 'blocked') and range_m is not None:
        return '%s|%.2f|%.0f' % (status, range_m, bearing_deg)
    return status


def parse_follow_status(data):
    """(state, range_m or None, bearing_deg or None)."""
    parts = data.split('|')
    if len(parts) == 3:
        return parts[0], float(parts[1]), float(parts[2])
    return parts[0], None, None


def format_trick_status(trick=None, color=None, mode=None):
    """'idle', or 'name|#RRGGBB|mode' so led_status just renders it."""
    if trick is None:
        return 'idle'
    return '%s|%s|%s' % (trick, color, mode)


def parse_trick_status(data):
    """(name, color, mode) — color/mode default '' / 'chase' as led_status does."""
    name, color, mode = (data.split('|') + ['', 'chase'])[:3]
    return name, color, mode


def format_nav_state(status_name, distance_m=None, recoveries=0):
    """/nav_state (nav_manager, ADR-0018): bare 'idle' before any goal;
    otherwise '<status_name>|<dist 2dp or empty>|<recoveries>' — distance is
    empty (not a fake 0.00) until the first action feedback arrives."""
    if status_name == 'idle':
        return 'idle'
    dist = '' if distance_m is None else '%.2f' % distance_m
    return '%s|%s|%d' % (status_name, dist, recoveries)


def parse_nav_state(data):
    """(status_name, distance_m or None, recoveries or None)."""
    parts = data.split('|')
    if len(parts) == 1:
        return parts[0], None, None
    dist = float(parts[1]) if parts[1] else None
    return parts[0], dist, int(parts[2])


def format_patrol_plan(text):
    """'plan|<free text>' progress feedback during route planning."""
    return 'plan|%s' % text


def format_patrol_status(state, route_len, wp_index=None):
    """'idle|<route len>' when idle; '<state>|<len>|<i+1>/<len>' mid-route
    (wp_index is 0-based; the wire shows 1-based progress)."""
    if state == 'idle':
        return 'idle|%d' % route_len
    return '%s|%d|%d/%d' % (state, route_len, wp_index + 1, route_len)


def parse_patrol_status(data):
    """(state, route_len, current or None, total or None)."""
    parts = data.split('|')
    if parts[0] == 'plan':
        return 'plan', None, None, None
    if len(parts) == 2:
        return parts[0], int(parts[1]), None, None
    cur, total = parts[2].split('/')
    return parts[0], int(parts[1]), int(cur), int(total)
