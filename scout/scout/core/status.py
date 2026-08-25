"""The frozen status wire formats (CONTEXT.md, ADR-0012/0013) — pipe AND JSON.

Every status topic that crosses a process or container boundary is a
stringly-typed std_msgs/String contract (deliberately NOT .msg — ADR-0012):

  * `|`-delimited grammars: /nav_state (nav_manager), /patrol_status
    (patrol_capture) — parsed by webui/app.js and companion/inspection.
  * JSON payloads: /flipper/status, /rfid/reads (flipper_node),
    /traction/status (traction_monitor) — parsed by webui/app.js,
    scout-skills server.py and companion/rfid/recorder.py.
  * /roboclaw_status (driver-owned JSON) — parse side only, three consumers.

These formatters/parsers are the single source of every grammar; SC9
(test_status.py) freezes the exact strings and the JSON schemas. A node may
not json.dumps a status payload inline (test_conventions.py bans it) — change
a format here and every consumer breaks loudly in one place instead of
silently on the wire.

(format/parse_trick_status and _follow_status left with trick_player and
follow_me on 2026-08-24.)
"""

import json


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


# The /nav_state status names that mean "a goal is in flight" — the
# profile's goal_status_names for GoalStatus 1 (accepted), 2 (executing) and
# 3 (canceling). webui/app.js keeps a literal copy of this tuple (JS cannot
# import it); test_status.py freezes both the tuple and the profile mapping.
NAV_BUSY_STATES = ('accepted', 'driving', 'canceling')


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


# --- JSON wire formats -------------------------------------------------------
# All formatters serialize with sort_keys=True so the wire bytes are
# deterministic and SC9 can assert exact strings.


def format_flipper_status(state, connected, rfid_enabled, last_error=''):
    """/flipper/status (flipper_node, ADR-0025), latched. Consumers: the webui
    RFID badge (connected/rfid_enabled) and scout-skills' wait_rfid_read gate
    (rfid_enabled)."""
    return json.dumps({
        'state': state,
        'connected': bool(connected),
        'rfid_enabled': bool(rfid_enabled),
        'last_error': last_error,
    }, sort_keys=True)


def parse_flipper_status(data):
    """dict with keys state/connected/rfid_enabled/last_error."""
    return json.loads(data)


def format_rfid_read(protocol, data_hex, pose, stamp_utc, read_id):
    """/rfid/reads (flipper_node -> zenoh -> companion rfid_recorder,
    ADR-0025). `pose` is (x, y, yaw) or None (no map localization at read
    time — degrade, don't break)."""
    return json.dumps({
        'read_id': read_id,
        'protocol': protocol,
        'data_hex': data_hex,
        'pose': (None if pose is None
                 else {'x': pose[0], 'y': pose[1], 'yaw': pose[2]}),
        'stamp_utc': stamp_utc,
    }, sort_keys=True)


def parse_rfid_read(data):
    """dict with keys read_id/protocol/data_hex/pose/stamp_utc."""
    return json.loads(data)


def format_traction_status(m1, m2, left_channel):
    """/traction/status (traction_monitor). m1/m2 are per-channel dicts with
    keys speed (counts/s), current (A), expected (A or None), verdict,
    derate; the formatter owns the rounding. `left_channel` names which
    channel drives the left side ('m1' or 'm2')."""
    def _chan(c):
        return {
            'speed': round(c['speed'], 1),
            'current': round(c['current'], 3),
            'expected': None if c['expected'] is None else round(c['expected'], 3),
            'verdict': c['verdict'],
            'derate': round(c['derate'], 3),
        }
    return json.dumps({'m1': _chan(m1), 'm2': _chan(m2),
                       'left': left_channel}, sort_keys=True)


def parse_traction_status(data):
    """dict with keys m1/m2 (per-channel dicts) and left ('m1'|'m2')."""
    return json.loads(data)


def parse_roboclaw_status(data):
    """/roboclaw_status (roboclaw_driver's JSON String) -> dict. The one
    answer to "is this a valid driver status"; raises ValueError otherwise.
    Field extraction stays with the consumer (battery_monitor, health_monitor,
    traction_monitor) — the driver owns the schema, this owns the envelope."""
    try:
        status = json.loads(data)
    except (ValueError, TypeError) as exc:
        raise ValueError('unparseable /roboclaw_status: %s' % exc) from exc
    if not isinstance(status, dict):
        raise ValueError('/roboclaw_status is not a JSON object')
    return status
