"""SC9: the status wire formats are FROZEN (ADR-0012/0013) — pipe AND JSON.

Exact-string assertions — consumers in webui/app.js, scout-skills server.py
and the companion recorders parse these across rosbridge/zenoh boundaries, so
any change here is a breaking protocol change and must update every consumer
in the same commit. JSON formatters serialize with sort_keys=True precisely so
these exact-string freezes hold.
"""

import pathlib

import pytest
import yaml

from scout.core import status as s

REPO = pathlib.Path(__file__).resolve().parent.parent.parent


def test_patrol_plan_exact_string():
    assert s.format_patrol_plan('scored 4 stripes') == 'plan|scored 4 stripes'


def test_patrol_idle_and_progress_exact_strings():
    assert s.format_patrol_status('idle', 5) == 'idle|5'
    assert s.format_patrol_status('driving', 5, 2) == 'driving|5|3/5'


def test_patrol_round_trip():
    assert s.parse_patrol_status('driving|5|3/5') == ('driving', 5, 3, 5)
    assert s.parse_patrol_status('idle|5') == ('idle', 5, None, None)
    assert s.parse_patrol_status('plan|text') == ('plan', None, None, None)


@pytest.mark.parametrize('state', ['driving', 'settling', 'capturing'])
def test_patrol_progress_is_one_based(state):
    assert s.format_patrol_status(state, 3, 0).endswith('|1/3')


def test_nav_state_exact_strings():
    assert s.format_nav_state('idle') == 'idle'
    assert s.format_nav_state('driving', 3.7, 1) == 'driving|3.70|1'
    # No feedback yet: the distance field is EMPTY, never a fake 0.00.
    assert s.format_nav_state('accepted') == 'accepted||0'
    assert s.format_nav_state('canceled', 0.42, 2) == 'canceled|0.42|2'


def test_nav_state_round_trip():
    assert s.parse_nav_state('driving|3.70|1') == ('driving', 3.7, 1)
    assert s.parse_nav_state('accepted||0') == ('accepted', None, 0)
    assert s.parse_nav_state('idle') == ('idle', None, None)


def test_nav_busy_states_match_profile_goal_status_names():
    # NAV_BUSY_STATES must be exactly the profile names for GoalStatus 1/2/3
    # (accepted/executing/canceling). webui/app.js keeps a literal copy for
    # its site-switch guard — the copy check below pins it.
    prof = yaml.safe_load(
        (REPO / 'scout' / 'config' / 'robot_profile.yaml').read_text())
    names = prof['robot_profile']['goal_status_names']
    assert s.NAV_BUSY_STATES == tuple(names[1:4])


def test_nav_busy_states_literal_copy_in_webui():
    # app.js cannot import Python; it carries the tuple as a JS array literal.
    app = (REPO / 'webui' / 'app.js').read_text()
    literal = "['%s']" % "', '".join(s.NAV_BUSY_STATES)
    assert literal in app, (
        'webui/app.js lost its copy of NAV_BUSY_STATES %r — the site-switch '
        'nav-busy guard parses /nav_state with it (core.status owns the '
        'grammar)' % (s.NAV_BUSY_STATES,))


# --- JSON wire formats --------------------------------------------------------


def test_flipper_status_exact_string():
    assert s.format_flipper_status('idle', True, False, False) == (
        '{"connected": true, "last_error": "", "nfc_enabled": false, '
        '"rfid_enabled": false, "state": "idle"}')


def test_flipper_status_round_trip():
    d = s.parse_flipper_status(
        s.format_flipper_status('scanning', True, True, False, 'boom'))
    assert d == {'state': 'scanning', 'connected': True, 'rfid_enabled': True,
                 'nfc_enabled': False, 'last_error': 'boom'}


def test_rfid_read_exact_string():
    assert s.format_rfid_read('EM4100', '1A2B3C4D5E', (1.5, -0.25, 0.79),
                              '2026-08-24T15:04:05Z', 'abc-123') == (
        '{"data_hex": "1A2B3C4D5E", "pose": {"x": 1.5, "y": -0.25, '
        '"yaw": 0.79}, "protocol": "EM4100", "read_id": "abc-123", '
        '"stamp_utc": "2026-08-24T15:04:05Z"}')


def test_rfid_read_null_pose_round_trip():
    d = s.parse_rfid_read(s.format_rfid_read('EM4100', 'AABB', None, 't', 'id'))
    assert d == {'read_id': 'id', 'protocol': 'EM4100', 'data_hex': 'AABB',
                 'pose': None, 'stamp_utc': 't'}


def test_nfc_read_exact_string():
    # Structural mirror of format_rfid_read; data_hex carries the tag UID.
    assert s.format_nfc_read('MIFARE Classic 1K', '04A22B5C',
                             (1.5, -0.25, 0.79),
                             '2026-08-24T15:04:05Z', 'abc-123') == (
        '{"data_hex": "04A22B5C", "pose": {"x": 1.5, "y": -0.25, '
        '"yaw": 0.79}, "protocol": "MIFARE Classic 1K", "read_id": "abc-123", '
        '"stamp_utc": "2026-08-24T15:04:05Z"}')


def test_nfc_read_null_pose_round_trip():
    d = s.parse_nfc_read(s.format_nfc_read('NTAG215', 'AABB', None, 't', 'id'))
    assert d == {'read_id': 'id', 'protocol': 'NTAG215', 'data_hex': 'AABB',
                 'pose': None, 'stamp_utc': 't'}


def test_traction_status_exact_string_and_rounding():
    m1 = {'speed': 2000.04, 'current': 1.23456, 'expected': 1.5004,
          'verdict': 'loaded', 'derate': 1.0}
    m2 = {'speed': 100.0, 'current': 0.1, 'expected': None,
          'verdict': 'below_gate', 'derate': 0.4001}
    assert s.format_traction_status(m1, m2, 'm1') == (
        '{"left": "m1", "m1": {"current": 1.235, "derate": 1.0, '
        '"expected": 1.5, "speed": 2000.0, "verdict": "loaded"}, '
        '"m2": {"current": 0.1, "derate": 0.4, "expected": null, '
        '"speed": 100.0, "verdict": "below_gate"}}')


def test_traction_status_round_trip():
    m = {'speed': 0.0, 'current': 0.0, 'expected': None,
         'verdict': 'no_data', 'derate': 1.0}
    d = s.parse_traction_status(s.format_traction_status(m, m, 'm2'))
    assert d['left'] == 'm2'
    assert d['m1']['verdict'] == 'no_data'


def test_roboclaw_status_parse():
    assert s.parse_roboclaw_status('{"main_battery": 19.1}') == {
        'main_battery': 19.1}
    for bad in ('not json', '[1, 2]', '"str"', ''):
        with pytest.raises(ValueError):
            s.parse_roboclaw_status(bad)
