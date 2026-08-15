"""SC9: the |-delimited status wire formats are FROZEN (ADR-0012/0013).

Exact-string assertions — consumers in led_status, webui/app.js and
scout-skills parse these across a rosbridge boundary, so any change here is a
breaking protocol change and must update every consumer in the same commit.
"""

import pytest

from scout.core import status as s


def test_follow_locked_exact_string():
    assert s.format_follow_status('locked', 1.414, 45.0) == 'locked|1.41|45'


def test_follow_blocked_rounds_bearing_to_whole_degrees():
    assert s.format_follow_status('blocked', 0.5, -12.6) == 'blocked|0.50|-13'


def test_follow_bare_states_pass_through():
    assert s.format_follow_status('seeking') == 'seeking'
    assert s.format_follow_status('idle') == 'idle'


def test_follow_round_trip():
    state, rng, brg = s.parse_follow_status('locked|1.41|45')
    assert (state, rng, brg) == ('locked', 1.41, 45.0)
    assert s.parse_follow_status('idle') == ('idle', None, None)


def test_trick_idle_and_active_exact_strings():
    assert s.format_trick_status() == 'idle'
    assert s.format_trick_status('spin', '#FF8800', 'chase') == 'spin|#FF8800|chase'


def test_trick_parse_defaults_match_led_status():
    assert s.parse_trick_status('spin|#FF8800|chase') == ('spin', '#FF8800', 'chase')
    assert s.parse_trick_status('spin') == ('spin', '', 'chase')


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
