"""scout.core.collision — zone truth table (1:1 with core/collision.py).

The triple is (front_enabled, rear_enabled, turn_enabled) pushed into
collision_monitor's parameters. Exactly one polygon is enabled while guarded;
zero while bypassed — frozen here because a wrong pair means either a blind
direction or a re-fused stop flap (ADR-0016 addendum).
"""

import itertools

from scout.core.collision import (
    FORWARD,
    REVERSE,
    TURN,
    desired_zone_state,
    zone_mode,
)


def test_mode_precedence_turn_wins():
    assert zone_mode(False, False) == FORWARD
    assert zone_mode(False, True) == REVERSE
    assert zone_mode(True, False) == TURN
    assert zone_mode(True, True) == TURN


def test_forward_guards_front_only():
    assert desired_zone_state(False, False, False) == (True, False, False)


def test_reverse_guards_rear_only():
    assert desired_zone_state(False, False, True) == (False, True, False)


def test_turn_guards_full_box_regardless_of_reverse():
    assert desired_zone_state(False, True, False) == (False, False, True)
    assert desired_zone_state(False, True, True) == (False, False, True)


def test_bypass_disables_all_three_in_every_mode():
    for turning, reversing in itertools.product((False, True), repeat=2):
        assert desired_zone_state(True, turning, reversing) == (
            False, False, False)


def test_exactly_one_polygon_enabled_when_guarded():
    for turning, reversing in itertools.product((False, True), repeat=2):
        assert sum(desired_zone_state(False, turning, reversing)) == 1
