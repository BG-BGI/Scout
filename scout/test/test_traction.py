"""scout.core.traction — verdicts, derate walk, side split (1:1 with core/traction.py)."""

import pytest

from scout.core import traction as t

CURVE = ([1000.0, 3000.0, 5000.0], [1.0, 2.0, 3.0])
GATE = 1850.0
MARGIN = 0.35


# --- validate_curves ---------------------------------------------------------

def test_placeholder_is_uncalibrated_not_error():
    curves, err = t.validate_curves([0.0], [0.0], [0.0])
    assert curves is None and err is None


def test_single_point_is_uncalibrated():
    curves, err = t.validate_curves([1000.0], [1.0], [1.0])
    assert curves is None and err is None


def test_length_mismatch_is_an_error():
    curves, err = t.validate_curves([1.0, 2.0], [1.0], [1.0, 2.0])
    assert curves is None and 'lengths differ' in err


def test_unsorted_speeds_is_an_error():
    curves, err = t.validate_curves([2.0, 1.0], [1.0, 2.0], [1.0, 2.0])
    assert curves is None and 'ascending' in err


def test_valid_curves_per_channel():
    curves, err = t.validate_curves(*CURVE, [2.0, 4.0, 6.0])
    assert err is None
    assert set(curves) == {'m1', 'm2'}


# --- channel_verdict ---------------------------------------------------------

def test_below_gate_no_verdict():
    assert t.channel_verdict(500.0, 9.9, CURVE, GATE, MARGIN) == (
        t.BELOW_GATE, None)


def test_uncalibrated_above_gate():
    assert t.channel_verdict(3000.0, 1.0, None, GATE, MARGIN) == (
        t.UNCALIBRATED, None)


def test_loaded_at_expected_current():
    verdict, expected = t.channel_verdict(3000.0, 2.0, CURVE, GATE, MARGIN)
    assert verdict == t.LOADED and expected == 2.0


def test_unloaded_below_margin():
    # expected 2.0 A at 3000 counts/s; margin 0.35 -> trip below 1.3 A.
    verdict, expected = t.channel_verdict(3000.0, 1.29, CURVE, GATE, MARGIN)
    assert verdict == t.UNLOADED and expected == 2.0
    verdict, _ = t.channel_verdict(3000.0, 1.31, CURVE, GATE, MARGIN)
    assert verdict == t.LOADED


def test_interpolation_between_points():
    _, expected = t.channel_verdict(2000.0, 9.9, CURVE, GATE, MARGIN)
    assert expected == pytest.approx(1.5)


# --- step_derate -------------------------------------------------------------

def test_walk_down_fast_up_slow():
    d = t.step_derate(1.0, t.UNLOADED, 0.02, 0.15, 0.4)
    assert d == pytest.approx(0.85)
    d = t.step_derate(d, t.LOADED, 0.02, 0.15, 0.4)
    assert d == pytest.approx(0.87)


def test_walk_clamps_at_floor_and_one():
    assert t.step_derate(0.45, t.UNLOADED, 0.02, 0.15, 0.4) == 0.4
    assert t.step_derate(0.99, t.LOADED, 0.02, 0.15, 0.4) == 1.0


def test_below_gate_decays_up_not_snap():
    assert t.step_derate(0.4, t.BELOW_GATE, 0.02, 0.15, 0.4) == pytest.approx(0.42)


# --- side split --------------------------------------------------------------

def test_split_merge_round_trip():
    vl, vr = t.split_sides(0.5, 1.0, 0.278)
    assert (vl, vr) == pytest.approx((0.5 - 0.139, 0.5 + 0.139))
    assert t.merge_sides(vl, vr, 0.278) == pytest.approx((0.5, 1.0))


def test_pure_pivot_splits_antisymmetric():
    vl, vr = t.split_sides(0.0, 2.0, 0.278)
    assert vl == pytest.approx(-vr)


def test_one_sided_derate_creates_yaw():
    # Left side derated to 0.5 while commanding straight: the recomposed
    # Twist must slow down AND turn toward the derated side (positive wz =
    # CCW = toward the left wheel that lost speed).
    vl, vr = t.split_sides(0.6, 0.0, 0.278)
    vx, wz = t.merge_sides(vl * 0.5, vr * 1.0, 0.278)
    assert vx == pytest.approx(0.45)
    assert wz > 0.0
