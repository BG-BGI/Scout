import math

import pytest

from scout.core import battery as b


def test_validate_curve_rejects_bad_input():
    with pytest.raises(ValueError):
        b.validate_curve([1, 2, 3], [0, 1])          # length mismatch
    with pytest.raises(ValueError):
        b.validate_curve([1], [0])                   # too short
    with pytest.raises(ValueError):
        b.validate_curve([1, 1, 2], [0, 0.5, 1])     # not strictly ascending
    v, f = b.validate_curve([1, 2], [0, 1])
    assert v == [1, 2] and f == [0, 1]


def test_fraction_at_endpoints_and_interp():
    v, f = b.DEFAULT_CURVE_VOLTS, b.DEFAULT_CURVE_FRACTION
    assert b.fraction_at(v, f, 10.0) == 0.0           # below empty -> 0
    assert b.fraction_at(v, f, 25.0) == 1.0           # above full -> 1
    assert b.fraction_at(v, f, 16.0) == 0.0
    assert b.fraction_at(v, f, 21.0) == 1.0
    # Midpoint of the 18.0->18.5 segment (0.30 -> 0.45).
    assert math.isclose(b.fraction_at(v, f, 18.25), 0.375, abs_tol=1e-9)


def _est():
    return b.RestingSocEstimator(
        b.DEFAULT_CURVE_VOLTS, b.DEFAULT_CURVE_FRACTION,
        rest_speed_counts=50.0, rest_seconds=3.0, estimate_period=5.0)


def test_estimator_requires_rest_then_produces():
    e = _est()
    # Moving: never estimates.
    assert e.update(0.0, 19.0, 200.0) is None
    # At rest but not long enough.
    assert e.update(1.0, 19.0, 0.0) is None          # rest starts here
    assert e.update(3.0, 19.0, 0.0) is None          # only 2 s of rest
    # Past rest_seconds -> first estimate.
    est = e.update(4.5, 19.0, 0.0)
    assert est is not None and math.isclose(est, 0.60, abs_tol=1e-9)


def test_estimator_motion_clears_samples():
    e = _est()
    e.update(0.0, 19.0, 0.0)          # rest starts
    e.update(4.0, 19.0, 0.0)          # first estimate at ~0.60
    moving = e.update(5.0, 17.0, 300.0)
    assert moving is None
    # After motion, the rest clock restarts; no estimate until rest_seconds pass.
    assert e.update(6.0, 17.0, 0.0) is None
    assert e.update(9.5, 17.0, 0.0) == pytest.approx(0.12)


def test_estimator_medians_out_quantization_dither():
    e = _est()
    e.update(0.0, 19.0, 0.0)          # rest starts
    # Gather dithered samples within one estimate_period; the estimate at the
    # period boundary is the MEDIAN, not the last flickered value.
    e.update(4.0, 19.0, 0.0)          # first estimate (period boundary)
    for t, v in ((4.5, 19.1), (5.0, 18.9), (5.5, 19.1)):
        e.update(t, v, 0.0)
    est = e.update(9.5, 18.9, 0.0)    # next period: median of [19.1,18.9,19.1,18.9]=19.0
    assert est == pytest.approx(b.fraction_at(
        b.DEFAULT_CURVE_VOLTS, b.DEFAULT_CURVE_FRACTION, 19.0))
