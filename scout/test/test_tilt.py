"""scout.core.tilt — tilt tracking + abort decision (1:1 with core/tilt.py).

Frame convention under test: the optical level axis (0, -1, 0) — level means
gravity reads on -Y (CLAUDE.md: the D455's Y axis points down).
"""

import math

from scout.core.tilt import ABORT, WARN, TiltTracker

LEVEL = (0.0, -1.0, 0.0)
G = 9.81
STILL = (0.0, 0.0, 0.0)


def _tracker(**kw):
    args = dict(level_axis=LEVEL, warn_deg=8.0, abort_deg=15.0,
                stillness_gyro=0.08, hold_s=0.5, lpf_alpha=1.0)
    args.update(kw)
    return TiltTracker(**args)


def _accel_at(deg):
    # Tip about the optical X axis: gravity swings from -Y toward +Z.
    rad = math.radians(deg)
    return (0.0, -G * math.cos(rad), G * math.sin(rad))


def test_level_reads_zero_tilt():
    t = _tracker()
    t.update(_accel_at(0.0), STILL, 0.0)
    assert abs(t.tilt_deg) < 1e-6


def test_tilt_angle_matches_geometry():
    t = _tracker()
    t.update(_accel_at(12.0), STILL, 0.0)
    assert abs(t.tilt_deg - 12.0) < 1e-6


def test_warn_fires_once_and_rearms():
    t = _tracker()
    assert t.update(_accel_at(10.0), STILL, 0.0) == WARN
    assert t.update(_accel_at(10.0), STILL, 0.1) is None      # no repeat
    t.update(_accel_at(2.0), STILL, 0.2)                       # below: re-arm
    assert t.update(_accel_at(10.0), STILL, 0.3) == WARN


def test_abort_requires_hold():
    t = _tracker()
    assert t.update(_accel_at(20.0), STILL, 0.0) in (WARN, None)
    assert t.update(_accel_at(20.0), STILL, 0.4) is None
    assert t.update(_accel_at(20.0), STILL, 0.5) == ABORT
    assert t.latched


def test_latched_tracker_goes_silent():
    t = _tracker()
    t.update(_accel_at(20.0), STILL, 0.0)
    t.update(_accel_at(20.0), STILL, 0.6)
    assert t.latched
    assert t.update(_accel_at(20.0), STILL, 1.0) is None


def test_spinning_resets_abort_dwell():
    # A pivot mid-excursion must restart the hold clock (accel is corrupted
    # by centripetal terms while spinning).
    t = _tracker()
    spin = (0.0, 2.5, 0.0)
    t.update(_accel_at(20.0), STILL, 0.0)
    assert t.update(_accel_at(20.0), spin, 0.3) is None       # gated
    assert t.update(_accel_at(20.0), STILL, 0.6) is None      # dwell restarted
    assert t.update(_accel_at(20.0), STILL, 1.1) == ABORT


def test_freefall_sample_ignored():
    t = _tracker()
    assert t.update((0.0, -0.2, 0.0), STILL, 0.0) is None
    assert t.tilt_deg is None


def test_lpf_smooths_a_spike():
    t = _tracker(lpf_alpha=0.2)
    t.update(_accel_at(0.0), STILL, 0.0)
    t.update(_accel_at(20.0), STILL, 0.1)   # one-sample spike
    assert t.tilt_deg < 5.0                 # alpha 0.2: moves only 4 deg
