"""Resting-voltage state-of-charge estimation for the 5s li-ion DeWalt pack.

Extracted from battery_monitor so the curve interpolation and the rest-gated
median estimator are testable without a ROS clock: the estimator takes an
explicit timestamp, so a test (or a bag replay) can drive it deterministically.

The voltage THRESHOLDS (warn / critical / activity-floor) are deliberately NOT
here — they are cross-surface values (webui badge, led_status, trick/patrol
gating) and live in robot_profile.yaml so every surface agrees. This module is
the home of the pure charge-curve logic only.
"""

import statistics

# Resting pack voltage -> fraction of charge. 4.20 V/cell full, 3.20 V/cell at
# the RoboClaw's 16.0 V Min Main cutoff. The middle rows are soft because a
# li-ion curve is genuinely flat there.
DEFAULT_CURVE_VOLTS = [16.0, 17.0, 18.0, 18.5, 19.0, 20.0, 21.0]
DEFAULT_CURVE_FRACTION = [0.0, 0.12, 0.30, 0.45, 0.60, 0.85, 1.00]


def validate_curve(volts, fraction):
    """Return (volts, fraction) unchanged, or raise ValueError describing the
    problem. The node catches this, logs, and falls back to the default curve."""
    if len(volts) != len(fraction):
        raise ValueError('curve_volts and curve_fraction differ in length')
    if len(volts) < 2:
        raise ValueError('the curve needs at least two points')
    if any(b <= a for a, b in zip(volts, volts[1:])):
        raise ValueError('curve_volts must be strictly ascending')
    return volts, fraction


def fraction_at(curve_v, curve_f, volts):
    """Piecewise-linear charge fraction at `volts`, clamped to the curve ends."""
    if volts <= curve_v[0]:
        return curve_f[0]
    if volts >= curve_v[-1]:
        return curve_f[-1]
    for i in range(1, len(curve_v)):
        if volts <= curve_v[i]:
            span = (volts - curve_v[i - 1]) / (curve_v[i] - curve_v[i - 1])
            return curve_f[i - 1] + span * (curve_f[i] - curve_f[i - 1])
    return curve_f[-1]


class RestingSocEstimator:
    """Rest-gated, per-period median SoC estimate. Replayable: feed (t, volts,
    speed) samples with `t` in seconds (any monotonic clock); update() returns a
    NEW fraction estimate when one is produced this call, else None.

    Motion clears pending samples (they were measured against a different load
    history); a sample only counts after the pack has been at rest for
    `rest_seconds`; estimates are recomputed at most once per `estimate_period`
    from the median of the samples gathered since the last one.
    """

    def __init__(self, curve_v, curve_f, *, rest_speed_counts=50.0,
                 rest_seconds=3.0, estimate_period=5.0):
        self._curve_v = curve_v
        self._curve_f = curve_f
        self._rest_speed_counts = rest_speed_counts
        self._rest_seconds = rest_seconds
        self._estimate_period = estimate_period
        self._rest_since = None
        self._samples = []
        self._last_estimate_at = None
        self.estimate = None  # last fraction produced, or None until the first

    def update(self, t, volts, speed):
        if speed > self._rest_speed_counts:
            self._rest_since = None
            self._samples.clear()
            return None
        if self._rest_since is None:
            self._rest_since = t
            return None
        if t - self._rest_since < self._rest_seconds:
            return None
        self._samples.append(volts)
        due = (self._last_estimate_at is None
               or t - self._last_estimate_at >= self._estimate_period)
        if not due:
            return None
        self._last_estimate_at = t
        self.estimate = fraction_at(
            self._curve_v, self._curve_f, statistics.median(self._samples))
        self._samples.clear()
        return self.estimate
