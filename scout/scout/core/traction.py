"""Traction-derate decision logic (pure numpy, no ROS) — traction_monitor's
brain (traction spec option (a)).

Owns the calibrated-curve validation, the per-channel verdict ladder
(below_gate / uncalibrated / unloaded / loaded), the asymmetric derate walk
(fast down, slow up — PID current blips must not chatter a side back to full
speed), and the skid-steer Twist <-> per-side split the actuation path scales
through. The node keeps only I/O: parse the status, publish the outputs.
"""

import numpy as np

BELOW_GATE = 'below_gate'
UNCALIBRATED = 'uncalibrated'
UNLOADED = 'unloaded'
LOADED = 'loaded'
NO_DATA = 'no_data'


def validate_curves(speeds, m1_amps, m2_amps):
    """(curves dict or None, error str or None). [0.0] is the un-set
    placeholder (rclpy needs a typed default) — uncalibrated, not an error."""
    if speeds == [0.0] or len(speeds) < 2:
        return None, None
    if not (len(speeds) == len(m1_amps) == len(m2_amps)):
        return None, ('curve_speeds/m1_curve_amps/m2_curve_amps lengths '
                      'differ — treating as uncalibrated')
    if sorted(speeds) != speeds:
        return None, 'curve_speeds must be ascending — treating as uncalibrated'
    return {'m1': (np.asarray(speeds), np.asarray(m1_amps)),
            'm2': (np.asarray(speeds), np.asarray(m2_amps))}, None


def channel_verdict(speed, current, curve, gate_counts, margin):
    """(verdict, expected_amps or None) for one channel sample. `curve` is
    the (speeds, amps) pair for THIS channel, or None when uncalibrated.
    Below the gate there is NO verdict — current telemetry is noise below
    ~20% duty (CLAUDE.md)."""
    if speed < gate_counts:
        return BELOW_GATE, None
    if curve is None:
        return UNCALIBRATED, None
    xs, ys = curve
    expected = float(np.interp(speed, xs, ys))
    if current < (1.0 - margin) * expected:
        return UNLOADED, expected
    return LOADED, expected


def step_derate(derate, verdict, step_up, step_down, floor):
    """Walk one channel's derate for one verdict tick: UNLOADED steps down
    toward the floor; every other verdict decays back toward 1.0 (including
    BELOW_GATE — an instant reset there would oscillate trip/reset when a
    derated side falls below the gate)."""
    if verdict == UNLOADED:
        return max(floor, derate - step_down)
    return min(1.0, derate + step_up)


def split_sides(vx, wz, track):
    """Twist -> (v_left, v_right) wheel speeds for a skid-steer."""
    return vx - wz * track / 2.0, vx + wz * track / 2.0


def merge_sides(v_left, v_right, track):
    """(v_left, v_right) -> (vx, wz) — inverse of split_sides."""
    return (v_left + v_right) / 2.0, (v_right - v_left) / track
