"""Diagnostic-level decisions for the health_monitor aggregator (pure).

Extracted so the OK/WARN/ERROR/STALE calls are testable without a ROS clock or
diagnostic_msgs: the node hands over the ages and readings it has already
collected, these return a (level, message) pair, and the node maps `level`
straight onto diagnostic_msgs/DiagnosticStatus — whose OK/WARN/ERROR/STALE bytes
are these same 0/1/2/3 values (the node asserts that at construction).

Thresholds are NOT owned here. The battery warn/critical volts are cross-surface
(robot_profile.yaml, shared with battery_monitor and led_status); the staleness
timeouts are per-node tunables. Both are injected by the caller — the same
contract as the rest of scout.core.
"""

import math

# diagnostic_msgs/DiagnosticStatus byte values, redeclared so this module stays
# ROS-free. health_monitor asserts these still match upstream.
OK = 0
WARN = 1
ERROR = 2
STALE = 3


def worst(levels):
    """The dominating level of a set (empty -> OK). STALE outranks ERROR: a
    subsystem that stopped reporting is a harder failure than one reporting a
    fault, and the byte values already order it that way."""
    return max(levels, default=OK)


def staleness_level(age_s, timeout_s, label):
    """STALE once no message has arrived for `timeout_s` (or none ever, when
    `age_s` is None) — the freshness gate every subsystem passes before its
    value is trusted."""
    if age_s is None or age_s > timeout_s:
        seen = 'ever' if age_s is None else '%.1f s' % age_s
        return STALE, '%s: no data for %s' % (label, seen)
    return OK, '%s: fresh (%.1f s)' % (label, age_s)


def battery_level(present, volts, percentage, warn_v, crit_v):
    """Pack health from resting voltage, mirroring battery_monitor's ladder:
    at/below critical is ERROR, at/below warn is WARN, else OK (thresholds
    inclusive). `percentage` (a fraction, or NaN until the first resting
    estimate) is appended to the message only when known."""
    if not present:
        return STALE, 'battery: not present'
    pct = '' if percentage is None or math.isnan(percentage) \
        else ' (%.0f%%)' % (100.0 * percentage)
    if volts <= crit_v:
        return ERROR, 'battery: %.2f V CRITICAL%s' % (volts, pct)
    if volts <= warn_v:
        return WARN, 'battery: %.2f V low%s' % (volts, pct)
    return OK, 'battery: %.2f V%s' % (volts, pct)


def tilt_level(alarm):
    """The tilt_monitor latch: True means the chassis tipped past the abort
    angle and navigation was cancelled."""
    if alarm:
        return ERROR, 'tilt: ABORT latched (chassis tipped)'
    return OK, 'tilt: level'


def traction_level(m1_verdict, m2_verdict, derate_left, derate_right):
    """traction_monitor's parsed /traction/status. A derated side is WARN
    (performance, not safety — the CM and estop own safety); uncalibrated is
    OK-with-a-note so a fresh install isn't permanently yellow."""
    if min(derate_left, derate_right) < 1.0:
        return WARN, ('traction: derated L=%.2f R=%.2f (m1 %s, m2 %s)'
                      % (derate_left, derate_right, m1_verdict, m2_verdict))
    if 'uncalibrated' in (m1_verdict, m2_verdict):
        return OK, 'traction: uncalibrated (passthrough only)'
    return OK, 'traction: full drive (m1 %s, m2 %s)' % (m1_verdict, m2_verdict)


def bypass_level(bypassed, zone_mode):
    """collision_polygon_manager's latched surfaces: a bypassed collision
    monitor is deliberate but must stay loudly visible until auto-release."""
    if bypassed:
        return WARN, 'collision: STOP ZONES BYPASSED (auto-release pending)'
    return OK, 'collision: guarded (%s zone)' % zone_mode


def flipper_level(connected, rfid_enabled, last_error, nfc_enabled=False):
    """flipper_node's latched /flipper/status. No Flipper attached is NORMAL
    (tier-2 peripheral) — only a fault while one was in use is WARN. RFID and
    NFC are mutually exclusive; either one on reads as 'scanning'."""
    if connected:
        return OK, ('flipper: scanning' if (rfid_enabled or nfc_enabled)
                    else 'flipper: idle')
    if last_error:
        return WARN, 'flipper: disconnected (%s)' % last_error
    return OK, 'flipper: not attached'


def cliff_level(stop_points):
    """cliff_detector's /cliff/stop_points width. Nonzero means a remembered
    ledge sits in the forward stop corridor and the CM is holding the robot —
    WARN so the operator sees WHY it stopped. The freshness gate above this is
    the real safeguard: cliff_detector goes deliberately silent on camera/TF
    loss, so STALE here means driving blind toward ledges (ADR-0024)."""
    if stop_points:
        return WARN, 'cliff: ledge in stop corridor (%d pts)' % stop_points
    return OK, 'cliff: clear'
