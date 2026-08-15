import math

from scout.core import health as h


def test_level_bytes_match_diagnostic_status():
    # These MUST stay equal to diagnostic_msgs/DiagnosticStatus OK/WARN/ERROR/
    # STALE; health_monitor re-asserts it against the real message at startup.
    assert (h.OK, h.WARN, h.ERROR, h.STALE) == (0, 1, 2, 3)


def test_worst_dominates_and_defaults_ok():
    assert h.worst([]) == h.OK
    assert h.worst([h.OK, h.WARN, h.OK]) == h.WARN
    assert h.worst([h.WARN, h.ERROR]) == h.ERROR
    assert h.worst([h.ERROR, h.STALE]) == h.STALE   # STALE outranks ERROR


def test_staleness_gate():
    assert h.staleness_level(0.5, 3.0, 'x')[0] == h.OK
    lvl, msg = h.staleness_level(9.0, 3.0, 'x')
    assert lvl == h.STALE and 'no data' in msg
    lvl, msg = h.staleness_level(None, 3.0, 'x')
    assert lvl == h.STALE and 'ever' in msg


def test_battery_ladder():
    assert h.battery_level(True, 19.0, 0.6, 17.5, 16.5)[0] == h.OK
    assert h.battery_level(True, 17.0, 0.2, 17.5, 16.5)[0] == h.WARN
    assert h.battery_level(True, 16.0, 0.05, 17.5, 16.5)[0] == h.ERROR
    assert h.battery_level(False, 0.0, math.nan, 17.5, 16.5)[0] == h.STALE


def test_battery_thresholds_are_inclusive():
    assert h.battery_level(True, 16.5, 0.1, 17.5, 16.5)[0] == h.ERROR
    assert h.battery_level(True, 17.5, 0.2, 17.5, 16.5)[0] == h.WARN


def test_battery_message_omits_unknown_percentage():
    _, msg = h.battery_level(True, 19.0, math.nan, 17.5, 16.5)
    assert '%' not in msg
    _, msg = h.battery_level(True, 19.0, 0.6, 17.5, 16.5)
    assert '60%' in msg


def test_tilt_latch():
    assert h.tilt_level(False)[0] == h.OK
    assert h.tilt_level(True)[0] == h.ERROR
