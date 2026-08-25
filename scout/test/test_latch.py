"""scout.core.latch — the shared entry/exit/dwell latch (1:1 with core/latch.py)."""

from scout.core.latch import Latch


def test_immediate_enter_and_leave():
    latch = Latch()
    assert latch.update(True, False) is True
    assert latch.update(False, True) is False


def test_stays_off_without_enter():
    latch = Latch()
    assert latch.update(False, False) is False
    assert latch.state is False


def test_enter_dwell_requires_continuous_hold():
    # tilt_monitor's shape: abort only after hold_s over the threshold.
    latch = Latch(on_dwell=0.5)
    assert latch.update(True, False, now=0.0) is False
    assert latch.update(True, False, now=0.4) is False
    assert latch.update(True, False, now=0.5) is True


def test_enter_dwell_resets_on_gap():
    # A spin-gated sample (enter=False) must restart the accumulation.
    latch = Latch(on_dwell=0.5)
    latch.update(True, False, now=0.0)
    latch.update(False, False, now=0.3)   # gate: reset
    assert latch.update(True, False, now=0.6) is False
    assert latch.update(True, False, now=1.1) is True


def test_exit_dwell_requires_continuous_hold():
    # collision_polygon_manager's shape: leave 'turning' only after the
    # exit condition holds for the dwell.
    latch = Latch(off_dwell=0.3)
    latch.update(True, False, now=0.0)
    assert latch.update(False, True, now=1.0) is True
    assert latch.update(False, True, now=1.2) is True
    assert latch.update(False, True, now=1.3) is False


def test_exit_dwell_resets_when_condition_breaks():
    latch = Latch(off_dwell=0.3)
    latch.update(True, False, now=0.0)
    latch.update(False, True, now=1.0)
    latch.update(False, False, now=1.2)   # back above exit: reset
    assert latch.update(False, True, now=1.4) is True
    assert latch.update(False, True, now=1.7) is False


def test_value_hysteresis_shape():
    # led_status's shape: on at v <= 16.5, off only above 16.9.
    crit, hyst = 16.5, 0.4
    latch = Latch()
    for v, want in ((17.0, False), (16.5, True), (16.7, True),
                    (16.89, True), (16.91, False)):
        assert latch.update(v <= crit, v > crit + hyst) is want, v


def test_initial_state_can_be_true():
    latch = Latch(state=True)
    assert latch.state is True
    assert latch.update(False, True) is False
