import pytest

from scout.core import tricks as t

CAPS = dict(max_linear=1.0, max_angular=3.0, min_pivot_rate=0.35)


def test_shipped_tricks_validate():
    t.validate_tricks(t.TRICKS, t.TRICK_LED, **CAPS)


def test_trick_led_complete():
    assert set(t.TRICKS) <= set(t.TRICK_LED)


@pytest.mark.parametrize('seg', [
    (0.0, 0.0, 1.0),   # duration <= 0
    (1.0, 2.0, 0.0),   # |vx| > cap
    (1.0, 0.0, 5.0),   # |wz| > cap
    (1.0, 0.0, 0.10),  # pivot under the floor
])
def test_each_violation_raises(seg):
    with pytest.raises(ValueError):
        t.validate_tricks({'x': [seg]}, {'x': ('', 'chase')}, **CAPS)


def test_missing_led_entry_raises():
    with pytest.raises(ValueError):
        t.validate_tricks({'x': [(1.0, 0.0, 1.0)]}, {}, **CAPS)
