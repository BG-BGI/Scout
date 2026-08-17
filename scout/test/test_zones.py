"""scout.core.zones — store, /zone_cmd grammar, rasterizer, artifacts (ADR-0019)."""

import numpy as np
import pytest

from scout.core import zones as z

SQUARE = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]


def _store_with(zone_name='keepout-1', ztype='keepout', poly=SQUARE, pct=None):
    store = z.blank()
    z.set_zone(store, 'house', zone_name, ztype, poly, pct)
    return store


# --- store ---------------------------------------------------------------------

def test_store_round_trip(tmp_path):
    path = str(tmp_path / 'zones.json')
    store = _store_with()
    z.set_zone(store, 'house', 'speed-1', 'speed', SQUARE, 40.0)
    z.save(path, store)
    back = z.load(path)
    assert back == store
    assert back['maps']['house']['zones']['speed-1']['speed_pct'] == 40.0


def test_load_missing_file_is_blank(tmp_path):
    assert z.load(str(tmp_path / 'nope.json')) == z.blank()


def test_set_zone_validation():
    store = z.blank()
    with pytest.raises(ValueError):
        z.set_zone(store, 'm', 'x', 'lava', SQUARE)
    with pytest.raises(ValueError):
        z.set_zone(store, 'm', 'x', 'keepout', SQUARE[:2])
    with pytest.raises(ValueError):   # speed needs a pct in (0, 100]
        z.set_zone(store, 'm', 'x', 'speed', SQUARE)
    with pytest.raises(ValueError):
        z.set_zone(store, 'm', 'x', 'speed', SQUARE, 0.0)


def test_delete_and_next_name():
    store = _store_with()
    zn = z.zones_for(store, 'house')
    assert z.next_name(zn, 'keepout') == 'keepout-2'
    assert z.next_name(zn, 'speed') == 'speed-1'
    assert z.delete_zone(store, 'house', 'keepout-1')
    assert not z.delete_zone(store, 'house', 'keepout-1')
    assert z.zones_for(store, 'house') == {}
    assert z.zones_for(store, 'other-map') == {}


# --- /zone_cmd grammar (frozen wire — crosses rosbridge) --------------------------

def test_zone_cmd_exact_strings():
    assert z.format_zone_cmd('add', 'keepout', None,
                             [[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]]) == \
        'add|keepout||1.000,2.000;3.000,4.000;5.000,6.000'
    assert z.format_zone_cmd('add', 'speed', 40.0, [[0, 0], [1, 0], [1, 1]]) \
        .startswith('add|speed|40|')
    assert z.format_zone_cmd('delete', name='keepout-1') == 'delete|keepout-1'
    assert z.format_zone_cmd('clear') == 'clear|'


def test_zone_cmd_round_trip():
    wire = z.format_zone_cmd('add', 'speed', 40.0,
                             [[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
    op, ztype, pct, poly = z.parse_zone_cmd(wire)
    assert (op, ztype, pct) == ('add', 'speed', 40.0)
    assert poly == [[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]]
    assert z.parse_zone_cmd('delete|speed-1') == ('delete', 'speed-1')
    assert z.parse_zone_cmd('clear|') == ('clear',)
    with pytest.raises(ValueError):
        z.parse_zone_cmd('bogus')


# --- rasterizer ------------------------------------------------------------------

def test_rasterize_keepout_inside_and_outside():
    keep, speed, origin = z.rasterize(
        z.zones_for(_store_with(), 'house'), resolution=0.05, pad_m=1.0)
    assert origin == (-1.0, -1.0)
    assert keep.shape == (60, 60)   # 3 m box at 0.05
    # World (0.5, 0.5) = cell center inside; (-0.5, -0.5) in the pad = outside.
    def cell(wx, wy):
        return (int((wy - origin[1]) / 0.05), int((wx - origin[0]) / 0.05))
    assert keep[cell(0.5, 0.5)] == 100
    assert keep[cell(-0.5, -0.5)] == 0
    assert speed.max() == 0


def test_rasterize_speed_overlap_keeps_slowest():
    store = z.blank()
    z.set_zone(store, 'm', 'speed-1', 'speed', SQUARE, 60.0)
    z.set_zone(store, 'm', 'speed-2', 'speed',
               [[0.5, 0.5], [1.5, 0.5], [1.5, 1.5], [0.5, 1.5]], 30.0)
    _, speed, origin = z.rasterize(z.zones_for(store, 'm'), resolution=0.05)
    def cell(wx, wy):
        return (int((wy - origin[1]) / 0.05), int((wx - origin[0]) / 0.05))
    assert speed[cell(0.25, 0.25)] == 60    # first zone only
    assert speed[cell(0.75, 0.75)] == 30    # overlap -> slowest
    assert speed[cell(1.25, 1.25)] == 30    # second zone only


def test_rasterize_empty_returns_none():
    assert z.rasterize({}) == (None, None, None)


# --- artifacts -------------------------------------------------------------------

def test_pgm_encoding_and_flip():
    grid = np.zeros((2, 3), dtype=np.uint8)
    grid[0, 0] = 100   # bottom-left in world
    data = z.to_pgm(grid)
    assert data.startswith(b'P5\n3 2\n255\n')
    pixels = data[len(b'P5\n3 2\n255\n'):]
    # PGM row 0 is the TOP image row = grid's LAST row; occ 100 -> gray 0.
    assert pixels == bytes([255, 255, 255, 0, 255, 255])


def test_mask_yaml_modes():
    y_keep = z.mask_yaml('zone_keepout.pgm', 0.05, (-1.0, -2.0), 'trinary')
    assert 'mode: trinary' in y_keep and 'origin: [-1.000, -2.000, 0.0]' in y_keep
    y_speed = z.mask_yaml('zone_speed.pgm', 0.05, (0.0, 0.0), 'scale')
    # occupied_thresh 0.996: gray 0 must still read exactly 100.
    assert 'mode: scale' in y_speed and 'occupied_thresh: 0.996' in y_speed
    with pytest.raises(ValueError):
        z.mask_yaml('x.pgm', 0.05, (0, 0), 'blend')
