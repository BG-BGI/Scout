import json

import pytest

from scout.core import waypoints as w


def test_migrate_v2_roundtrips():
    store = {'version': 2,
             'waypoints': {'a': {'x': 1.0, 'y': 2.0, 'yaw': 0.0}},
             'routes': {'patrol': ['a']}}
    assert w.migrate(store) == store


def test_migrate_legacy_flat_waypoints():
    flat = {'kitchen': {'x': 1.2, 'y': 3.4, 'yaw': 0.5, 'source': 'operator'}}
    out = w.migrate(flat)
    assert out['version'] == 2
    assert out['waypoints']['kitchen']['x'] == 1.2
    assert out['routes'] == {}


def test_migrate_legacy_patrol_route_yaml_shape():
    legacy = {'waypoints': [{'x': 1.0, 'y': 0.0, 'yaw': 0.0},
                            {'x': 2.0, 'y': 0.0}]}   # yaw optional
    out = w.migrate(legacy)
    assert out['waypoints'] == {}
    assert out['routes']['patrol'] == [
        {'x': 1.0, 'y': 0.0, 'yaw': 0.0}, {'x': 2.0, 'y': 0.0, 'yaw': 0.0}]


def test_set_waypoint_map_stamp():
    # ADR-0029: `map` stamps which site map the pose belongs to; absent =
    # legacy = assume the active map.
    store = w.blank()
    w.set_waypoint(store, 'a', (1.0, 2.0, 0.5), 'operator', map='floor1')
    w.set_waypoint(store, 'b', (0.0, 0.0, 0.0), 'operator')
    assert store['waypoints']['a']['map'] == 'floor1'
    assert 'map' not in store['waypoints']['b']


def test_migrate_preserves_map_key():
    store = {'version': 2,
             'waypoints': {'a': {'x': 1.0, 'y': 2.0, 'yaw': 0.0,
                                 'map': 'floor2'}},
             'routes': {}}
    assert w.migrate(store)['waypoints']['a']['map'] == 'floor2'


def test_resolve_route_names_and_inline():
    store = w.blank()
    w.set_waypoint(store, 'a', (1.0, 2.0, 0.5), 'operator')
    store['routes']['r'] = ['a', {'x': 9.0, 'y': 8.0, 'yaw': 1.0}]
    poses = w.resolve_route(store, 'r')
    assert poses == [{'x': 1.0, 'y': 2.0, 'yaw': 0.5},
                     {'x': 9.0, 'y': 8.0, 'yaw': 1.0}]


def test_resolve_route_missing_raises():
    store = w.blank()
    store['routes']['r'] = ['ghost']
    with pytest.raises(KeyError):
        w.resolve_route(store, 'r')
    with pytest.raises(KeyError):
        w.resolve_route(store, 'nope')


def test_save_load_roundtrip_atomic(tmp_path):
    store = w.blank()
    w.set_waypoint(store, 'a', (1.0, 2.0, 0.5), 'mark', saved='2026-08-15')
    path = str(tmp_path / 'sub' / 'waypoints.json')
    w.save(path, store)
    assert not (tmp_path / 'sub' / 'waypoints.json.tmp').exists()
    reloaded = w.load(path)
    assert reloaded['waypoints']['a'] == {
        'x': 1.0, 'y': 2.0, 'yaw': 0.5, 'source': 'mark', 'saved': '2026-08-15'}


def test_load_missing_is_blank(tmp_path):
    assert w.load(str(tmp_path / 'nope.json')) == w.blank()


def test_load_tolerates_legacy_flat_file(tmp_path):
    path = tmp_path / 'waypoints.json'
    path.write_text(json.dumps({'k': {'x': 1.0, 'y': 2.0, 'yaw': 0.0}}))
    assert w.load(str(path))['waypoints']['k']['x'] == 1.0
