"""fleet_status tag pose setter (docker/fleet-status/tagpose.py)."""
import importlib.util
import json
import math
import pathlib
import sqlite3

import pytest

_P = pathlib.Path(__file__).resolve().parents[2] / 'docker/fleet-status/tagpose.py'
_spec = importlib.util.spec_from_file_location('fleet_tagpose', _P)
tp = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(tp)


@pytest.fixture
def env(tmp_path):
    db = tmp_path / 'tags.db'
    c = sqlite3.connect(db)
    c.execute("""CREATE TABLE tags(family TEXT NOT NULL, tag_id INTEGER NOT NULL,
        name TEXT NOT NULL UNIQUE, role TEXT NOT NULL DEFAULT '',
        size_m REAL NOT NULL DEFAULT 0.16, map_x REAL, map_y REAL, map_yaw REAL,
        last_seen TEXT, map_name TEXT, PRIMARY KEY(family, tag_id))""")
    c.execute("INSERT INTO tags(family,tag_id,name,role,last_seen)"
              " VALUES('36h11',3,'doghouse','home','t0')")
    c.commit()
    c.close()
    site = tmp_path / 'site.json'
    site.write_text(json.dumps({'active_map': 'floor1'}))
    return str(db), str(tmp_path / 'waypoints.json'), str(site)


def _call(env, body, name='doghouse'):
    db, wp, site = env
    return tp.set_pose(name, body, db, wp, site)


def test_set_writes_row_and_waypoint(env):
    code, out = _call(env, {'x': 2.0, 'y': 1.0, 'yaw': math.pi / 2})
    assert code == 200
    t = out['tag']
    assert (t['map_x'], t['map_y']) == (2.0, 1.0)
    assert t['map_name'] == 'floor1' and t['last_seen'] == 't0'
    wp = json.load(open(env[1]))['waypoints']['doghouse']
    # standoff formula == scout-skills: tag - 0.5*(cos, sin)(yaw)
    assert wp['x'] == pytest.approx(2.0, abs=1e-3)
    assert wp['y'] == pytest.approx(0.5, abs=1e-3)
    assert wp['yaw'] == pytest.approx(math.pi / 2, abs=1e-3)
    assert wp['source'] == 'manual' and wp['map'] == 'floor1'
    assert wp['saved'].endswith(' UTC')


def test_yaw_normalized(env):
    code, out = _call(env, {'x': 0, 'y': 0, 'yaw': math.pi + 1e-9})
    assert code == 200 and abs(out['tag']['map_yaw']) <= math.pi


@pytest.mark.parametrize('body', [
    {}, {'x': 1, 'y': 1}, {'x': 'a', 'y': 1, 'yaw': 0},
    {'x': float('nan'), 'y': 1, 'yaw': 0}, {'x': float('inf'), 'y': 1, 'yaw': 0},
    {'x': 1, 'y': 1, 'yaw': 4.0}, {'x': 1000, 'y': 0, 'yaw': 0},
    {'x': True, 'y': 1, 'yaw': 0}, [],
])
def test_bad_input_rejected(env, body):
    code, _ = _call(env, body)
    assert code == 400
    assert not pathlib.Path(env[1]).exists()
    row = sqlite3.connect(env[0]).execute('SELECT map_x FROM tags').fetchone()
    assert row[0] is None


def test_unknown_tag_404(env):
    code, _ = _call(env, {'x': 1, 'y': 1, 'yaw': 0}, name='nope')
    assert code == 404


def test_preserves_other_waypoints_and_no_tmp_left(env):
    pathlib.Path(env[1]).write_text(json.dumps(
        {'version': 2, 'waypoints': {'kitchen': {'x': 9, 'y': 9, 'yaw': 0}},
         'routes': {'patrol': ['kitchen']}}))
    assert _call(env, {'x': 1, 'y': 1, 'yaw': 0})[0] == 200
    store = json.load(open(env[1]))
    assert 'kitchen' in store['waypoints'] and store['routes'] == {'patrol': ['kitchen']}
    assert not pathlib.Path(env[1] + '.tmp').exists()


def test_legacy_flat_waypoints_migrated(env):
    pathlib.Path(env[1]).write_text(json.dumps({'kitchen': {'x': 9, 'y': 9, 'yaw': 0}}))
    assert _call(env, {'x': 1, 'y': 1, 'yaw': 0})[0] == 200
    store = json.load(open(env[1]))
    assert store['version'] == 2 and set(store['waypoints']) == {'kitchen', 'doghouse'}
