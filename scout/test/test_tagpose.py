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


# --- U11: poses are per map -------------------------------------------------------

def _poses(db_path):
    c = sqlite3.connect(db_path)
    try:
        return {(r[0], r[1]): r[2:] for r in c.execute(
            'SELECT tag_id, map_name, map_x, map_y, map_yaw FROM tag_poses')
            for r in [(r[0], r[1], r[2], r[3], r[4])]}
    finally:
        c.close()


def _activate(env, name):
    pathlib.Path(env[2]).write_text(json.dumps({'active_map': name}))


def test_set_pose_writes_the_active_maps_row_and_migrates_the_file(env):
    assert _call(env, {'x': 2.0, 'y': 1.0, 'yaw': 0.5})[0] == 200
    assert _poses(env[0]) == {(3, 'floor1'): (2.0, 1.0, 0.5)}
    c = sqlite3.connect(env[0])
    assert c.execute('PRAGMA user_version').fetchone()[0] == tp.SCHEMA_VERSION
    assert c.execute('SELECT map_x FROM tags').fetchone()[0] is None   # legacy frozen


def test_surveying_on_map_b_leaves_map_a_untouched(env):
    assert _call(env, {'x': 2.0, 'y': 1.0, 'yaw': 0.5})[0] == 200
    _activate(env, 'floor1_plan')
    code, out = _call(env, {'x': 20.0, 'y': 10.0, 'yaw': -1.0})
    assert code == 200
    assert _poses(env[0]) == {(3, 'floor1'): (2.0, 1.0, 0.5),
                              (3, 'floor1_plan'): (20.0, 10.0, -1.0)}
    t = out['tag']
    assert (t['map_x'], t['map_name']) == (20.0, 'floor1_plan')       # the active pose
    assert set(t['poses']) == {'floor1', 'floor1_plan'}
    assert t['poses']['floor1']['x'] == 2.0


def test_resetting_a_pose_on_the_same_map_replaces_only_that_row(env):
    _call(env, {'x': 2.0, 'y': 1.0, 'yaw': 0.5})
    _activate(env, 'floor1_plan')
    _call(env, {'x': 20.0, 'y': 10.0, 'yaw': -1.0})
    _activate(env, 'floor1')
    assert _call(env, {'x': 3.0, 'y': 3.0, 'yaw': 0.0})[0] == 200
    assert _poses(env[0]) == {(3, 'floor1'): (3.0, 3.0, 0.0),
                              (3, 'floor1_plan'): (20.0, 10.0, -1.0)}


@pytest.mark.parametrize('site', [None, {}, {'active_map': None},
                                  {'active_map': ['x']}, '{bad json'])
def test_no_usable_active_map_refuses_and_changes_nothing(env, site):
    p = pathlib.Path(env[2])
    if site is None:
        p.unlink()
    else:
        p.write_text(site if isinstance(site, str) else json.dumps(site))
    code, out = _call(env, {'x': 1, 'y': 1, 'yaw': 0})
    assert code == 409 and 'active map' in out['error']
    assert not pathlib.Path(env[1]).exists()
    c = sqlite3.connect(env[0])
    tables = {r[0] for r in c.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    assert 'tag_poses' not in tables or c.execute(
        'SELECT COUNT(*) FROM tag_poses').fetchone()[0] == 0


def test_first_waypoint_reports_nothing_replaced(env):
    code, out = _call(env, {'x': 2.0, 'y': 1.0, 'yaw': 0.0})
    assert code == 200 and out['replaced_waypoint'] is None


def test_waypoint_overwrite_names_the_map_it_replaced(env):
    _call(env, {'x': 2.0, 'y': 1.0, 'yaw': 0.0})                   # floor1's waypoint
    _activate(env, 'floor1_plan')
    code, out = _call(env, {'x': 20.0, 'y': 10.0, 'yaw': 0.0})
    assert code == 200
    rep = out['replaced_waypoint']
    assert rep['map'] == 'floor1' and rep['x'] == pytest.approx(1.5, abs=1e-3)
    assert rep['source'] == 'manual'
    wp = json.load(open(env[1]))['waypoints']['doghouse']
    assert wp['map'] == 'floor1_plan' and wp['x'] == pytest.approx(19.5, abs=1e-3)


def test_waypoint_never_pairs_one_maps_coordinates_with_another_maps_name(env):
    for active, x in (('floor1', 2.0), ('floor1_plan', 20.0), ('floor1', 4.0)):
        _activate(env, active)
        assert _call(env, {'x': x, 'y': 0.0, 'yaw': 0.0})[0] == 200
        wp = json.load(open(env[1]))['waypoints']['doghouse']
        assert wp['map'] == active and wp['x'] == pytest.approx(x - 0.5, abs=1e-3)


def test_replacing_a_waypoint_with_no_map_is_reported_as_unscoped(env):
    pathlib.Path(env[1]).write_text(json.dumps({'version': 2, 'routes': {}, 'waypoints': {
        'doghouse': {'x': 9.0, 'y': 9.0, 'yaw': 0.0, 'source': 'manual'}}}))
    out = _call(env, {'x': 2.0, 'y': 1.0, 'yaw': 0.0})[1]
    assert out['replaced_waypoint']['map'] is None
    assert out['replaced_waypoint']['x'] == 9.0


def test_listing_on_an_unmigrated_file_matches_the_migrated_view(env):
    before = tp.list_tags(env[0], 'floor1')
    assert before[0]['map_x'] is None and before[0]['poses'] == {}
    _call(env, {'x': 2.0, 'y': 1.0, 'yaw': 0.5})
    after = tp.list_tags(env[0], 'floor1')[0]
    assert (after['map_x'], after['map_name']) == (2.0, 'floor1')
    assert tp.list_tags(env[0], 'floor1_plan')[0]['map_x'] is None
    assert after['poses']['floor1']['surveyed_at'].endswith('UTC')


def test_locked_database_is_a_500_not_an_exception(env):
    c = sqlite3.connect(env[0], isolation_level=None)
    c.execute('BEGIN EXCLUSIVE')
    try:
        tp.BUSY_S, old = 0.05, tp.BUSY_S
        code, out = _call(env, {'x': 1, 'y': 1, 'yaw': 0})
        tp.BUSY_S = old
    finally:
        c.execute('ROLLBACK')
        c.close()
    assert code == 500 and 'tags.db' in out['error']
