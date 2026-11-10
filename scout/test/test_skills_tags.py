"""scout-skills tag tools with per-map poses (U11): which sightings may survey a
tag, the waypoint that follows, resurvey/delete, and the cross-map guard on
go_to_waypoint. server.py needs fastmcp/rosbridge, so the functions are
AST-extracted and exec'd against fakes at the system boundary (rosbridge, TF);
tags.py is the real module on a tmp tags.db. Nothing touches a robot."""

import ast
import asyncio
import importlib.util
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import pytest

DOCKER = Path(__file__).resolve().parents[2] / 'docker' / 'scout-skills'
_TREE = ast.parse((DOCKER / 'server.py').read_text())


class _ToolError(Exception):
    pass


def _extract(name, ns):
    fn = next(n for n in _TREE.body
              if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
              and n.name == name)
    fn.decorator_list = []
    exec(compile(ast.Module(body=[fn], type_ignores=[]), name, 'exec'), ns)
    return ns[name]


@pytest.fixture
def tagdb(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('skills_tags_u11', DOCKER / 'tags.py')
    mod = importlib.util.module_from_spec(spec)
    sys.modules['skills_tags_u11'] = mod
    spec.loader.exec_module(mod)
    monkeypatch.setattr(mod, 'DB_PATH', str(tmp_path / 'tags.db'))
    monkeypatch.setattr(mod, 'SITE_JSON', str(tmp_path / 'site.json'))
    return mod


def _activate(tagdb, name):
    Path(tagdb.SITE_JSON).write_text(json.dumps({'version': 2, 'active_map': name}))


# --- _may_first_survey -------------------------------------------------------------

def _may():
    ns = {}
    return _extract('_may_first_survey', ns)


@pytest.mark.parametrize('entry,active,armed,want', [
    ({'name': 't', 'map_x': None, 'poses': {}}, 'floor1', set(), True),
    ({'name': 't', 'map_x': 1.0, 'poses': {'floor1': {}}}, 'floor1', set(), False),
    ({'name': 't', 'map_x': None, 'poses': {'floor2': {}}}, 'floor1', set(), False),
    ({'name': 't', 'map_x': None, 'poses': {'floor2': {}}}, 'floor1',
     {('t', 'floor1')}, True),
    ({'name': 't', 'map_x': None, 'poses': {'floor2': {}}}, 'floor1',
     {('t', 'floor2')}, False),
    ({'name': 't', 'map_x': None, 'poses': {'floor2': {}}}, 'floor1',
     {('other', 'floor1')}, False),
    ({'name': 't', 'map_x': None, 'poses': {}}, None, set(), False),
    ({'name': 't', 'map_x': 1.0, 'poses': {}}, 'floor1', {('t', 'floor1')}, False),
])
def test_may_first_survey(entry, active, armed, want):
    assert _may()(entry, active, armed) is want


# --- _scan_tags with fakes ---------------------------------------------------------

class _Tree:
    """TfTree stand-in: the tag sits at (tx, 0) facing +x."""

    def __init__(self, tx):
        self.tx = tx

    def add_message(self, m):
        pass

    def to_ancestor(self, v, frame, target):
        v = np.asarray(v, dtype=float)
        return np.array([self.tx + (1.0 if v[2] else 0.0), 0.0, 0.0])


class _Bridge:
    def __init__(self, family, tag_id):
        self.det = {'family': family, 'id': tag_id,
                    'centre': {'x': 1.0, 'y': 1.0},
                    'corners': [{'x': 0.0, 'y': 0.0}, {'x': 2.0, 'y': 2.0}]}

    async def __aenter__(self):
        return self

    async def __aexit__(self, *exc):
        return False

    async def subscribe_collect(self, topic, msg_type, duration):
        if topic == '/detections':
            return [{'detections': [self.det]}]
        if topic == '/tf_static':
            return [{'transforms': [{'child_frame_id': 'tag36h11:0'}]}]
        return []


def _scan(tagdb, tx=5.0, update_waypoints=True, armed=None, store=None):
    store = store if store is not None else {'waypoints': {}}
    saved = []
    armed = armed if armed is not None else set()

    async def robot_pose(rb):
        return {'x': 0.0, 'y': 0.0}

    ns = {
        'asyncio': asyncio, 'math': math, 'datetime': datetime,
        'timezone': timezone, 'tagdb': tagdb, 'DETECTIONS_TOPIC': '/detections',
        'RosBridge': lambda background=False: _Bridge('tag36h11', 0),
        'TfTree': lambda: _Tree(tx), '_robot_pose': robot_pose,
        '_load_waypoints': lambda: store,
        '_store_waypoints': lambda s: saved.append(json.loads(json.dumps(s))),
        '_survey_armed': armed,
    }
    _extract('_may_first_survey', ns)
    scan = _extract('_scan_tags', ns)
    results = asyncio.run(scan(update_waypoints=update_waypoints))
    return results, store, saved, armed


def test_never_surveyed_tag_takes_its_first_pose_on_the_active_map(tagdb):
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    _activate(tagdb, 'floor1')
    results, store, saved, _ = _scan(tagdb)
    assert results[0]['waypoint_refreshed'] == 'doghouse'
    assert tagdb.lookup('tag36h11', 0, active='floor1')['map_x'] == 5.0
    assert store['waypoints']['doghouse']['map'] == 'floor1'
    assert len(saved) == 1


def test_a_surveyed_tag_keeps_its_survey_and_refreshes_its_waypoint(tagdb):
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    tagdb.record_sighting('tag36h11', 0, (9.0, 9.0, 0.0), map_name='floor1')
    _activate(tagdb, 'floor1')
    _, store, _, _ = _scan(tagdb, tx=5.0)
    assert tagdb.lookup('tag36h11', 0, active='floor1')['map_x'] == 9.0   # immutable
    assert store['waypoints']['doghouse']['x'] == pytest.approx(8.5)


def test_tag_surveyed_elsewhere_is_not_auto_surveyed_here_and_gets_no_waypoint(tagdb):
    # the floor-transit candidate: surveying it from the live robot pose would
    # pollute this map and kill the transit
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    tagdb.record_sighting('tag36h11', 0, (9.0, 9.0, 0.0), map_name='floor2')
    _activate(tagdb, 'floor1')
    results, store, saved, _ = _scan(tagdb)
    assert tagdb.lookup('tag36h11', 0, active='floor1')['map_x'] is None
    assert 'doghouse' not in store['waypoints'] and saved == []
    assert 'waypoint_refreshed' not in results[0]
    assert results[0]['registered'] is True


def test_armed_tag_surveys_the_active_map_once_and_disarms(tagdb):
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    tagdb.record_sighting('tag36h11', 0, (9.0, 9.0, 0.0), map_name='floor1')
    _activate(tagdb, 'floor1_plan')
    results, store, _, armed = _scan(tagdb, armed={('doghouse', 'floor1_plan')})
    assert tagdb.lookup('tag36h11', 0, active='floor1_plan')['map_x'] == 5.0
    assert tagdb.lookup('tag36h11', 0, active='floor1')['map_x'] == 9.0   # untouched
    assert armed == set()
    assert store['waypoints']['doghouse']['map'] == 'floor1_plan'


def test_no_active_map_never_surveys_or_writes_a_waypoint(tagdb):
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    results, store, saved, _ = _scan(tagdb)
    assert tagdb.lookup('tag36h11', 0, active='floor1')['map_x'] is None
    assert store['waypoints'] == {} and saved == []


def test_waypoint_refresh_reports_which_map_it_replaced(tagdb):
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    tagdb.record_sighting('tag36h11', 0, (9.0, 9.0, 0.0), map_name='floor1_plan')
    _activate(tagdb, 'floor1_plan')
    prev = {'x': 1.0, 'y': 1.0, 'yaw': 0.0, 'map': 'floor1', 'source': 'tag'}
    results, store, _, _ = _scan(tagdb, store={'waypoints': {'doghouse': prev}})
    assert results[0]['waypoint_replaced']['map'] == 'floor1'
    assert results[0]['waypoint_replaced']['x'] == 1.0
    assert store['waypoints']['doghouse']['map'] == 'floor1_plan'


def test_same_map_refresh_reports_no_replacement(tagdb):
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    tagdb.record_sighting('tag36h11', 0, (9.0, 9.0, 0.0), map_name='floor1')
    _activate(tagdb, 'floor1')
    prev = {'x': 1.0, 'y': 1.0, 'yaw': 0.0, 'map': 'floor1', 'source': 'tag'}
    results, _, _, _ = _scan(tagdb, store={'waypoints': {'doghouse': prev}})
    assert 'waypoint_replaced' not in results[0]


def test_unchanged_waypoint_is_not_rewritten(tagdb):
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    tagdb.record_sighting('tag36h11', 0, (9.0, 9.0, 0.0), map_name='floor1')
    _activate(tagdb, 'floor1')
    _, store, saved, _ = _scan(tagdb)
    assert len(saved) == 1
    _, _, saved2, _ = _scan(tagdb, store=store)
    assert saved2 == []


# --- resurvey / delete / go_to_waypoint ---------------------------------------------

def _tool(name, tagdb, armed):
    ns = {'tagdb': tagdb, 'ToolError': _ToolError, '_survey_armed': armed}
    return _extract(name, ns)


def test_resurvey_clears_the_active_map_only_and_arms_it(tagdb):
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    tagdb.record_sighting('tag36h11', 0, (1.0, 1.0, 0.0), map_name='floor1')
    tagdb.record_sighting('tag36h11', 0, (2.0, 2.0, 0.0), map_name='floor1_plan')
    _activate(tagdb, 'floor1')
    armed = set()
    out = asyncio.run(_tool('resurvey_apriltag', tagdb, armed)('doghouse'))
    assert out['map'] == 'floor1' and out['cleared_poses'] == 1
    assert armed == {('doghouse', 'floor1')}
    assert tagdb.lookup('tag36h11', 0, active='floor1')['map_x'] is None
    assert tagdb.lookup('tag36h11', 0, active='floor1_plan')['map_x'] == 2.0


def test_resurvey_arms_a_tag_that_is_surveyed_only_elsewhere(tagdb):
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    tagdb.record_sighting('tag36h11', 0, (1.0, 1.0, 0.0), map_name='floor1')
    _activate(tagdb, 'floor1_plan')
    armed = set()
    out = asyncio.run(_tool('resurvey_apriltag', tagdb, armed)('doghouse'))
    assert out['cleared_poses'] == 0 and armed == {('doghouse', 'floor1_plan')}
    assert tagdb.lookup('tag36h11', 0, active='floor1')['map_x'] == 1.0


def test_resurvey_refuses_without_an_active_map_or_for_an_unknown_tag(tagdb):
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    armed = set()
    tool = _tool('resurvey_apriltag', tagdb, armed)
    with pytest.raises(_ToolError, match='no active map'):
        asyncio.run(tool('doghouse'))
    _activate(tagdb, 'floor1')
    with pytest.raises(_ToolError, match='no AprilTag'):
        asyncio.run(tool('ghost'))
    assert armed == set()


def test_delete_apriltag_removes_all_poses_and_disarms(tagdb):
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    tagdb.record_sighting('tag36h11', 0, (1.0, 1.0, 0.0), map_name='floor1')
    tagdb.record_sighting('tag36h11', 0, (2.0, 2.0, 0.0), map_name='floor1_plan')
    armed = {('doghouse', 'floor1'), ('keep', 'floor1')}
    out = asyncio.run(_tool('delete_apriltag', tagdb, armed)('doghouse'))
    assert out == {'deleted': 'doghouse'} and armed == {('keep', 'floor1')}
    assert tagdb.all_tags() == []


def test_list_apriltags_exposes_the_per_map_poses(tagdb):
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    tagdb.record_sighting('tag36h11', 0, (1.0, 1.0, 0.0), map_name='floor1')
    _activate(tagdb, 'floor1')
    ns = {'tagdb': tagdb, '_tag_watch_enabled': True, '_tag_watch_last': {}}
    out = asyncio.run(_extract('list_apriltags', ns)())
    assert out['tags'][0]['map_x'] == 1.0 and 'floor1' in out['tags'][0]['poses']


def _go_to_waypoint(tagdb, store):
    dispatched = []

    async def dispatch(x, y, yaw):
        dispatched.append((x, y, yaw))
        return {'sent': True}

    ns = {'tagdb': tagdb, 'ToolError': _ToolError, '_load_waypoints': lambda: store,
          '_dispatch_goal': dispatch}
    return _extract('go_to_waypoint', ns), dispatched


def test_go_to_waypoint_refuses_a_waypoint_of_the_other_map(tagdb):
    # surveying a tag on map B rewrites its waypoint WITH map B; whichever map's
    # waypoint is stored, it can only drive while that map is active
    store = {'waypoints': {'doghouse': {'x': 1.0, 'y': 2.0, 'yaw': 0.0, 'map': 'floor1'}}}
    go, dispatched = _go_to_waypoint(tagdb, store)
    _activate(tagdb, 'floor1_plan')
    with pytest.raises(_ToolError, match="belongs to map 'floor1'"):
        asyncio.run(go('doghouse'))
    assert dispatched == []
    _activate(tagdb, 'floor1')
    assert asyncio.run(go('doghouse'))['sent'] is True
    assert dispatched == [(1.0, 2.0, 0.0)]


def test_a_surveyed_tag_waypoint_always_carries_the_map_of_its_coordinates(tagdb):
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    tagdb.record_sighting('tag36h11', 0, (9.0, 0.0, 0.0), map_name='floor1')
    tagdb.record_sighting('tag36h11', 0, (20.0, 0.0, 0.0), map_name='floor1_plan')
    store = {'waypoints': {}}
    for active, x in (('floor1', 8.5), ('floor1_plan', 19.5), ('floor1', 8.5)):
        _activate(tagdb, active)
        _scan(tagdb, store=store)
        wp = store['waypoints']['doghouse']
        assert wp['map'] == active and wp['x'] == pytest.approx(x)
