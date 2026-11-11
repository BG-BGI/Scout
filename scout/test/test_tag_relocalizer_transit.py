"""tag_relocalizer floor transit across map sources (U6, R9): a transit between
a scan map and a drawing map is never a live LoadMap — it logs one throttled
line and leaves the slam restart to the operator; same-source transit is
unchanged. The node needs rclpy, so the two methods are AST-extracted from the
class and exec'd against a fake `self` (the ROS boundary only: logger, params,
the LoadMap client). The decision itself is core.sites.sources_differ."""

import ast
import json
import pathlib
import types

import pytest
from test_sites import HOSTILE

from scout.core import sites

NODE = pathlib.Path(__file__).resolve().parent.parent / 'scout' / 'tag_relocalizer.py'
_CLASS = next(n for n in ast.parse(NODE.read_text()).body
              if isinstance(n, ast.ClassDef) and n.name == 'TagRelocalizer')

NOW = 1000.0


def _method(name, **ns):
    fn = next(n for n in _CLASS.body
              if isinstance(n, ast.FunctionDef) and n.name == name)
    glob = dict(ns)
    exec(compile(ast.Module(body=[fn], type_ignores=[]), name, 'exec'), glob)
    return glob[name]


class _Logger:
    def __init__(self):
        self.warns = []
        self.errors = []

    def warn(self, msg, throttle_duration_sec=None):
        self.warns.append((msg, throttle_duration_sec))

    def error(self, msg):
        self.errors.append(msg)

    def info(self, msg):
        pass


class _Future:
    def __init__(self):
        self.callbacks = []

    def add_done_callback(self, cb):
        self.callbacks.append(cb)


class _Self:
    def __init__(self, maps_dir, ready=True):
        self.logger = _Logger()
        self.load_requests = []
        self._cand = {'map': None, 'count': 0, 'pose': None, 'name': None}
        self._switching = False
        self._last_switch_t = 0.0
        self._nav_busy = False
        params = {'min_transit_sightings': 3, 'transit_cooldown_s': 30.0,
                  'maps_dir': str(maps_dir), 'site_json': str(maps_dir / 'site.json')}
        self.params = params
        self.get_parameter = lambda n: types.SimpleNamespace(value=self.params[n])
        self._load_map_cli = types.SimpleNamespace(
            service_is_ready=lambda: ready, call_async=self._call_async)
        self._on_map_loaded = lambda *a: None

    def get_logger(self):
        return self.logger

    def _call_async(self, req):
        self.load_requests.append(req.map_url)
        return _Future()


def _track(maps_dir, maps, active, home, *, sightings=3, slam_mode='localization',
           ready=True, mode=None):
    node = _Self(maps_dir, ready)
    fake_loadmap = types.SimpleNamespace(
        Request=lambda: types.SimpleNamespace(map_url=None),
        Response=types.SimpleNamespace(RESULT_SUCCESS=0))
    track = _method('_track_transit', sites=sites, os=__import__('os'),
                    time=types.SimpleNamespace(monotonic=lambda: NOW),
                    LoadMap=fake_loadmap)
    for _ in range(sightings):
        track(node, home, 1.0, 'tag7', 0.0, 'pose', slam_mode=slam_mode,
              active_map=active, maps=maps, mode=mode or slam_mode)
    return node


MAPS = {'scanned': {'source': 'scan'}, 'other_scan': {'source': 'scan'},
        'plan': {'source': 'drawing'}, 'other_plan': {'source': 'drawing'}}


@pytest.fixture
def maps_dir(tmp_path):
    for name in MAPS:
        (tmp_path / (name + '.yaml')).write_text('x')
    return tmp_path


@pytest.mark.parametrize('active,home', [('scanned', 'plan'), ('plan', 'scanned')])
def test_cross_source_transit_is_skipped_with_one_clear_throttled_line(
        maps_dir, active, home):
    node = _track(maps_dir, MAPS, active, home)
    assert node.load_requests == []
    assert node._switching is False
    assert len(node.logger.warns) == 1
    msg, throttle = node.logger.warns[0]
    assert throttle == 30.0
    assert home in msg and active in msg and 'tag7' in msg
    assert 'drawing' in msg and 'scan' in msg
    assert 'slam restart' in msg and 'operator' in msg


def test_cross_source_stays_quiet_below_the_sighting_threshold(maps_dir):
    node = _track(maps_dir, MAPS, 'scanned', 'plan', sightings=2)
    assert node.logger.warns == [] and node.load_requests == []


def test_cross_source_never_loads_a_map_however_long_the_tag_is_seen(maps_dir):
    node = _track(maps_dir, MAPS, 'scanned', 'plan', sightings=50)
    assert node.load_requests == []
    assert all(t == 30.0 for _, t in node.logger.warns)


@pytest.mark.parametrize('active,home', [
    ('scanned', 'other_scan'), ('plan', 'other_plan')])
def test_same_source_transit_still_swaps_live(maps_dir, active, home):
    node = _track(maps_dir, MAPS, active, home)
    assert node.load_requests == [str(maps_dir / (home + '.yaml'))]
    assert node._switching is True
    assert node.logger.warns == []


def test_v1_site_without_maps_transits_as_before(maps_dir):
    node = _track(maps_dir, {}, 'scanned', 'other_scan')
    assert node.load_requests == [str(maps_dir / 'other_scan.yaml')]


def test_non_localization_mode_keeps_its_own_refusal(maps_dir):
    node = _track(maps_dir, MAPS, 'scanned', 'plan', slam_mode='continue')
    assert node.load_requests == []
    assert len(node.logger.warns) == 1
    assert 'localization mode only' in node.logger.warns[0][0]


def test_nav_busy_check_still_runs_for_same_source(maps_dir):
    node = _Self(maps_dir)
    node._nav_busy = True
    track = _method('_track_transit', sites=sites, os=__import__('os'),
                    time=types.SimpleNamespace(monotonic=lambda: NOW),
                    LoadMap=None)
    for _ in range(3):
        track(node, 'other_scan', 1.0, 'tag7', 0.0, 'pose', slam_mode='localization',
              active_map='scanned', maps=MAPS, mode='localization')
    assert node.load_requests == []
    assert 'nav goal' in node.logger.warns[0][0]


def _read_site(tmp_path, content):
    path = tmp_path / 'site.json'
    if content is not None:
        path.write_text(content if isinstance(content, str) else json.dumps(content))
    node = _Self(tmp_path)
    return _method('_read_site', json=json, sites=sites)(node)


def test_read_site_returns_maps_for_v2(tmp_path):
    data = {'active_map': 'plan', 'slam_mode': 'auto', 'maps': MAPS}
    assert _read_site(tmp_path, data) == ('plan', 'auto', MAPS, 'new')


def test_read_site_v1_has_no_maps(tmp_path):
    assert _read_site(tmp_path, {'default_map': 'office'}) == (
        'office', None, {}, 'new')


@pytest.mark.parametrize('content', [None, '{not json', '[]', '"x"'])
def test_read_site_problem_disables_transit(tmp_path, content):
    assert _read_site(tmp_path, content) == (None, None, {}, None)


def test_read_site_non_dict_maps_is_empty(tmp_path):
    assert _read_site(tmp_path, {'active_map': 'a', 'maps': ['a']})[2] == {}


# --- the gate is the EFFECTIVE mode: `auto` on a drawing map runs amcl -------------

def _site_json(active, slam_mode, **sources):
    return {'version': 2, 'active_map': active, 'slam_mode': slam_mode,
            'maps': {n: {'source': src} for n, src in sources.items()}}


_KEEP = object()


def _transit(tmp_path, site, home, *, grids=(), sightings=3, maps_dir=_KEEP):
    """The real _read_site feeding the real _track_transit, against the
    ROS-boundary fake: the composition the node runs per detection frame."""
    for g in grids:
        (tmp_path / (g + '.yaml')).write_text('x')
    (tmp_path / 'site.json').write_text(json.dumps(site))
    node = _Self(tmp_path)
    if maps_dir is not _KEEP:
        node.params['maps_dir'] = maps_dir
    active, slam_mode, maps, mode = _method(
        '_read_site', json=json, sites=sites)(node)
    fake_loadmap = types.SimpleNamespace(
        Request=lambda: types.SimpleNamespace(map_url=None),
        Response=types.SimpleNamespace(RESULT_SUCCESS=0))
    track = _method('_track_transit', sites=sites, os=__import__('os'),
                    time=types.SimpleNamespace(monotonic=lambda: NOW),
                    LoadMap=fake_loadmap)
    for _ in range(sightings):
        track(node, home, 1.0, 'tag7', 0.0, 'pose', slam_mode=slam_mode,
              active_map=active, maps=maps, mode=mode)
    return node


def test_read_site_reports_the_effective_mode(tmp_path):
    (tmp_path / 'plan.yaml').write_text('x')
    node = _Self(tmp_path)
    (tmp_path / 'site.json').write_text(json.dumps(
        _site_json('plan', 'auto', plan='drawing')))
    out = _method('_read_site', json=json, sites=sites)(node)
    assert out[1] == 'auto' and out[3] == 'localization'


def test_auto_on_a_drawing_map_with_grids_transits_live(tmp_path):
    site = _site_json('plan', 'auto', plan='drawing', other_plan='drawing')
    node = _transit(tmp_path, site, 'other_plan', grids=('plan', 'other_plan'))
    assert node.load_requests == [str(tmp_path / 'other_plan.yaml')]
    assert node.logger.warns == []


def test_auto_drawing_transit_is_refused_when_the_home_grid_is_missing(tmp_path):
    site = _site_json('plan', 'auto', plan='drawing', other_plan='drawing')
    node = _transit(tmp_path, site, 'other_plan', grids=('plan',))
    assert node.load_requests == []
    assert len(node.logger.warns) == 1
    assert 'has no grid' in node.logger.warns[0][0]


def test_auto_on_a_drawing_map_without_its_own_grid_is_not_localization(tmp_path):
    site = _site_json('plan', 'auto', plan='drawing', other_plan='drawing')
    node = _transit(tmp_path, site, 'other_plan', grids=('other_plan',))
    assert node.load_requests == []
    assert "slam_mode is 'auto'" in node.logger.warns[0][0]


@pytest.mark.parametrize('files', [(), ('scanned.posegraph',)])
def test_auto_on_a_scan_map_is_refused_with_the_same_message_as_before(
        tmp_path, files):
    for f in files:
        (tmp_path / f).write_text('x')
    site = _site_json('scanned', 'auto', scanned='scan', other_scan='scan')
    node = _transit(tmp_path, site, 'other_scan', grids=('scanned', 'other_scan'))
    assert node.load_requests == []
    assert node.logger.warns == [(
        "tag 'tag7' lives on map 'other_scan' but slam_mode is 'auto' — not "
        'switching (localization mode only)', 30.0)]


def test_pinned_localization_is_unchanged(tmp_path):
    site = _site_json('scanned', 'localization', scanned='scan', other_scan='scan')
    node = _transit(tmp_path, site, 'other_scan', grids=('other_scan',))
    assert node.load_requests == [str(tmp_path / 'other_scan.yaml')]


@pytest.mark.parametrize('slam_mode,active,home,grids', [
    ('localization', 'scanned', 'plan', ('scanned', 'plan')),
    ('auto', 'plan', 'scanned', ('plan', 'scanned')),
    ('localization', 'plan', 'scanned', ('plan', 'scanned')),
])
def test_cross_source_is_refused_whatever_the_mode(
        tmp_path, slam_mode, active, home, grids):
    site = _site_json(active, slam_mode, scanned='scan', plan='drawing')
    node = _transit(tmp_path, site, home, grids=grids)
    assert node.load_requests == []
    assert len(node.logger.warns) == 1
    assert 'operator' in node.logger.warns[0][0]
    assert node.logger.warns[0][1] == 30.0


def test_unnormalizable_site_keeps_the_raw_mode_gate(tmp_path):
    # dangling active_map: effective_mode falls back to the raw slam_mode, so
    # a pinned localization site still transits as it did before
    site = _site_json('ghost', 'localization', other_scan='scan', third='scan')
    node = _transit(tmp_path, site, 'other_scan', grids=('other_scan',))
    assert node.load_requests == [str(tmp_path / 'other_scan.yaml')]


# --- a malformed-but-valid-JSON site.json must never escape the detection callback --

def _hostile_read(tmp_path, data, maps_dir=_KEEP):
    (tmp_path / 'site.json').write_text(json.dumps(data))
    node = _Self(tmp_path)
    if maps_dir is not _KEEP:
        node.params['maps_dir'] = maps_dir
    return _method('_read_site', json=json, sites=sites)(node)


@pytest.mark.parametrize('label,data,want', HOSTILE, ids=[c[0] for c in HOSTILE])
def test_read_site_never_raises_on_hostile_json(tmp_path, label, data, want):
    active, slam_mode, maps, mode = _hostile_read(tmp_path, data)
    assert isinstance(maps, dict)
    assert mode == (want if isinstance(data, dict) else None)
    if not isinstance(data, dict):
        assert (active, slam_mode, maps) == (None, None, {})


@pytest.mark.parametrize('label,data,want', HOSTILE, ids=[c[0] for c in HOSTILE])
@pytest.mark.parametrize('bad_dir', [None, 5, ['x']])
def test_read_site_never_raises_on_a_bad_maps_dir_param(
        tmp_path, label, data, want, bad_dir):
    mode = _hostile_read(tmp_path, data, maps_dir=bad_dir)[3]
    assert mode is None or isinstance(mode, str)


@pytest.mark.parametrize('label,data,want', HOSTILE, ids=[c[0] for c in HOSTILE])
def test_transit_never_raises_and_only_a_localization_gate_reaches_loadmap(
        tmp_path, label, data, want):
    node = _transit(tmp_path, data, 'other', grids=('other',))
    assert bool(node.load_requests) == (want == 'localization')


@pytest.mark.parametrize('label,data,want', HOSTILE, ids=[c[0] for c in HOSTILE])
@pytest.mark.parametrize('bad_dir', [None, 5])
def test_transit_survives_a_bad_maps_dir_too(tmp_path, label, data, want, bad_dir):
    node = _Self(tmp_path)
    node.params['maps_dir'] = bad_dir
    (tmp_path / 'site.json').write_text(json.dumps(data))
    active, slam_mode, maps, mode = _method(
        '_read_site', json=json, sites=sites)(node)
    track = _method('_track_transit', sites=sites, os=__import__('os'),
                    time=types.SimpleNamespace(monotonic=lambda: NOW),
                    LoadMap=None)
    if mode == 'localization':
        # pinned localization with a bad maps_dir param: the node's own
        # yaml_path join is a pre-existing misconfiguration, not a data shape
        return
    for _ in range(3):
        track(node, 'other', 1.0, 'tag7', 0.0, 'pose', slam_mode=slam_mode,
              active_map=active, maps=maps, mode=mode)
    assert node.load_requests == []


# =============================================================================
# U11: a tag is surveyed per map. The real _registered / _on_detections /
# _track_transit run against a real tmp tags.db; only the ROS boundary is faked.
# =============================================================================

import importlib.util  # noqa: E402
import math  # noqa: E402
import sqlite3  # noqa: E402
import sys  # noqa: E402

from scout.core import tag_homes  # noqa: E402

TAGS_PY = NODE.parents[2] / 'docker' / 'scout-skills' / 'tags.py'
_NODE_TREE = ast.parse(NODE.read_text())


def _module_fn(name):
    fn = next(n for n in _NODE_TREE.body
              if isinstance(n, ast.FunctionDef) and n.name == name)
    ns = {}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), name, 'exec'), ns)
    return ns[name]


@pytest.fixture
def tagdb(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('skills_tags_reloc', TAGS_PY)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['skills_tags_reloc'] = mod
    spec.loader.exec_module(mod)
    monkeypatch.setattr(mod, 'DB_PATH', str(tmp_path / 'tags.db'))
    monkeypatch.setattr(mod, 'SITE_JSON', str(tmp_path / 'site.json'))
    return mod


SITE_MAPS = {'floor1': {'source': 'scan'}, 'floor1_plan': {'source': 'drawing'},
             'floor2': {'source': 'scan'}, 'floor2_plan': {'source': 'drawing'},
             'floor3': {'source': 'scan'}}


class _Frames(_Self):
    """_Self + what _on_detections touches beyond the transit fakes."""

    def __init__(self, maps_dir):
        super().__init__(maps_dir)
        self.params['tags_db'] = str(maps_dir / 'tags.db')
        self._done = False
        self.published = []
        self.solved_for = []
        self._pub = types.SimpleNamespace(publish=self.published.append)
        self._corrector = types.SimpleNamespace(reset=lambda: None)


def _pose_msg(x=1.0):
    pos = types.SimpleNamespace(x=x, y=0.0)
    return types.SimpleNamespace(
        pose=types.SimpleNamespace(pose=types.SimpleNamespace(position=pos)))


def _node(tmp_path, tagdb, *, active, slam_mode='localization', grids=(),
          busy=0.05):
    for g in grids:
        (tmp_path / (g + '.yaml')).write_text('x')
    (tmp_path / 'site.json').write_text(json.dumps({
        'version': 2, 'active_map': active, 'slam_mode': slam_mode,
        'maps': SITE_MAPS}))
    node = _Frames(tmp_path)
    fake_loadmap = types.SimpleNamespace(
        Request=lambda: types.SimpleNamespace(map_url=None),
        Response=types.SimpleNamespace(RESULT_SUCCESS=0))
    glob = dict(sites=sites, tag_homes=tag_homes, sqlite3=sqlite3,
                os=__import__('os'), json=json, math=math,
                time=types.SimpleNamespace(monotonic=lambda: NOW),
                LoadMap=fake_loadmap, REGISTRY_BUSY_S=busy,
                _norm_family=_module_fn('_norm_family'))
    for name in ('_registered', '_read_site', '_track_transit', '_on_detections'):
        setattr(node, name, types.MethodType(_method(name, **glob), node))

    def solve(det, entry):
        node.solved_for.append(entry['name'])
        return 1.0, entry['name'], 0.0, _pose_msg(entry['map_x'])

    node._solve = solve
    node._maybe_correct = lambda *a: None
    return node


def _frame(*tags):
    return types.SimpleNamespace(detections=[
        types.SimpleNamespace(family=f, id=i) for f, i in tags])


def _frames(node, n=3, tags=(('tag36h11', 0),)):
    for _ in range(n):
        node._on_detections(_frame(*tags))
    return node


def _tag(tagdb, tag_id, name, **poses):
    tagdb.upsert(name, tag_id, 'tag36h11', '', 0.16)
    for m, pose in poses.items():
        assert tagdb.record_sighting('tag36h11', tag_id, pose, map_name=m)


def test_pose_on_the_active_map_seeds_there_and_never_transits(tmp_path, tagdb):
    _tag(tagdb, 0, 'doghouse', floor1=(1.0, 2.0, 0.5), floor2=(9.0, 9.0, 0.0))
    node = _frames(_node(tmp_path, tagdb, active='floor1',
                         grids=('floor1', 'floor2')))
    assert node.solved_for == ['doghouse'] * 3
    assert len(node.published) == 1 and node.load_requests == []
    assert node._cand['count'] == 0


def test_the_pose_used_is_the_active_maps_not_the_other_maps(tmp_path, tagdb):
    _tag(tagdb, 0, 'doghouse', floor1=(1.0, 2.0, 0.5), floor1_plan=(7.0, 8.0, 0.1))
    n1 = _frames(_node(tmp_path, tagdb, active='floor1', grids=('floor1',)), n=1)
    assert n1.published[0].pose.pose.position.x == 1.0
    n2 = _frames(_node(tmp_path, tagdb, active='floor1_plan'), n=1)
    assert n2.published[0].pose.pose.position.x == 7.0


def test_one_same_source_foreign_map_transits_via_the_existing_path(tmp_path, tagdb):
    _tag(tagdb, 0, 'elev', floor2=(3.0, 3.0, 0.0))
    node = _frames(_node(tmp_path, tagdb, active='floor1', grids=('floor1', 'floor2')))
    assert node.load_requests == [str(tmp_path / 'floor2.yaml')]
    assert node._switching is True and node.logger.warns == []


def test_same_source_choice_ignores_poses_on_the_other_source(tmp_path, tagdb):
    _tag(tagdb, 0, 'elev', floor2=(3.0, 3.0, 0.0), floor1_plan=(1.0, 1.0, 0.0))
    node = _frames(_node(tmp_path, tagdb, active='floor1',
                         grids=('floor1', 'floor2', 'floor1_plan')))
    assert node.load_requests == [str(tmp_path / 'floor2.yaml')]


def test_foreign_only_on_the_other_source_is_refused_with_the_existing_message(
        tmp_path, tagdb):
    _tag(tagdb, 0, 'elev', floor1_plan=(1.0, 1.0, 0.0))
    node = _frames(_node(tmp_path, tagdb, active='floor1',
                         grids=('floor1', 'floor1_plan')))
    assert node.load_requests == []
    assert len(node.logger.warns) == 1
    msg, throttle = node.logger.warns[0]
    assert "tag 'elev' lives on drawing map 'floor1_plan'" in msg
    assert 'slam restart' in msg and throttle == 30.0


def test_two_same_source_foreign_maps_are_refused_and_named(tmp_path, tagdb):
    _tag(tagdb, 0, 'elev', floor2=(3.0, 3.0, 0.0), floor3=(4.0, 4.0, 0.0))
    node = _frames(_node(tmp_path, tagdb, active='floor1',
                         grids=('floor1', 'floor2', 'floor3')))
    assert node.load_requests == [] and node.solved_for == []
    assert len(node.logger.warns) == 3                 # one per frame, throttled by rclpy
    msg, throttle = node.logger.warns[0]
    assert 'floor2' in msg and 'floor3' in msg and "'elev'" in msg
    assert "active scan map 'floor1'" in msg and throttle == 30.0


def test_two_other_source_foreign_maps_are_refused_and_named(tmp_path, tagdb):
    _tag(tagdb, 0, 'elev', floor1_plan=(3.0, 3.0, 0.0), floor2_plan=(4.0, 4.0, 0.0))
    node = _frames(_node(tmp_path, tagdb, active='floor1'), n=1)
    assert node.load_requests == [] and 'floor1_plan' in node.logger.warns[0][0]


def test_legacy_null_map_row_behaves_as_before(tmp_path, tagdb):
    db = sqlite3.connect(tagdb.DB_PATH)
    db.execute("""CREATE TABLE tags(family TEXT NOT NULL, tag_id INTEGER NOT NULL,
        name TEXT NOT NULL UNIQUE, role TEXT NOT NULL DEFAULT '',
        size_m REAL NOT NULL DEFAULT 0.16, map_x REAL, map_y REAL, map_yaw REAL,
        last_seen TEXT, map_name TEXT, PRIMARY KEY(family, tag_id))""")
    db.execute("INSERT INTO tags VALUES('tag36h11',0,'elev','',0.16,4.0,5.0,0.0,NULL,NULL)")
    db.commit()
    db.close()
    for migrated in (False, True):
        if migrated:
            tagdb.all_tags()
        for active in ('floor1', 'floor2_plan'):
            node = _frames(_node(tmp_path, tagdb, active=active,
                                 grids=('floor1', 'floor2_plan')), n=2)
            assert node.published and node.published[0].pose.pose.position.x == 4.0
            assert node.load_requests == [] and node._cand['count'] == 0


def test_legacy_pose_plus_a_named_survey_elsewhere_transits_to_it(tmp_path, tagdb):
    db = sqlite3.connect(tagdb.DB_PATH)
    db.execute("""CREATE TABLE tags(family TEXT NOT NULL, tag_id INTEGER NOT NULL,
        name TEXT NOT NULL UNIQUE, role TEXT NOT NULL DEFAULT '',
        size_m REAL NOT NULL DEFAULT 0.16, map_x REAL, map_y REAL, map_yaw REAL,
        last_seen TEXT, map_name TEXT, PRIMARY KEY(family, tag_id))""")
    db.execute("INSERT INTO tags VALUES('tag36h11',0,'elev','',0.16,4.0,5.0,0.0,NULL,NULL)")
    db.commit()
    db.close()
    tagdb.record_sighting('tag36h11', 0, (8.0, 8.0, 0.0), map_name='floor2')
    node = _frames(_node(tmp_path, tagdb, active='floor1', grids=('floor1', 'floor2')))
    assert node.load_requests == [str(tmp_path / 'floor2.yaml')]


def test_unsurveyed_and_unregistered_tags_do_nothing(tmp_path, tagdb):
    _tag(tagdb, 0, 'blank')
    node = _frames(_node(tmp_path, tagdb, active='floor1'),
                   tags=(('tag36h11', 0), ('tag36h11', 77)))
    assert node.solved_for == [] and node.published == [] and node.logger.warns == []


def test_auto_on_a_drawing_map_transits_between_drawing_maps(tmp_path, tagdb):
    _tag(tagdb, 0, 'elev', floor2_plan=(3.0, 3.0, 0.0))
    node = _frames(_node(tmp_path, tagdb, active='floor1_plan', slam_mode='auto',
                         grids=('floor1_plan', 'floor2_plan')))
    assert node.load_requests == [str(tmp_path / 'floor2_plan.yaml')]


def test_two_foreign_tags_in_one_frame_pick_the_nearest_without_raising(
        tmp_path, tagdb):
    _tag(tagdb, 0, 'a', floor2=(3.0, 3.0, 0.0))
    _tag(tagdb, 1, 'b', floor2=(5.0, 5.0, 0.0))
    node = _node(tmp_path, tagdb, active='floor1', grids=('floor1', 'floor2'))
    dists = {'a': 2.0, 'b': 1.0}

    def solve(det, entry):
        return dists[entry['name']], entry['name'], 0.0, _pose_msg()

    node._solve = solve
    _frames(node, tags=(('tag36h11', 0), ('tag36h11', 1)))
    assert node.load_requests == [str(tmp_path / 'floor2.yaml')]
    assert node._cand['name'] == 'b'


# --- hostile / damaged registries never escape the per-frame path --------------------

def _write_db(path, sql):
    db = sqlite3.connect(path)
    for stmt in sql:
        db.execute(stmt)
    db.commit()
    db.close()


TAGS_DDL = ('CREATE TABLE tags(family, tag_id, name, role, size_m, map_x, map_y, '
            'map_yaw, last_seen, map_name)')
POSES_DDL = ('CREATE TABLE tag_poses(family, tag_id, map_name, map_x, map_y, '
             'map_yaw, surveyed_at)')

HOSTILE_DBS = {
    'no tables': [],
    'tags only, no pose columns': ['CREATE TABLE tags(family, tag_id, name)',
                                   "INSERT INTO tags VALUES('tag36h11',0,'a')"],
    'tag_poses with a missing column': [
        TAGS_DDL, "INSERT INTO tags(family,tag_id,name) VALUES('tag36h11',0,'a')",
        'CREATE TABLE tag_poses(family, tag_id)'],
    'text coordinates': [
        TAGS_DDL, "INSERT INTO tags(family,tag_id,name) VALUES('tag36h11',0,'a')",
        POSES_DDL, "INSERT INTO tag_poses VALUES('tag36h11',0,'floor2','x','y','z',NULL)"],
    'blob coordinates': [
        TAGS_DDL, "INSERT INTO tags(family,tag_id,name) VALUES('tag36h11',0,'a')",
        POSES_DDL, "INSERT INTO tag_poses VALUES('tag36h11',0,'floor2',x'00',x'01',x'02',NULL)"],
    'integer family': [
        TAGS_DDL, "INSERT INTO tags(family,tag_id,name) VALUES(7,0,'a')"],
    'null family': [
        TAGS_DDL, "INSERT INTO tags(family,tag_id,name) VALUES(NULL,0,'a')"],
    'non-str name': [
        TAGS_DDL, "INSERT INTO tags(family,tag_id,name) VALUES('tag36h11',0,5)",
        POSES_DDL, "INSERT INTO tag_poses VALUES('tag36h11',0,'floor2',1,2,3,NULL)"],
    'null map name': [
        TAGS_DDL, "INSERT INTO tags(family,tag_id,name) VALUES('tag36h11',0,'a')",
        POSES_DDL, "INSERT INTO tag_poses VALUES('tag36h11',0,NULL,1,2,3,NULL)"],
    'legacy text pose': [
        TAGS_DDL, "INSERT INTO tags VALUES('tag36h11',0,'a','',0.1,'x',2,3,NULL,NULL)"],
    'tag_poses is a view of garbage': [
        TAGS_DDL, "INSERT INTO tags(family,tag_id,name) VALUES('tag36h11',0,'a')",
        'CREATE VIEW tag_poses AS SELECT 1 AS x'],
}


@pytest.mark.parametrize('label', sorted(HOSTILE_DBS))
def test_hostile_registry_is_not_registered_and_never_raises(tmp_path, tagdb, label):
    _write_db(tmp_path / 'tags.db', HOSTILE_DBS[label])
    node = _node(tmp_path, tagdb, active='floor1')
    assert node._registered('tag36h11', 0) is None
    _frames(node, n=2)
    assert node.load_requests == [] and node.published == []


def test_a_corrupt_file_is_not_registered(tmp_path, tagdb):
    (tmp_path / 'tags.db').write_bytes(b'not a database at all' * 200)
    node = _node(tmp_path, tagdb, active='floor1')
    assert node._registered('tag36h11', 0) is None
    _frames(node, n=1)
    assert node.logger.warns and 'tags.db read failed' in node.logger.warns[0][0]
    assert node.logger.warns[0][1] == 30.0


def test_a_directory_in_place_of_the_file_is_not_registered(tmp_path, tagdb):
    (tmp_path / 'tags.db').mkdir()
    node = _node(tmp_path, tagdb, active='floor1')
    assert node._registered('tag36h11', 0) is None


def test_a_missing_file_is_silently_not_registered(tmp_path, tagdb):
    node = _node(tmp_path, tagdb, active='floor1')
    assert node._registered('tag36h11', 0) is None and node.logger.warns == []


def test_a_locked_database_skips_the_frame_instead_of_blocking_or_raising(
        tmp_path, tagdb):
    _tag(tagdb, 0, 'elev', floor2=(3.0, 3.0, 0.0))
    node = _node(tmp_path, tagdb, active='floor1', busy=0.05)
    lock = sqlite3.connect(tagdb.DB_PATH, isolation_level=None)
    lock.execute('BEGIN EXCLUSIVE')
    try:
        assert node._registered('tag36h11', 0) is None
        _frames(node, n=1)
    finally:
        lock.execute('ROLLBACK')
        lock.close()
    assert node.load_requests == []
    assert node._registered('tag36h11', 0)['poses'] == {
        'floor2': {'x': 3.0, 'y': 3.0, 'yaw': 0.0}}


def test_an_unmigrated_registry_with_a_named_legacy_pose_still_seeds(tmp_path, tagdb):
    _write_db(tmp_path / 'tags.db', [
        TAGS_DDL, "INSERT INTO tags VALUES('tag36h11',0,'a','',0.1,1.0,2.0,0.5,NULL,'floor1')"])
    node = _frames(_node(tmp_path, tagdb, active='floor1'), n=1)
    assert node.published and node.published[0].pose.pose.position.x == 1.0
    db = sqlite3.connect(tmp_path / 'tags.db')
    tables = {r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    assert 'tag_poses' not in tables                    # the relocalizer never migrates


def test_registered_opens_the_file_read_only(tmp_path, tagdb):
    _tag(tagdb, 0, 'a', floor1=(1.0, 2.0, 0.5))
    src = ast.get_source_segment(NODE.read_text(), next(
        n for n in ast.walk(_CLASS) if isinstance(n, ast.FunctionDef)
        and n.name == '_registered'))
    assert 'mode=ro' in src and 'REGISTRY_BUSY_S' in src
