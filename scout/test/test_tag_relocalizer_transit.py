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
        self.get_parameter = lambda n: types.SimpleNamespace(value=params[n])
        self._load_map_cli = types.SimpleNamespace(
            service_is_ready=lambda: ready, call_async=self._call_async)
        self._on_map_loaded = lambda *a: None

    def get_logger(self):
        return self.logger

    def _call_async(self, req):
        self.load_requests.append(req.map_url)
        return _Future()


def _track(maps_dir, maps, active, home, *, sightings=3, slam_mode='localization',
           ready=True):
    node = _Self(maps_dir, ready)
    fake_loadmap = types.SimpleNamespace(
        Request=lambda: types.SimpleNamespace(map_url=None),
        Response=types.SimpleNamespace(RESULT_SUCCESS=0))
    track = _method('_track_transit', sites=sites, os=__import__('os'),
                    time=types.SimpleNamespace(monotonic=lambda: NOW),
                    LoadMap=fake_loadmap)
    for _ in range(sightings):
        track(node, home, 1.0, 'tag7', 0.0, 'pose',
              slam_mode=slam_mode, active_map=active, maps=maps)
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
        track(node, 'other_scan', 1.0, 'tag7', 0.0, 'pose',
              slam_mode='localization', active_map='scanned', maps=MAPS)
    assert node.load_requests == []
    assert 'nav goal' in node.logger.warns[0][0]


def _read_site(tmp_path, content):
    path = tmp_path / 'site.json'
    if content is not None:
        path.write_text(content if isinstance(content, str) else json.dumps(content))
    node = _Self(tmp_path)
    return _method('_read_site', json=json)(node)


def test_read_site_returns_maps_for_v2(tmp_path):
    data = {'active_map': 'plan', 'slam_mode': 'auto', 'maps': MAPS}
    assert _read_site(tmp_path, data) == ('plan', 'auto', MAPS)


def test_read_site_v1_has_no_maps(tmp_path):
    assert _read_site(tmp_path, {'default_map': 'office'}) == ('office', None, {})


@pytest.mark.parametrize('content', [None, '{not json', '[]', '"x"'])
def test_read_site_problem_disables_transit(tmp_path, content):
    assert _read_site(tmp_path, content) == (None, None, {})


def test_read_site_non_dict_maps_is_empty(tmp_path):
    assert _read_site(tmp_path, {'active_map': 'a', 'maps': ['a']})[2] == {}
