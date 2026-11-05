"""fleet_status grid store (docker/fleet-status/gridstore.py) + agreement with
the scout-skills validator (docker/scout-skills/grid.py)."""
import importlib.util
import json
import pathlib

import pytest

_R = pathlib.Path(__file__).resolve().parents[2] / 'docker'


def _load(rel, name):
    spec = importlib.util.spec_from_file_location(name, _R / rel)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


gs = _load('fleet-status/gridstore.py', 'fleet_gridstore')
sk = _load('scout-skills/grid.py', 'skills_grid')


def _ok(**kw):
    b = {'version': 1, 'units': 'ft', 'origin': {'x': 1, 'y': 2, 'yaw_rad': 0.5},
         'numbers': {'labels': ['1', '2', '3'], 'offsets_m': [0, 3.6576, 9.0]},
         'letters': {'labels': ['A', 'B'], 'offsets_m': [0, 6]}}
    b.update(kw)
    return b


CASES = [
    _ok(),
    _ok(units='m'),
    _ok(units='yd'),
    _ok(version=2),
    [1],
    _ok(origin={'x': float('nan'), 'y': 0, 'yaw_rad': 0}),
    _ok(origin={'x': 0, 'y': 0, 'yaw_rad': 9}),
    _ok(origin=None),
    _ok(numbers={'labels': ['1'], 'offsets_m': [0]}),
    _ok(numbers={'labels': ['1', '2'], 'offsets_m': [0]}),
    _ok(numbers={'labels': ['1', '2'], 'offsets_m': [3, 3]}),
    _ok(numbers={'labels': ['1', '2'], 'offsets_m': [0, 1e9]}),
    _ok(letters={'labels': ['A', 'a'], 'offsets_m': [0, 3]}),
    _ok(letters={'labels': ['1', 'B'], 'offsets_m': [0, 3]}),
    _ok(letters={'labels': ['A b', 'B'], 'offsets_m': [0, 3]}),
    _ok(letters={'labels': ['A', 'B'], 'offsets_m': [0, True]}),
    _ok(numbers={'labels': [' 1', '2 '], 'offsets_m': [0, 3]}),
    _ok(bay_anchor='upper'),
    _ok(bay_anchor='lower'),
    {},
]


@pytest.mark.parametrize('case', CASES)
def test_validators_agree(case):
    def run(mod):
        try:
            return mod.validate(case)
        except mod.GridError:
            return 'rejected'
    assert run(gs) == run(sk)


@pytest.fixture
def env(tmp_path):
    maps = tmp_path / 'maps'
    site = tmp_path / 'site.json'
    site.write_text(json.dumps({'active_map': 'floor1'}))
    return str(maps), str(site)


def test_get_empty_then_roundtrip(env):
    maps, site = env
    assert gs.get_grid(maps, site) == (200, {'grid': None})
    code, out = gs.put_grid(maps, site, _ok())
    assert code == 200 and out['grid']['bay_anchor'] == 'lower'
    code, got = gs.get_grid(maps, site)
    assert got['grid'] == out['grid'] and got['map'] == 'floor1'
    assert pathlib.Path(maps, 'floor1.grid.json').exists()
    assert not pathlib.Path(maps, 'floor1.grid.json.tmp').exists()
    assert gs.delete_grid(maps, site) == (200, {'grid': None})
    assert gs.get_grid(maps, site)[1]['grid'] is None
    assert gs.delete_grid(maps, site)[0] == 200


def test_post_rejects_bad(env):
    maps, site = env
    code, out = gs.put_grid(maps, site, _ok(units='yd'))
    assert code == 400 and 'units' in out['error']
    assert not pathlib.Path(maps).exists() or not list(pathlib.Path(maps).iterdir())


def test_session_fallback_and_bad_names(tmp_path):
    maps = str(tmp_path)
    for body in ({'active_map': None}, {'active_map': '../evil'}, 'garbage'):
        site = tmp_path / 's.json'
        site.write_text(body if isinstance(body, str) else json.dumps(body))
        assert gs.grid_path(maps, str(site)).endswith('/_session.grid.json')
    assert gs.grid_path(maps, str(tmp_path / 'missing.json')).endswith('_session.grid.json')


def test_corrupt_file_reads_as_none(env):
    maps, site = env
    pathlib.Path(maps).mkdir()
    pathlib.Path(maps, 'floor1.grid.json').write_text('{nope')
    code, out = gs.get_grid(maps, site)
    assert code == 200 and out['grid'] is None and 'error' in out


def test_adopt_session(tmp_path):
    maps = str(tmp_path)
    assert gs.adopt_session(maps, 'newmap')[1]['adopted'] is False
    (tmp_path / '_session.grid.json').write_text(json.dumps(_ok()))
    assert gs.adopt_session(maps, 'newmap') == (200, {'adopted': True})
    assert (tmp_path / 'newmap.grid.json').exists()
    assert not (tmp_path / '_session.grid.json').exists()
    (tmp_path / '_session.grid.json').write_text('{}')
    assert gs.adopt_session(maps, 'newmap')[1]['adopted'] is False  # no clobber
    assert gs.adopt_session(maps, '../x')[0] == 400
    assert gs.adopt_session(maps, '_session')[0] == 400
