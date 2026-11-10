"""fleet_status site.json schema (docker/fleet-status/sitestore.py) and its
agreement with scout.core.sites and the scout-skills reader.

sitestore is loaded by path (stdlib only). server.py is loaded the same way with
`docker`/`requests` stubbed, so update_active_site and _site_meta run for real.
"""
import ast
import importlib.util
import json
import os
import pathlib
import sys
import types

import pytest

from scout.core import sites

_R = pathlib.Path(__file__).resolve().parents[2] / 'docker'
_FLEET = _R / 'fleet-status'
_SIBLINGS = ('bluetooth', 'gridstore', 'mapsafe', 'poi', 'tagpose', 'sitestore')


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


ss = _load(_FLEET / 'sitestore.py', 'fleet_sitestore')


def _stub_docker():
    docker = types.ModuleType('docker')
    errors = types.ModuleType('docker.errors')

    class NotFound(Exception):
        pass

    errors.NotFound = NotFound
    docker.errors = errors
    docker.from_env = lambda **_kw: object()
    return {'docker': docker, 'docker.errors': errors,
            'requests': types.ModuleType('requests')}


@pytest.fixture
def fleet(tmp_path, monkeypatch):
    saved = {n: sys.modules.pop(n, None) for n in _SIBLINGS}
    for name, mod in _stub_docker().items():
        monkeypatch.setitem(sys.modules, name, mod)
    monkeypatch.syspath_prepend(str(_FLEET))
    server = _load(_FLEET / 'server.py', 'fleet_server_under_test')
    root = tmp_path / 'sites'
    site_dir = root / 'office'
    (site_dir / 'maps').mkdir(parents=True)
    os.symlink('office', root / 'active')
    monkeypatch.setattr(server, 'SITES_DIR', str(root))
    monkeypatch.setattr(server, 'SITE_SCAFFOLD', 'pi')
    yield server, site_dir
    for name, mod in saved.items():
        if mod is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = mod


def _write(site_dir, data):
    (site_dir / 'site.json').write_text(json.dumps(data))


def _touch(site_dir, *names):
    for n in names:
        (site_dir / 'maps' / n).write_bytes(b'x')


def _skills_load_site(site_json):
    tree = ast.parse((_R / 'scout-skills' / 'server.py').read_text())
    fn = next(n for n in tree.body
              if isinstance(n, ast.FunctionDef) and n.name == '_load_site')
    ns = {'json': json, 'tagdb': types.SimpleNamespace(SITE_JSON=str(site_json))}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), 'skills_load_site', 'exec'), ns)
    return ns['_load_site']()


# --- norm_map_entry / normalize_site -------------------------------------------

def test_defaults_mirror_core():
    assert ss._MAP_DEFAULTS == sites._MAP_DEFAULTS
    assert ss.SOURCES == sites.SOURCES
    assert ss.SLAM_MODES == sites.SLAM_MODES
    assert ss.SITE_NAME_RE.pattern == sites.SITE_NAME_RE.pattern


def test_norm_entry_defaults_source_to_scan():
    assert ss.norm_map_entry('a', None) == {
        'label': 'a', 'floor': None, 'source': 'scan',
        'map_start_pose': [0.0, 0.0, 0.0]}


def test_norm_entry_keeps_drawing_source():
    assert ss.norm_map_entry('a', {'source': 'drawing'})['source'] == 'drawing'


@pytest.mark.parametrize('raw', ['bim', 'Drawing', '', 7, ['drawing'], None])
def test_norm_entry_unknown_source_reads_as_scan(raw):
    assert ss.norm_map_entry('a', {'source': raw})['source'] == 'scan'


def test_normalize_v1_has_scan_source():
    site = ss.normalize_site({'default_map': 'office', 'map_start_pose': [1, 2, 3]})
    assert site['active_map'] == 'office'
    assert site['maps']['office']['source'] == 'scan'
    assert site['maps']['office']['map_start_pose'] == [1, 2, 3]


def test_normalize_v2_keeps_source_per_map():
    site = ss.normalize_site({'active_map': 'plan', 'maps': {
        'plan': {'source': 'drawing'}, 'scanned': {}}})
    assert site['maps']['plan']['source'] == 'drawing'
    assert site['maps']['scanned']['source'] == 'scan'


@pytest.mark.parametrize('bad', [None, [], 'x', 3])
def test_normalize_non_dict_is_empty_site(bad):
    assert ss.normalize_site(bad)['maps'] == {}


# --- validate_map_entry / apply_maps_patch ---------------------------------------

@pytest.mark.parametrize('source', ['scan', 'drawing'])
def test_validate_accepts_known_sources(source):
    assert ss.validate_map_entry({'source': source}) is None


@pytest.mark.parametrize('source', ['bim', '', 'Scan', None, 3, ['scan']])
def test_validate_rejects_other_sources(source):
    assert 'source must be one of scan, drawing' in ss.validate_map_entry(
        {'source': source})


def test_validate_keeps_existing_rules():
    assert 'floor' in ss.validate_map_entry({'floor': 'one'})
    assert 'map_start_pose' in ss.validate_map_entry({'map_start_pose': [0, 0]})
    assert ss.validate_map_entry({'floor': None, 'label': 'x'}) is None


def test_patch_adds_drawing_entry():
    site = ss.normalize_site({})
    assert ss.apply_maps_patch(site, {'plan': {'source': 'drawing', 'floor': 2}}) is None
    assert site['maps']['plan']['source'] == 'drawing'
    assert site['maps']['plan']['floor'] == 2
    assert site['maps']['plan']['label'] == 'plan'


def test_patch_without_source_keeps_existing_source():
    site = ss.normalize_site({'maps': {'plan': {'source': 'drawing'}}})
    assert ss.apply_maps_patch(site, {'plan': {'label': 'Floor 1 plan'}}) is None
    assert site['maps']['plan']['source'] == 'drawing'
    assert site['maps']['plan']['label'] == 'Floor 1 plan'


def test_patch_rejects_unknown_source_and_leaves_entry():
    site = ss.normalize_site({'maps': {'plan': {'source': 'drawing'}}})
    assert 'source' in ss.apply_maps_patch(site, {'plan': {'source': 'bim'}})
    assert site['maps']['plan']['source'] == 'drawing'


def test_patch_null_deletes_and_clears_active():
    site = ss.normalize_site({'active_map': 'plan', 'maps': {'plan': {}}})
    assert ss.apply_maps_patch(site, {'plan': None}) is None
    assert site['maps'] == {} and site['active_map'] is None


def test_patch_rejects_bad_name_and_non_object():
    site = ss.normalize_site({})
    assert 'invalid map name' in ss.apply_maps_patch(site, {'Bad Name': {}})
    assert 'must be an object' in ss.apply_maps_patch(site, {'plan': 'drawing'})


# --- site_files / resolved_mode / describe_site ---------------------------------

def test_site_files_flags_and_skips_internal(tmp_path):
    maps = tmp_path / 'maps'
    maps.mkdir()
    for n in ('a.posegraph', 'a.data', 'a.yaml', 'a.pgm', 'b.yaml', 'b.pgm',
              'c.posegraph', '_autosave.posegraph', '_pending.yaml', 'a.drawing.json'):
        (maps / n).write_bytes(b'x')
    assert ss.site_files(str(tmp_path)) == {
        'a': {'posegraph': True, 'grid': True}, 'b': {'grid': True},
        'c': {'posegraph': True}}


def test_site_files_missing_dir_is_empty(tmp_path):
    assert ss.site_files(str(tmp_path)) == {}


def test_describe_reports_source_flags_and_resolved_mode():
    meta = ss.normalize_site({'active_map': 'plan', 'maps': {
        'plan': {'source': 'drawing'}, 'scanned': {}}})
    files = {'plan': {'grid': True}, 'scanned': {'posegraph': True, 'grid': True}}
    ss.describe_site(meta, files)
    assert meta['maps']['plan'] == {
        'label': 'plan', 'floor': None, 'source': 'drawing',
        'map_start_pose': [0.0, 0.0, 0.0], 'posegraph': False, 'grid': True}
    assert meta['maps']['scanned']['source'] == 'scan'
    assert meta['maps']['scanned']['posegraph'] is True
    assert meta['resolved_mode'] == 'localization'


def test_describe_lists_unregistered_files_as_scan():
    meta = ss.describe_site(ss.normalize_site({}), {'orphan': {'grid': True}})
    assert meta['maps']['orphan']['unregistered'] is True
    assert meta['maps']['orphan']['source'] == 'scan'
    assert meta['resolved_mode'] == 'new'


def test_describe_without_site_json_has_empty_maps():
    meta = ss.describe_site({'name': 'x'}, {})
    assert meta['maps'] == {} and meta['resolved_mode'] == 'new'


def test_explicit_mode_reported_as_is():
    for want in ('new', 'localization', 'continue'):
        assert ss.resolved_mode(want, None, {}, {}) == want


def test_resolved_mode_agrees_with_core_resolve_slam(tmp_path):
    n = 0
    for source in ('scan', 'drawing', 'bim'):
        for graph in (False, True):
            for grid in (False, True):
                for mode in ss.SLAM_MODES:
                    n += 1
                    site_dir = tmp_path / f'case{n}'
                    (site_dir / 'maps').mkdir(parents=True)
                    if graph:
                        (site_dir / 'maps' / 'floor.posegraph').write_bytes(b'x')
                    if grid:
                        (site_dir / 'maps' / 'floor.yaml').write_bytes(b'x')
                    raw = {'active_map': 'floor', 'slam_mode': mode,
                           'maps': {'floor': {'source': source}}}
                    _write(site_dir, raw)
                    want, _, _ = sites.resolve_slam(
                        sites.load_site(str(site_dir)), str(site_dir / 'maps'))
                    got = ss.resolved_mode(
                        mode, 'floor', ss.normalize_site(raw)['maps'],
                        ss.site_files(str(site_dir)))
                    assert got == want, (source, graph, grid, mode)
    assert n == 48


# --- schema agreement across every site.json reader ----------------------------

_AGREE = {
    'v1_default_map': ({'version': 1, 'default_map': 'office',
                        'map_start_pose': [1.0, 2.0, 0.5]}, 'scan'),
    'v2_no_source': ({'version': 2, 'active_map': 'office',
                      'maps': {'office': {'label': 'Lobby'}}}, 'scan'),
    'v2_scan': ({'version': 2, 'active_map': 'office',
                 'maps': {'office': {'source': 'scan'}}}, 'scan'),
    'v2_drawing': ({'version': 2, 'active_map': 'office',
                    'maps': {'office': {'source': 'drawing'}}}, 'drawing'),
}


@pytest.mark.parametrize('case', sorted(_AGREE))
def test_every_reader_agrees_on_source(case, fleet, tmp_path):
    raw, want = _AGREE[case]
    server, site_dir = fleet
    _write(site_dir, raw)
    site_json = site_dir / 'site.json'
    core = sites.load_site(str(site_dir))['maps']['office']['source']
    store = ss.normalize_site(raw)['maps']['office']['source']
    skills_site = _skills_load_site(site_json)
    skills = skills_site['maps']['office'].get('source', 'scan')
    status = server._site_meta('office')['maps']['office']['source']
    assert (core, store, skills, status) == (want,) * 4


def test_skills_reader_passes_other_entry_keys_through(fleet):
    raw, _ = _AGREE['v2_drawing']
    server, site_dir = fleet
    _write(site_dir, raw)
    site = _skills_load_site(site_dir / 'site.json')
    assert site['active_map'] == 'office' and site['slam_mode'] == 'auto'
    assert site['maps']['office'] == {'source': 'drawing'}


# --- server.py wiring: update_active_site, _site_meta ---------------------------

def test_update_rejects_unknown_source_and_writes_nothing(fleet):
    server, site_dir = fleet
    _write(site_dir, {'version': 2, 'active_map': 'floor1',
                      'maps': {'floor1': {}}})
    before = (site_dir / 'site.json').read_bytes()
    status, body = server.update_active_site({'maps': {'plan': {'source': 'bim'}}})
    assert status == 400 and 'source must be one of' in body['error']
    assert (site_dir / 'site.json').read_bytes() == before


def test_update_rejects_null_source(fleet):
    server, site_dir = fleet
    _write(site_dir, {'version': 2, 'maps': {'plan': {'source': 'drawing'}}})
    status, _ = server.update_active_site({'maps': {'plan': {'source': None}}})
    assert status == 400


def test_update_roundtrip_preserves_source_through_core_load_site(fleet):
    server, site_dir = fleet
    _write(site_dir, {'version': 2, 'active_map': 'floor1',
                      'maps': {'floor1': {'label': 'Lobby', 'floor': 1}}})
    _touch(site_dir, 'floor1.posegraph', 'floor1.data', 'floor1.yaml', 'floor1.pgm',
           'floor1_plan.yaml', 'floor1_plan.pgm')
    status, body = server.update_active_site({'maps': {'floor1_plan': {
        'source': 'drawing', 'floor': 1, 'label': 'Lobby (drawing)'}}})
    assert status == 200
    assert body['site']['maps']['floor1_plan']['source'] == 'drawing'
    loaded = sites.load_site(str(site_dir))
    assert loaded['maps']['floor1_plan']['source'] == 'drawing'
    assert loaded['maps']['floor1'] == {
        'label': 'Lobby', 'floor': 1, 'source': 'scan',
        'map_start_pose': [0.0, 0.0, 0.0]}
    assert (site_dir / 'maps' / 'floor1.posegraph').exists()


def test_update_of_other_fields_keeps_drawing_source(fleet):
    server, site_dir = fleet
    _write(site_dir, {'version': 2, 'active_map': 'plan',
                      'maps': {'plan': {'source': 'drawing'}}})
    _touch(site_dir, 'plan.yaml', 'plan.pgm')
    assert server.update_active_site({'slam_mode': 'auto'})[0] == 200
    assert server.update_active_site({'map_start_pose': [1.0, 2.0, 0.0]})[0] == 200
    assert server.update_active_site(
        {'maps': {'plan': {'label': 'Plan'}}})[0] == 200
    assert sites.load_site(str(site_dir))['maps']['plan']['source'] == 'drawing'


def test_update_active_map_registers_new_entry_as_scan(fleet):
    server, site_dir = fleet
    _write(site_dir, {'version': 2, 'maps': {}})
    _touch(site_dir, 'saved.posegraph', 'saved.data')
    status, body = server.update_active_site({'active_map': 'saved'})
    assert status == 200 and body['site']['maps']['saved']['source'] == 'scan'


def test_site_meta_reports_source_and_resolved_mode(fleet):
    server, site_dir = fleet
    _write(site_dir, {'version': 2, 'active_map': 'plan', 'slam_mode': 'auto',
                      'maps': {'plan': {'source': 'drawing'}, 'floor1': {}}})
    _touch(site_dir, 'plan.yaml', 'plan.pgm', 'floor1.posegraph', 'floor1.data')
    meta = server._site_meta('office')
    assert meta['resolved_mode'] == 'localization'
    assert meta['maps']['plan']['source'] == 'drawing'
    assert meta['maps']['plan']['grid'] is True
    assert meta['maps']['plan']['posegraph'] is False
    assert meta['maps']['floor1']['source'] == 'scan'
    assert meta['maps']['floor1']['posegraph'] is True
    server.update_active_site({'active_map': 'floor1'})
    assert server._site_meta('office')['resolved_mode'] == 'continue'


def test_site_meta_unreadable_site_json_still_reports(fleet):
    server, site_dir = fleet
    (site_dir / 'site.json').write_text('{not json')
    _touch(site_dir, 'orphan.yaml')
    meta = server._site_meta('office')
    assert meta['maps']['orphan']['source'] == 'scan'
    assert meta['resolved_mode'] == 'new'
