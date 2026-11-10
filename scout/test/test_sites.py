"""Location site store (ADR-0023, multi-map v2 ADR-0029): name contract,
site.json v1->v2 normalization, and the slam-mode resolution policy that
mode:=site feeds into ADR-0003's table."""

import json
import os

import pytest

from scout.core import sites


def _write_site(tmp_path, **data):
    with open(os.path.join(tmp_path, 'site.json'), 'w') as f:
        json.dump(data, f)
    return str(tmp_path)


# --- name contract (duplicated in fleet-status + inspection recorder) --------

@pytest.mark.parametrize('name', ['office', 'jobsite-42', 'a', 'x' * 32, 'a_b'])
def test_valid_names(name):
    assert sites.valid_name(name)


@pytest.mark.parametrize('name', [
    'active',           # reserved: the symlink itself
    '', None, 'Office', 'a b', '-lead', '_lead', 'x' * 33, '../evil', 'a/b',
])
def test_invalid_names(name):
    assert not sites.valid_name(name)


def test_map_name_re_is_site_name_re():
    # Shared contract: map names obey the site-name grammar.
    assert sites.MAP_NAME_RE is sites.SITE_NAME_RE


# --- active symlink -----------------------------------------------------------

def test_active_site_name(tmp_path):
    assert sites.active_site_name(str(tmp_path)) is None
    (tmp_path / 'office').mkdir()
    os.symlink('office', tmp_path / 'active')
    assert sites.active_site_name(str(tmp_path)) == 'office'


# --- load_site ------------------------------------------------------------------

def test_load_site_defaults(tmp_path):
    site = sites.load_site(_write_site(tmp_path, version=1))
    assert site['active_map'] is None
    assert site['maps'] == {}
    assert site['slam_mode'] == 'auto'


def test_load_site_v1_upgrades(tmp_path):
    # v1's default_map + top-level map_start_pose fold into one maps entry.
    site = sites.load_site(_write_site(
        tmp_path, version=1, default_map='office',
        map_start_pose=[1.0, 2.0, 0.5]))
    assert site['active_map'] == 'office'
    assert site['maps'] == {'office': {
        'label': 'office', 'floor': None, 'source': 'scan',
        'map_start_pose': [1.0, 2.0, 0.5]}}


def test_load_site_v2_roundtrip(tmp_path):
    site = sites.load_site(_write_site(
        tmp_path, version=2, active_map='floor1',
        maps={'floor1': {'label': 'Lobby', 'floor': 1,
                         'map_start_pose': [1.0, 0.0, 0.0]},
              'yard': {}}))
    assert site['active_map'] == 'floor1'
    assert site['maps']['floor1'] == {
        'label': 'Lobby', 'floor': 1, 'source': 'scan',
        'map_start_pose': [1.0, 0.0, 0.0]}
    # Per-map defaults fill in; label defaults to the map name.
    assert site['maps']['yard'] == {
        'label': 'yard', 'floor': None, 'source': 'scan',
        'map_start_pose': [0.0, 0.0, 0.0]}


def test_load_site_active_map_must_exist(tmp_path):
    with pytest.raises(ValueError):
        sites.load_site(_write_site(tmp_path, version=2, active_map='ghost',
                                    maps={'floor1': {}}))


def test_load_site_missing_file_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        sites.load_site(str(tmp_path))


def test_load_site_bad_mode_raises(tmp_path):
    with pytest.raises(ValueError):
        sites.load_site(_write_site(tmp_path, slam_mode='mapping'))


def test_load_site_null_values_fall_back(tmp_path):
    site = sites.load_site(_write_site(tmp_path, default_map=None,
                                       slam_mode=None))
    assert site['slam_mode'] == 'auto'
    assert site['maps'] == {}


# --- resolve_slam: the auto policy ---------------------------------------------

def _site(active_map=None, slam_mode='auto', pose=(0.0, 0.0, 0.0), source='scan',
          **maps):
    all_maps = {active_map: {'label': active_map, 'floor': None, 'source': source,
                             'map_start_pose': list(pose)}} if active_map else {}
    for name, entry in maps.items():
        all_maps[name] = {'label': name, 'floor': None, 'source': 'scan',
                          'map_start_pose': [0.0, 0.0, 0.0], **entry}
    return {'active_map': active_map, 'slam_mode': slam_mode, 'maps': all_maps}


def test_auto_without_map_is_new(tmp_path):
    mode, _, _ = sites.resolve_slam(_site(), str(tmp_path))
    assert mode == 'new'


def test_auto_with_named_but_missing_map_is_new(tmp_path):
    # active_map set but never serialized (fresh site named before mapping).
    mode, _, _ = sites.resolve_slam(_site(active_map='office'), str(tmp_path))
    assert mode == 'new'


def test_auto_with_saved_map_is_continue(tmp_path):
    # continue, never localization: serialize_map silently no-ops there.
    (tmp_path / 'office.posegraph').touch()
    mode, map_name, _ = sites.resolve_slam(_site(active_map='office'),
                                           str(tmp_path))
    assert (mode, map_name) == ('continue', 'office')


def test_auto_keyed_on_active_map_not_others(tmp_path):
    # Another map's posegraph must not flip the active map to continue.
    (tmp_path / 'floor2.posegraph').touch()
    mode, _, _ = sites.resolve_slam(
        _site(active_map='floor1', floor2={}), str(tmp_path))
    assert mode == 'new'


def test_explicit_modes_pass_through(tmp_path):
    for want in ('new', 'localization', 'continue'):
        site = _site(active_map='office', slam_mode=want,
                     pose=(1.5, 0.0, 3.14))
        mode, _, pose = sites.resolve_slam(site, str(tmp_path))
        assert mode == want
    assert pose == [1.5, 0.0, 3.14]


def test_explicit_load_mode_without_map_raises(tmp_path):
    for want in ('localization', 'continue'):
        with pytest.raises(ValueError):
            sites.resolve_slam(_site(slam_mode=want), str(tmp_path))


def test_bad_start_pose_raises(tmp_path):
    with pytest.raises(ValueError):
        sites.resolve_slam(_site(active_map='office', pose=(1.0, 2.0)),
                           str(tmp_path))


# --- map source (scan | drawing) -------------------------------------------------

def test_load_site_keeps_drawing_source(tmp_path):
    site = sites.load_site(_write_site(
        tmp_path, version=2, active_map='plan',
        maps={'plan': {'source': 'drawing'}, 'scanned': {}}))
    assert site['maps']['plan']['source'] == 'drawing'
    assert site['maps']['scanned']['source'] == 'scan'


@pytest.mark.parametrize('raw', ['bim', 'Drawing', '', 7, ['drawing'], None])
def test_load_site_unknown_source_reads_as_scan(tmp_path, raw):
    site = sites.load_site(_write_site(
        tmp_path, version=2, maps={'plan': {'source': raw}}))
    assert site['maps']['plan']['source'] == 'scan'


def test_source_is_a_known_map_key():
    assert sites.SOURCES == ('scan', 'drawing')
    assert sites._MAP_DEFAULTS['source'] == 'scan'


def test_auto_drawing_with_grid_is_localization(tmp_path):
    (tmp_path / 'plan.yaml').touch()
    (tmp_path / 'plan.pgm').touch()
    mode, map_name, pose = sites.resolve_slam(
        _site(active_map='plan', source='drawing', pose=(4.0, -2.5, 1.57)),
        str(tmp_path))
    assert (mode, map_name, pose) == ('localization', 'plan', [4.0, -2.5, 1.57])


def test_auto_drawing_without_grid_is_not_localization(tmp_path):
    mode, _, _ = sites.resolve_slam(
        _site(active_map='plan', source='drawing'), str(tmp_path))
    assert mode == 'new'


def test_auto_drawing_without_grid_but_with_posegraph_is_continue(tmp_path):
    (tmp_path / 'plan.posegraph').touch()
    mode, _, _ = sites.resolve_slam(
        _site(active_map='plan', source='drawing'), str(tmp_path))
    assert mode == 'continue'


def test_auto_drawing_with_grid_and_posegraph_is_localization(tmp_path):
    # Source decides: a force-installed drawing over an old scan keeps localizing.
    (tmp_path / 'plan.yaml').touch()
    (tmp_path / 'plan.posegraph').touch()
    mode, _, _ = sites.resolve_slam(
        _site(active_map='plan', source='drawing'), str(tmp_path))
    assert mode == 'localization'


def test_auto_scan_with_grid_only_is_still_new(tmp_path):
    # The old "no posegraph" signature must not flip a scan map to localization.
    (tmp_path / 'office.yaml').touch()
    (tmp_path / 'office.pgm').touch()
    mode, _, _ = sites.resolve_slam(_site(active_map='office'), str(tmp_path))
    assert mode == 'new'


def test_auto_scan_with_grid_and_posegraph_is_continue(tmp_path):
    (tmp_path / 'office.yaml').touch()
    (tmp_path / 'office.posegraph').touch()
    mode, _, _ = sites.resolve_slam(_site(active_map='office'), str(tmp_path))
    assert mode == 'continue'


def test_auto_scan_with_quarantined_graph_and_autosave_is_unchanged(tmp_path):
    # Posegraph pair quarantined, a valid _autosave beside it, grid still on
    # disk: resolves exactly as before this change (new; the launch adopts the
    # autosave), never localization.
    (tmp_path / 'office.yaml').touch()
    (tmp_path / '_autosave.posegraph').write_bytes(b'x' * 60_000)
    (tmp_path / '_autosave.data').write_bytes(b'x' * 5_000)
    quarantine = tmp_path / '_quarantine'
    quarantine.mkdir()
    (quarantine / 'office.posegraph').write_bytes(b'x' * 60_000)
    (quarantine / 'office.data').write_bytes(b'x' * 5_000)
    legacy = {'active_map': 'office', 'slam_mode': 'auto', 'maps': {
        'office': {'label': 'office', 'floor': None,
                   'map_start_pose': [0.0, 0.0, 0.0]}}}
    assert sites.resolve_slam(_site(active_map='office'), str(tmp_path)) == \
        sites.resolve_slam(legacy, str(tmp_path)) == ('new', 'office', [0.0] * 3)


def test_auto_keyed_on_active_source_not_others(tmp_path):
    (tmp_path / 'plan.yaml').touch()
    mode, _, _ = sites.resolve_slam(
        _site(active_map='floor1', plan={'source': 'drawing'}), str(tmp_path))
    assert mode == 'new'


@pytest.mark.parametrize('want', ['new', 'localization', 'continue'])
def test_explicit_modes_ignore_source(tmp_path, want):
    (tmp_path / 'plan.yaml').touch()
    mode, _, _ = sites.resolve_slam(
        _site(active_map='plan', source='drawing', slam_mode=want),
        str(tmp_path))
    assert mode == want


def test_hand_built_entry_without_source_resolves_as_scan(tmp_path):
    (tmp_path / 'office.yaml').touch()
    site = {'active_map': 'office', 'slam_mode': 'auto', 'maps': {
        'office': {'map_start_pose': [0.0, 0.0, 0.0]}}}
    assert sites.resolve_slam(site, str(tmp_path))[0] == 'new'
