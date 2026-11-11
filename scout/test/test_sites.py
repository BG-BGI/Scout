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


# --- active map source / cross-source detection ----------------------------------

def _two_source_site(tmp_path, active):
    return sites.load_site(_write_site(
        tmp_path, version=2, active_map=active, slam_mode='auto',
        maps={'plan': {'source': 'drawing'}, 'scanned': {}, 'other_plan':
              {'source': 'drawing'}}))


def test_active_map_source_drawing(tmp_path):
    assert sites.active_map_source(_two_source_site(tmp_path, 'plan')) == 'drawing'


def test_active_map_source_scan_ignores_other_drawing_maps(tmp_path):
    assert sites.active_map_source(_two_source_site(tmp_path, 'scanned')) == 'scan'


def test_active_map_source_v1_site_is_scan(tmp_path):
    site = sites.load_site(_write_site(
        tmp_path, version=1, default_map='office', map_start_pose=[1, 2, 0]))
    assert sites.active_map_source(site) == 'scan'


def test_active_map_source_no_active_map_is_scan(tmp_path):
    site = sites.load_site(_write_site(
        tmp_path, version=2, maps={'plan': {'source': 'drawing'}}))
    assert site['active_map'] is None
    assert sites.active_map_source(site) == 'scan'


def test_active_map_source_unknown_value_is_scan(tmp_path):
    site = sites.load_site(_write_site(
        tmp_path, version=2, active_map='plan',
        maps={'plan': {'source': 'bim'}}))
    assert sites.active_map_source(site) == 'scan'


@pytest.mark.parametrize('site', [
    {}, {'maps': None}, {'maps': []}, {'active_map': 'x'},
    {'active_map': 'x', 'maps': {'x': None}},
    {'active_map': 'x', 'maps': {'x': 'drawing'}},
    {'active_map': 'x', 'maps': {'x': {'source': ['drawing']}}},
    {'active_map': 'x', 'maps': {'x': {}}},
])
def test_active_map_source_tolerates_hand_built_sites(site):
    assert sites.active_map_source(site) == 'scan'


def test_active_map_source_reads_raw_entries(tmp_path):
    raw = {'active_map': 'plan', 'maps': {'plan': {'source': 'drawing'}}}
    assert sites.active_map_source(raw) == 'drawing'


@pytest.mark.parametrize('current,target,expect', [
    ('scanned', 'plan', True),
    ('plan', 'scanned', True),
    ('plan', 'other_plan', False),
    ('scanned', 'scanned', False),
    ('scanned', 'ghost', False),
    ('ghost', 'plan', True),
    (None, 'plan', True),
    (None, 'scanned', False),
])
def test_sources_differ(tmp_path, current, target, expect):
    site = _two_source_site(tmp_path, 'scanned')
    assert sites.sources_differ(site['maps'], current, target) is expect


def test_sources_differ_tolerates_missing_maps():
    assert sites.sources_differ(None, 'a', 'b') is False


# --- effective_mode: the one rule behind resolve_slam and the transit gate ----------

def _touch(path, *names):
    for n in names:
        (path / n).write_bytes(b'x')


def _norm(active, slam_mode='auto', **sources):
    maps = {n: {'source': s} for n, s in sources.items()}
    return sites.site_from_data(
        {'version': 2, 'active_map': active, 'slam_mode': slam_mode, 'maps': maps})


@pytest.mark.parametrize('mode', ['new', 'continue', 'localization'])
def test_effective_mode_pinned_modes_pass_through(tmp_path, mode):
    site = _norm('plan', mode, plan='drawing')
    assert sites.effective_mode(site, str(tmp_path)) == mode


def test_effective_mode_auto_drawing_with_grid_is_localization(tmp_path):
    _touch(tmp_path, 'plan.yaml')
    assert sites.effective_mode(_norm('plan', plan='drawing'), str(tmp_path)) == 'localization'


def test_effective_mode_auto_drawing_without_grid_is_not_localization(tmp_path):
    assert sites.effective_mode(_norm('plan', plan='drawing'), str(tmp_path)) == 'new'


@pytest.mark.parametrize('files,want', [([], 'new'), (['m.posegraph'], 'continue'),
                                        (['m.yaml'], 'new'),
                                        (['m.yaml', 'm.posegraph'], 'continue')])
def test_effective_mode_auto_scan_never_localizes(tmp_path, files, want):
    _touch(tmp_path, *files)
    assert sites.effective_mode(_norm('m', m='scan'), str(tmp_path)) == want


def test_effective_mode_auto_without_active_map_is_new(tmp_path):
    assert sites.effective_mode(_norm(None, plan='drawing'), str(tmp_path)) == 'new'


@pytest.mark.parametrize('slam_mode', ['auto', 'new', 'continue', 'localization'])
@pytest.mark.parametrize('source', ['scan', 'drawing'])
@pytest.mark.parametrize('files', [[], ['m.yaml'], ['m.posegraph'],
                                   ['m.yaml', 'm.posegraph']])
def test_effective_mode_is_what_resolve_slam_uses(tmp_path, slam_mode, source, files):
    d = tmp_path / ('-'.join(files) or 'none')
    d.mkdir(exist_ok=True)
    _touch(d, *files)
    site = _norm('m', slam_mode, m=source)
    assert sites.effective_mode(site, str(d)) == sites.resolve_slam(site, str(d))[0]


def test_effective_mode_from_data_reads_raw_site_json(tmp_path):
    _touch(tmp_path, 'plan.yaml')
    data = {'version': 2, 'active_map': 'plan', 'slam_mode': 'auto',
            'maps': {'plan': {'source': 'drawing'}}}
    assert sites.effective_mode_from_data(data, str(tmp_path)) == 'localization'


def test_effective_mode_from_data_v1_scan_site(tmp_path):
    data = {'version': 1, 'default_map': 'office', 'slam_mode': 'auto'}
    assert sites.effective_mode_from_data(data, str(tmp_path)) == 'new'


@pytest.mark.parametrize('data,want', [
    ({'slam_mode': 'bogus'}, 'bogus'),
    ({'slam_mode': 'auto', 'active_map': 'ghost', 'maps': {'a': {}}}, 'auto'),
    ({'slam_mode': 'localization', 'active_map': 'ghost', 'maps': {'a': {}}},
     'localization'),
    ({}, 'new'),
])
def test_effective_mode_from_data_unnormalizable_site_keeps_raw_mode(
        tmp_path, data, want):
    assert sites.effective_mode_from_data(data, str(tmp_path)) == want


def test_site_from_data_matches_load_site(tmp_path):
    data = {'version': 2, 'active_map': 'plan', 'slam_mode': 'auto',
            'maps': {'plan': {'source': 'drawing'}, 'b': {}}}
    assert sites.site_from_data(data) == sites.load_site(_write_site(tmp_path, **data))


# --- map_source_in_active_site: explicit-map launches --------------------------------

def _active_site_root(tmp_path, **site):
    (tmp_path / 'office').mkdir()
    os.symlink('office', tmp_path / 'active')
    (tmp_path / 'office' / 'site.json').write_text(json.dumps(site))
    return str(tmp_path)


def test_explicit_map_source_reads_the_entry(tmp_path):
    root = _active_site_root(
        tmp_path, version=2, active_map='a',
        maps={'a': {}, 'plan': {'source': 'drawing'}})
    assert sites.map_source_in_active_site(root, 'plan') == 'drawing'
    assert sites.map_source_in_active_site(root, 'a') == 'scan'


def test_explicit_map_source_unknown_map_is_scan(tmp_path):
    root = _active_site_root(tmp_path, version=2, maps={'plan': {'source': 'drawing'}})
    assert sites.map_source_in_active_site(root, 'ghost') == 'scan'


def test_explicit_map_source_no_active_site_is_scan(tmp_path):
    assert sites.map_source_in_active_site(str(tmp_path), 'plan') == 'scan'


@pytest.mark.parametrize('content', ['{not json', '[]'])
def test_explicit_map_source_unreadable_site_is_scan(tmp_path, content):
    (tmp_path / 'office').mkdir()
    os.symlink('office', tmp_path / 'active')
    (tmp_path / 'office' / 'site.json').write_text(content)
    assert sites.map_source_in_active_site(str(tmp_path), 'plan') == 'scan'


def test_explicit_map_source_missing_site_json_is_scan(tmp_path):
    (tmp_path / 'office').mkdir()
    os.symlink('office', tmp_path / 'active')
    assert sites.map_source_in_active_site(str(tmp_path), 'plan') == 'scan'


def test_explicit_map_source_invalid_site_is_scan(tmp_path):
    root = _active_site_root(
        tmp_path, version=2, slam_mode='bogus', maps={'plan': {'source': 'drawing'}})
    assert sites.map_source_in_active_site(root, 'plan') == 'scan'


def test_explicit_map_source_v1_site_is_scan(tmp_path):
    root = _active_site_root(tmp_path, version=1, default_map='office')
    assert sites.map_source_in_active_site(root, 'office') == 'scan'


# --- hostile (parsed-but-malformed) site.json must never raise out of the gates ------
# effective_mode_from_data runs in tag_relocalizer's per-frame callback and
# map_source_in_active_site in a launch's explicit-map path; site_from_data /
# load_site / resolve_slam are deliberately NOT softened (launch fails loudly).

def _hostile_cases():
    pin = {'slam_mode': 'localization'}
    return [
        ('active list', {'active_map': ['a'], 'maps': {'a': {}}}, None),
        ('active list pinned', {**pin, 'active_map': ['a'], 'maps': {'a': {}}},
         'localization'),
        ('active dict', {'active_map': {'a': 1}, 'maps': {'a': {}}}, None),
        ('active dict pinned', {'slam_mode': 'continue', 'active_map': {'a': 1},
                                'maps': {'a': {}}}, 'continue'),
        ('active int', {'active_map': 5, 'maps': {'a': {}}}, None),
        ('active int pinned', {**pin, 'active_map': 5, 'maps': {'a': {}}},
         'localization'),
        ('maps list', {'active_map': 'a', 'maps': ['a']}, 'new'),
        ('maps str', {'active_map': 'a', 'maps': 'a'}, 'new'),
        ('entry str', {'active_map': 'a', 'maps': {'a': 'x'}}, 'new'),
        ('entry list', {'active_map': 'a', 'maps': {'a': []}}, 'new'),
        ('entry null', {'active_map': 'a', 'maps': {'a': None}}, 'new'),
        ('entry int', {'active_map': 'a', 'maps': {'a': 3}}, 'new'),
        ('source list', {'active_map': 'a', 'maps': {'a': {'source': ['drawing']}}},
         'new'),
        ('start pose junk',
         {'active_map': 'a', 'maps': {'a': {'map_start_pose': 'x'}}}, 'new'),
        ('mode list', {'slam_mode': ['auto']}, None),
        ('mode dict', {'slam_mode': {'a': 1}}, None),
        ('mode int', {'slam_mode': 3}, None),
        ('mode null', {'slam_mode': None}, 'new'),
        ('mode unknown str', {'slam_mode': 'bogus'}, 'bogus'),
        ('default_map list', {'default_map': ['x']}, None),
        ('default_map dict', {'default_map': {'a': 1}}, None),
        ('default_map int', {'default_map': 7}, None),
        ('default_map list pinned', {**pin, 'default_map': ['x']}, 'localization'),
        ('name slash', {'active_map': '../x', 'maps': {'../x': {'source': 'drawing'}}},
         'new'),
        ('name NUL', {'active_map': 'a\x00b', 'maps': {'a\x00b': {'source': 'drawing'}}},
         'new'),
        ('name empty', {'active_map': '', 'maps': {'': {}}}, 'new'),
        ('name huge', {'active_map': 'a' * 5000,
                       'maps': {'a' * 5000: {'source': 'drawing'}}}, 'new'),
        ('non-str key in maps', {'active_map': 'a', 'maps': {'a': {}, 1: {}}}, 'new'),
        ('empty dict', {}, 'new'),
        ('data list', [], None), ('data str', 'x', None), ('data int', 3, None),
        ('data null', None, None), ('data float', 1.5, None), ('data bool', True, None),
    ]


HOSTILE = _hostile_cases()


@pytest.mark.parametrize('label,data,want', HOSTILE, ids=[c[0] for c in HOSTILE])
def test_effective_mode_from_data_never_raises_on_hostile_json(
        tmp_path, label, data, want):
    assert sites.effective_mode_from_data(data, str(tmp_path)) == want


@pytest.mark.parametrize('label,data,want', HOSTILE, ids=[c[0] for c in HOSTILE])
@pytest.mark.parametrize('bad_dir', [None, 5, b'x', ['x'], {}])
def test_effective_mode_from_data_never_raises_on_a_bad_maps_dir(
        label, data, want, bad_dir):
    got = sites.effective_mode_from_data(data, bad_dir)
    assert got is None or isinstance(got, str)


def test_effective_mode_from_data_pinned_modes_need_no_maps_dir():
    data = {'slam_mode': 'localization', 'active_map': 'a', 'maps': {'a': {}}}
    assert sites.effective_mode_from_data(data, None) == 'localization'


def test_effective_mode_from_data_auto_with_a_bad_maps_dir_is_raw_auto():
    data = {'slam_mode': 'auto', 'active_map': 'a', 'maps': {'a': {}}}
    assert sites.effective_mode_from_data(data, None) == 'auto'


def test_strict_normalization_still_raises_for_launch():
    # the softening is only in effective_mode_from_data
    with pytest.raises(TypeError):
        sites.site_from_data({'active_map': ['a'], 'maps': {'a': {}}})
    with pytest.raises(ValueError):
        sites.site_from_data({'slam_mode': 'bogus'})
    with pytest.raises(ValueError):
        sites.resolve_slam(
            sites.site_from_data({'slam_mode': 'continue'}), '/nonexistent')


@pytest.mark.parametrize('label,data,want', HOSTILE, ids=[c[0] for c in HOSTILE])
@pytest.mark.parametrize('map_name', ['a', 'plan', '', '../x'])
def test_map_source_in_active_site_never_raises_on_hostile_json(
        tmp_path, label, data, want, map_name):
    (tmp_path / 'office').mkdir()
    os.symlink('office', tmp_path / 'active')
    (tmp_path / 'office' / 'site.json').write_text(json.dumps(data))
    assert sites.map_source_in_active_site(str(tmp_path), map_name) in sites.SOURCES


@pytest.mark.parametrize('name', [None, 5, ['x'], {'a': 1}, b'a'])
def test_map_source_tolerates_a_non_str_name(name):
    assert sites.map_source({'a': {'source': 'drawing'}}, name) == 'scan'
    assert sites.sources_differ({'a': {'source': 'drawing'}}, name, 'a') is True
