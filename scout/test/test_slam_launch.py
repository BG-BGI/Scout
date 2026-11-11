"""Which amcl params slam.launch.py hands the localization nodes for a given
(SCOUT_PROFILE, map source): base <- profile overlay <- source overlay (U6,
R9/AE6). The launch file itself needs ROS, so the decision lives in
robot_profile.merged_params_for_source and is tested here off-ROS; the AST
checks pin that the launch file actually routes through it."""

import ast
import json
import pathlib
import shutil
import tempfile

import pytest
import yaml

from scout import robot_profile

SCOUT = pathlib.Path(__file__).resolve().parent.parent
CONFIG = SCOUT / 'config'
LAUNCH = SCOUT / 'launch' / 'slam.launch.py'


def _load(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


def _amcl(path):
    return _load(path)['amcl']['ros__parameters']


@pytest.fixture
def real_config(monkeypatch, tmp_path):
    monkeypatch.setattr(robot_profile, '_BIND_DIR', str(CONFIG))
    monkeypatch.setattr(robot_profile, '_share_dir',
                        lambda: str(tmp_path / 'no_share'))
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    return CONFIG


@pytest.fixture
def stacked_config(monkeypatch, tmp_path):
    """Real base amcl.yaml + real drawing overlay, plus a synthetic
    tight_tunnel amcl overlay that overlaps the drawing one on two keys."""
    cfg = tmp_path / 'config'
    (cfg / 'overlays' / 'tight_tunnel').mkdir(parents=True)
    (cfg / 'source_overlays' / 'drawing').mkdir(parents=True)
    shutil.copy(CONFIG / 'amcl.yaml', cfg / 'amcl.yaml')
    shutil.copy(CONFIG / 'source_overlays' / 'drawing' / 'amcl.yaml',
                cfg / 'source_overlays' / 'drawing' / 'amcl.yaml')
    with open(cfg / 'overlays' / 'tight_tunnel' / 'amcl.yaml', 'w') as f:
        yaml.safe_dump({'amcl': {'ros__parameters': {
            'laser_max_range': 12.0, 'max_beams': 200, 'update_min_d': 0.02}}}, f)
    monkeypatch.setattr(robot_profile, '_BIND_DIR', str(cfg))
    monkeypatch.setattr(robot_profile, '_share_dir',
                        lambda: str(tmp_path / 'no_share'))
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path / 'out'))
    return cfg


def test_drawing_map_gets_likelihood_field_prob(real_config):
    params = _amcl(robot_profile.merged_params_for_source(
        'amcl.yaml', 'default', 'drawing'))
    assert params['laser_model_type'] == 'likelihood_field_prob'
    assert params['do_beamskip'] is True


def test_scan_map_keeps_base_value_and_exact_path(real_config):
    path = robot_profile.merged_params_for_source('amcl.yaml', 'default', 'scan')
    assert path == str(CONFIG / 'amcl.yaml')
    assert _amcl(path)['laser_model_type'] == 'likelihood_field'


def test_same_profile_drawing_vs_scan_differ_only_in_source_layer(real_config):
    scan = _amcl(robot_profile.merged_params_for_source(
        'amcl.yaml', 'tight_tunnel', 'scan'))
    drawing = _amcl(robot_profile.merged_params_for_source(
        'amcl.yaml', 'tight_tunnel', 'drawing'))
    assert scan['laser_model_type'] == 'likelihood_field'
    assert drawing['laser_model_type'] == 'likelihood_field_prob'
    overlay = _amcl(CONFIG / 'source_overlays' / 'drawing' / 'amcl.yaml')
    untouched = set(scan) - set(overlay)
    assert untouched
    assert {k: drawing[k] for k in untouched} == {k: scan[k] for k in untouched}


@pytest.mark.parametrize('basename', ['slam.yaml', 'nav2.yaml', 'amcl.yaml'])
@pytest.mark.parametrize('profile', ['default', 'tight_tunnel'])
def test_scan_source_is_byte_identical_to_merged_params(
        real_config, basename, profile):
    legacy = robot_profile.merged_params(basename, profile)
    via_source = robot_profile.merged_params_for_source(
        basename, profile, 'scan')
    assert via_source == legacy
    with open(legacy, 'rb') as a, open(via_source, 'rb') as b:
        assert a.read() == b.read()


def test_file_without_a_source_overlay_follows_profile_only(real_config):
    assert robot_profile.merged_params_for_source(
        'nav2.yaml', 'tight_tunnel', 'drawing') == robot_profile.merged_params(
            'nav2.yaml', 'tight_tunnel')


def test_profile_and_source_both_apply_and_source_wins(stacked_config):
    params = _amcl(robot_profile.merged_params_for_source(
        'amcl.yaml', 'tight_tunnel', 'drawing'))
    assert params['update_min_d'] == 0.02
    assert params['laser_max_range'] == 10.0
    assert params['max_beams'] == 90
    assert params['laser_model_type'] == 'likelihood_field_prob'
    assert params['min_particles'] == 1000


def test_profile_alone_leaves_overlapping_keys_at_profile_values(stacked_config):
    params = _amcl(robot_profile.merged_params_for_source(
        'amcl.yaml', 'tight_tunnel', 'scan'))
    assert params['laser_max_range'] == 12.0
    assert params['max_beams'] == 200
    assert params['laser_model_type'] == 'likelihood_field'


def test_chain_files_are_distinct_per_profile_source_pair(stacked_config):
    drawing_tt = robot_profile.merged_params_for_source(
        'amcl.yaml', 'tight_tunnel', 'drawing')
    drawing = robot_profile.merged_params_for_source(
        'amcl.yaml', 'default', 'drawing')
    scan_tt = robot_profile.merged_params_for_source(
        'amcl.yaml', 'tight_tunnel', 'scan')
    assert len({drawing_tt, drawing, scan_tt}) == 3
    assert _amcl(drawing_tt)['update_min_d'] == 0.02
    assert _amcl(drawing)['update_min_d'] == 0.05


@pytest.mark.parametrize('source', ['bim', 'Drawing', '', None, 'scan '])
def test_unknown_source_raises_instead_of_behaving_as_scan(real_config, source):
    with pytest.raises(RuntimeError, match='unknown map source'):
        robot_profile.merged_params_for_source('amcl.yaml', 'default', source)


@pytest.mark.parametrize('source', ['scan', 'drawing'])
def test_unknown_profile_raises(real_config, source):
    with pytest.raises(RuntimeError, match='unknown profile'):
        robot_profile.merged_params_for_source('amcl.yaml', 'nope', source)


def test_missing_base_file_raises(real_config):
    with pytest.raises(RuntimeError, match='not found'):
        robot_profile.merged_params_for_source('no_such.yaml', 'default', 'drawing')


def _function(name):
    tree = ast.parse(LAUNCH.read_text())
    return next(n for n in tree.body
                if isinstance(n, ast.FunctionDef) and n.name == name)


def _called_names(node):
    return {getattr(c.func, 'id', getattr(c.func, 'attr', None))
            for c in ast.walk(node) if isinstance(c, ast.Call)}


def test_launch_localization_branch_routes_through_source_helper():
    loc = _called_names(_function('_localization_nodes'))
    assert 'merged_params_for_source' in loc
    assert 'merged_params' not in loc


def test_launch_reads_source_from_the_active_site():
    assert 'active_map_source' in _called_names(_function('_launch_setup'))


def test_launch_mapping_modes_still_use_plain_merged_params():
    assert 'merged_params' in _called_names(_function('_launch_setup'))


# --- explicit `mode:=localization map:=X` takes X's source from the active site ----

def _site_root(tmp_path, content=None, **site):
    root = tmp_path / 'sites'
    (root / 'office').mkdir(parents=True)
    (root / 'active').symlink_to('office')
    if content is not None:
        (root / 'office' / 'site.json').write_text(content)
    elif site:
        (root / 'office' / 'site.json').write_text(json.dumps(site))
    return str(root)


def _explicit_localization_params(root, map_name, profile='default'):
    from scout.core import sites
    source = sites.map_source_in_active_site(root, map_name)
    return robot_profile.merged_params_for_source('amcl.yaml', profile, source)


def test_explicit_localization_on_a_drawing_entry_gets_the_drawing_overlay(
        real_config, tmp_path):
    root = _site_root(tmp_path, version=2, active_map='plan',
                      maps={'plan': {'source': 'drawing'}, 'scanned': {}})
    params = _amcl(_explicit_localization_params(root, 'plan'))
    assert params['laser_model_type'] == 'likelihood_field_prob'


def test_explicit_localization_on_a_scan_entry_is_unchanged(real_config, tmp_path):
    root = _site_root(tmp_path, version=2, active_map='plan',
                      maps={'plan': {'source': 'drawing'}, 'scanned': {}})
    assert _explicit_localization_params(root, 'scanned') == str(CONFIG / 'amcl.yaml')


def test_explicit_localization_on_an_unknown_map_is_scan(real_config, tmp_path):
    root = _site_root(tmp_path, version=2, maps={'plan': {'source': 'drawing'}})
    assert _explicit_localization_params(root, 'ghost') == str(CONFIG / 'amcl.yaml')


@pytest.mark.parametrize('content', ['{not json', '[]', ''])
def test_explicit_localization_with_an_unreadable_site_is_scan(
        real_config, tmp_path, content):
    root = _site_root(tmp_path, content=content)
    assert _explicit_localization_params(root, 'plan') == str(CONFIG / 'amcl.yaml')


def test_explicit_localization_with_no_active_site_is_scan(real_config, tmp_path):
    assert _explicit_localization_params(str(tmp_path), 'plan') == str(
        CONFIG / 'amcl.yaml')


def test_explicit_localization_drawing_with_tight_tunnel_keeps_both_layers(
        stacked_config, tmp_path):
    root = _site_root(tmp_path, version=2, maps={'plan': {'source': 'drawing'}})
    params = _amcl(_explicit_localization_params(root, 'plan', 'tight_tunnel'))
    assert params['update_min_d'] == 0.02
    assert params['laser_max_range'] == 10.0


def _launch_setup_orelse_calls():
    func = _function('_launch_setup')
    for node in ast.walk(func):
        if (isinstance(node, ast.If) and isinstance(node.test, ast.Compare)
                and getattr(node.test.left, 'id', '') == 'mode'
                and [getattr(c, 'value', None) for c in node.test.comparators] == ['site']):
            return _called_names(ast.Module(body=node.orelse, type_ignores=[]))
    raise AssertionError('slam.launch.py lost its `if mode == "site"` branch')


def test_launch_explicit_localization_branch_reads_the_source_from_the_site():
    assert 'map_source_in_active_site' in _launch_setup_orelse_calls()


def test_launch_site_branch_still_reads_the_active_map_source():
    func = _function('_launch_setup')
    site_branch = next(
        n for n in ast.walk(func)
        if isinstance(n, ast.If) and isinstance(n.test, ast.Compare)
        and getattr(n.test.left, 'id', '') == 'mode'
        and [getattr(c, 'value', None) for c in n.test.comparators] == ['site'])
    assert 'active_map_source' in _called_names(
        ast.Module(body=site_branch.body, type_ignores=[]))
