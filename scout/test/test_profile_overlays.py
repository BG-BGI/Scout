"""Profile overlays (ADR-0010): deep_merge semantics, a typo guard (every
overlay key must exist in its base, so a misspelled key can't silently create a
new param), and sentinel merged values. Runs off-ROS — loads the YAML directly
and imports the pure deep_merge from robot_profile."""

import pathlib
import tempfile

import pytest
import yaml

from scout import robot_profile
from scout.robot_profile import deep_merge

CONFIG = pathlib.Path(__file__).resolve().parent.parent / 'config'
OVERLAYS = CONFIG / 'overlays' / 'tight_tunnel'
DRAWING = CONFIG / 'source_overlays' / 'drawing'

OPERATOR_OVERLAY_DIRS = sorted(p for p in (CONFIG / 'overlays').iterdir()
                               if p.is_dir())
SOURCE_OVERLAY_DIRS = sorted(p for p in (CONFIG / 'source_overlays').iterdir()
                             if p.is_dir())

# Keys an overlay may introduce that are absent from the base (reviewed).
ADDITIVE = {
    ('bt_navigator', 'ros__parameters', 'default_nav_to_pose_bt_xml'),
    # Real nav2_amcl param, absent from scout/config/amcl.yaml; the drawing
    # overlay pins it at the upstream default (2.0).
    ('amcl', 'ros__parameters', 'laser_likelihood_max_dist'),
}


def _load(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


def _dump(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w') as f:
        yaml.safe_dump(data, f)
    return path


@pytest.fixture
def real_config(monkeypatch, tmp_path):
    """Point the resolvers at the repo's config dir (off-ROS: no ament_index)
    and send merged output to tmp_path instead of /tmp/scout_profile."""
    monkeypatch.setattr(robot_profile, '_BIND_DIR', str(CONFIG))
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    return CONFIG


def test_deep_merge_semantics():
    base = {'a': 1, 'b': {'x': 1, 'y': 2}, 'lst': [1, 2, 3], 'gone': 9}
    overlay = {'b': {'y': 20, 'z': 30}, 'lst': [9], 'gone': None, 'c': 5}
    assert deep_merge(base, overlay) == {
        'a': 1, 'b': {'x': 1, 'y': 20, 'z': 30}, 'lst': [9], 'c': 5}
    assert deep_merge(base, {}) == base   # empty overlay is a no-op copy
    base_after = {'a': 1, 'b': {'x': 1, 'y': 2}, 'lst': [1, 2, 3], 'gone': 9}
    assert base == base_after             # inputs not mutated


def _leaf_paths(d, prefix=()):
    for k, v in d.items():
        if isinstance(v, dict):
            yield from _leaf_paths(v, prefix + (k,))
        else:
            yield prefix + (k,)


def _has_path(d, path):
    for k in path:
        if not isinstance(d, dict) or k not in d:
            return False
        d = d[k]
    return True


@pytest.mark.parametrize(
    'overlay_dir', OPERATOR_OVERLAY_DIRS + SOURCE_OVERLAY_DIRS,
    ids=lambda p: '%s/%s' % (p.parent.name, p.name))
def test_every_overlay_key_exists_in_base(overlay_dir):
    overlay_files = sorted(overlay_dir.glob('*.yaml'))
    assert overlay_files, '%s has no overlay files' % overlay_dir
    for overlay_file in overlay_files:
        base = _load(CONFIG / overlay_file.name)
        overlay = _load(overlay_file)
        for path in _leaf_paths(overlay):
            assert _has_path(base, path) or path in ADDITIVE, (
                '%s introduces key %s absent from %s — typo, or add it to '
                'ADDITIVE if deliberate' % (overlay_file, path, overlay_file.name))


def test_tight_tunnel_sentinels():
    nav2 = deep_merge(_load(CONFIG / 'nav2.yaml'), _load(OVERLAYS / 'nav2.yaml'))
    fp = nav2['controller_server']['ros__parameters']['FollowPath']
    assert fp['max_vel_x'] == 0.35 and fp['min_vel_x'] == -0.15
    local = nav2['local_costmap']['local_costmap']['ros__parameters']
    assert local['plugins'] == ['obstacle_layer', 'inflation_layer']   # stvl_layer dropped
    assert 'stvl_layer' in local         # ...but the block is still present (inert)
    assert local['inflation_layer']['inflation_radius'] == 0.17

    slam = deep_merge(_load(CONFIG / 'slam.yaml'), _load(OVERLAYS / 'slam.yaml'))
    assert slam['slam_toolbox']['ros__parameters']['resolution'] == 0.025

    rs = deep_merge(_load(CONFIG / 'realsense.yaml'), _load(OVERLAYS / 'realsense.yaml'))
    assert rs['enable_depth'] is False
    assert 'json_file_path' not in rs     # null in the overlay deletes it


def test_default_profile_is_untouched_base():
    # merged_params('default') returns the base path unchanged; deep_merge with
    # an empty overlay is the in-memory equivalent used here.
    base = _load(CONFIG / 'nav2.yaml')
    assert deep_merge(base, {}) == base


def test_known_profiles_excludes_source_overlays(real_config):
    assert robot_profile.known_profiles() == ['default', 'tight_tunnel']
    assert 'drawing' not in robot_profile.known_profiles()


def test_source_overlay_dir_resolves_like_profile_overlays(real_config):
    assert robot_profile.source_overlay_dir('drawing') == str(DRAWING)
    assert robot_profile.profile_overlay_dir('tight_tunnel') == str(OVERLAYS)
    assert robot_profile.profile_overlay_dir('default') is None


@pytest.fixture
def synthetic(monkeypatch, tmp_path):
    """Base x.yaml (a=1,b=1,c=1) plus overlay dirs ov1 (b=2,c=2), ov2 (c=3)."""
    monkeypatch.setattr(robot_profile, '_BIND_DIR', str(tmp_path / 'config'))
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    _dump(tmp_path / 'config' / 'x.yaml', {'a': 1, 'b': 1, 'c': 1})
    _dump(tmp_path / 'ov1' / 'x.yaml', {'b': 2, 'c': 2})
    _dump(tmp_path / 'ov2' / 'x.yaml', {'c': 3})
    return tmp_path


def test_chain_later_overlay_wins(synthetic):
    ov1, ov2 = str(synthetic / 'ov1'), str(synthetic / 'ov2')
    forward = robot_profile.merged_params_chain('x.yaml', [ov1, ov2])
    reverse = robot_profile.merged_params_chain('x.yaml', [ov2, ov1])
    assert _load(forward) == {'a': 1, 'b': 2, 'c': 3}
    assert _load(reverse) == {'a': 1, 'b': 2, 'c': 2}
    assert forward != reverse


def test_chain_missing_layer_file_is_noop(synthetic):
    ov1 = str(synthetic / 'ov1')
    empty = synthetic / 'empty'
    empty.mkdir()
    only_ov1 = robot_profile.merged_params_chain('x.yaml', [ov1])
    assert _load(robot_profile.merged_params_chain(
        'x.yaml', [ov1, str(empty)])) == _load(only_ov1)
    assert _load(robot_profile.merged_params_chain(
        'x.yaml', [str(empty), ov1])) == _load(only_ov1)
    assert _load(robot_profile.merged_params_chain(
        'x.yaml', [ov1, str(synthetic / 'no_such_dir')])) == _load(only_ov1)


def test_chain_without_contributing_layer_returns_base_path(synthetic):
    base = str(synthetic / 'config' / 'x.yaml')
    empty = synthetic / 'empty'
    empty.mkdir()
    assert robot_profile.merged_params_chain('x.yaml', []) == base
    assert robot_profile.merged_params_chain('x.yaml', [None]) == base
    assert robot_profile.merged_params_chain('x.yaml', [str(empty)]) == base


def test_chain_does_not_mutate_inputs(synthetic):
    robot_profile.merged_params_chain(
        'x.yaml', [str(synthetic / 'ov1'), str(synthetic / 'ov2')])
    assert _load(synthetic / 'config' / 'x.yaml') == {'a': 1, 'b': 1, 'c': 1}
    assert _load(synthetic / 'ov1' / 'x.yaml') == {'b': 2, 'c': 2}


def test_drawing_amcl_overrides_and_preserves_base(real_config):
    base = _load(CONFIG / 'amcl.yaml')
    out = robot_profile.merged_params_chain(
        'amcl.yaml', [robot_profile.source_overlay_dir('drawing')])
    merged = _load(out)
    params = merged['amcl']['ros__parameters']
    assert params['laser_model_type'] == 'likelihood_field_prob'
    assert params['do_beamskip'] is True
    assert params['beam_skip_distance'] == 0.5
    assert params['beam_skip_threshold'] == 0.3
    assert params['beam_skip_error_threshold'] == 0.9
    assert params['max_beams'] == 90
    assert params['laser_max_range'] == 10.0
    assert params['z_hit'] == 0.6 and params['z_rand'] == 0.4
    assert params['sigma_hit'] == 0.15
    assert params['laser_likelihood_max_dist'] == 2.0
    assert params['recovery_alpha_slow'] == 0.0
    assert params['recovery_alpha_fast'] == 0.0

    overlay_keys = set(_load(DRAWING / 'amcl.yaml')['amcl']['ros__parameters'])
    base_params = base['amcl']['ros__parameters']
    untouched = {k: v for k, v in base_params.items() if k not in overlay_keys}
    assert untouched
    assert {k: params[k] for k in untouched} == untouched
    assert params['base_frame_id'] == 'base_link'
    assert params['min_particles'] == 1000
    assert {k: v for k, v in merged.items() if k != 'amcl'} == {
        k: v for k, v in base.items() if k != 'amcl'}


def test_scan_source_leaves_amcl_untouched(real_config):
    base = str(CONFIG / 'amcl.yaml')
    scan = robot_profile.source_overlay_dir('scan')
    assert robot_profile.merged_params_chain('amcl.yaml', [scan]) == base
    assert robot_profile.merged_params_chain('amcl.yaml', [None, scan]) == base
    assert _load(base)['amcl']['ros__parameters'][
        'laser_model_type'] == 'likelihood_field'


def test_chain_with_profile_only_matches_merged_params(real_config):
    profile_dir = robot_profile.profile_overlay_dir('tight_tunnel')
    for basename in ('nav2.yaml', 'slam.yaml', 'amcl.yaml'):
        legacy = robot_profile.merged_params(basename, 'tight_tunnel')
        chained = robot_profile.merged_params_chain(basename, [profile_dir])
        with open(legacy, 'rb') as f_legacy, open(chained, 'rb') as f_chain:
            assert f_legacy.read() == f_chain.read()
    default_dir = robot_profile.profile_overlay_dir('default')
    assert robot_profile.merged_params_chain(
        'nav2.yaml', [default_dir]) == robot_profile.merged_params(
            'nav2.yaml', 'default')
