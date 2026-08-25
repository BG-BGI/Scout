"""Profile overlays (ADR-0010): deep_merge semantics, a typo guard (every
overlay key must exist in its base, so a misspelled key can't silently create a
new param), and sentinel merged values. Runs off-ROS — loads the YAML directly
and imports the pure deep_merge from robot_profile."""

import pathlib

import yaml

from scout.robot_profile import deep_merge

CONFIG = pathlib.Path(__file__).resolve().parent.parent / 'config'
OVERLAYS = CONFIG / 'overlays' / 'tight_tunnel'

# Keys an overlay may introduce that are absent from the base (reviewed).
ADDITIVE = {
    ('bt_navigator', 'ros__parameters', 'default_nav_to_pose_bt_xml'),
}


def _load(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


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


def test_every_overlay_key_exists_in_base():
    for overlay_file in sorted(OVERLAYS.glob('*.yaml')):
        base = _load(CONFIG / overlay_file.name)
        overlay = _load(overlay_file)
        for path in _leaf_paths(overlay):
            assert _has_path(base, path) or path in ADDITIVE, (
                '%s introduces key %s absent from %s — typo, or add it to '
                'ADDITIVE if deliberate' % (overlay_file.name, path, overlay_file.name))


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
