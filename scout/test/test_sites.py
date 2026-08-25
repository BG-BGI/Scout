"""Location site store (ADR-0023): name contract, site.json defaults, and the
slam-mode resolution policy that mode:=site feeds into ADR-0003's table."""

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


# --- active symlink -----------------------------------------------------------

def test_active_site_name(tmp_path):
    assert sites.active_site_name(str(tmp_path)) is None
    (tmp_path / 'office').mkdir()
    os.symlink('office', tmp_path / 'active')
    assert sites.active_site_name(str(tmp_path)) == 'office'


# --- load_site ------------------------------------------------------------------

def test_load_site_defaults(tmp_path):
    site = sites.load_site(_write_site(tmp_path, version=1))
    assert site['default_map'] is None
    assert site['slam_mode'] == 'auto'
    assert site['map_start_pose'] == [0.0, 0.0, 0.0]


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


# --- resolve_slam: the auto policy ---------------------------------------------

def _site(**over):
    base = {'default_map': None, 'slam_mode': 'auto',
            'map_start_pose': [0.0, 0.0, 0.0]}
    base.update(over)
    return base


def test_auto_without_map_is_new(tmp_path):
    mode, _, _ = sites.resolve_slam(_site(), str(tmp_path))
    assert mode == 'new'


def test_auto_with_named_but_missing_map_is_new(tmp_path):
    # default_map set but never serialized (fresh site named before mapping).
    mode, _, _ = sites.resolve_slam(_site(default_map='office'), str(tmp_path))
    assert mode == 'new'


def test_auto_with_saved_map_is_continue(tmp_path):
    # continue, never localization: serialize_map silently no-ops there.
    (tmp_path / 'office.posegraph').touch()
    mode, map_name, _ = sites.resolve_slam(_site(default_map='office'),
                                           str(tmp_path))
    assert (mode, map_name) == ('continue', 'office')


def test_explicit_modes_pass_through(tmp_path):
    for want in ('new', 'localization', 'continue'):
        site = _site(slam_mode=want, default_map='office',
                     map_start_pose=[1.5, 0.0, 3.14])
        mode, _, pose = sites.resolve_slam(site, str(tmp_path))
        assert mode == want
    assert pose == [1.5, 0.0, 3.14]


def test_explicit_load_mode_without_map_raises(tmp_path):
    for want in ('localization', 'continue'):
        with pytest.raises(ValueError):
            sites.resolve_slam(_site(slam_mode=want), str(tmp_path))


def test_bad_start_pose_raises(tmp_path):
    with pytest.raises(ValueError):
        sites.resolve_slam(_site(map_start_pose=[1.0, 2.0]), str(tmp_path))
