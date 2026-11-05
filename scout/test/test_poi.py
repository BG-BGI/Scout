"""POI registry (docker/scout-skills/poi.py, duplicated byte-identically into
docker/fleet-status/poi.py — separate images)."""

import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2] / 'docker'
SKILLS_POI = ROOT / 'scout-skills' / 'poi.py'
FLEET_POI = ROOT / 'fleet-status' / 'poi.py'


@pytest.fixture
def poi(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('skills_poi', SKILLS_POI)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['skills_poi'] = mod
    spec.loader.exec_module(mod)
    monkeypatch.setattr(mod, 'DB_PATH', str(tmp_path / 'pois.db'))
    site = tmp_path / 'site.json'
    site.write_text(json.dumps({'active_map': 'floor1'}))
    monkeypatch.setattr(mod, 'SITE_JSON', str(site))
    return mod


def test_copies_identical():
    assert SKILLS_POI.read_bytes() == FLEET_POI.read_bytes()


def test_upsert_stamps_active_map_and_lists(poi):
    row = poi.upsert('charger', 1.23456, -2, 0.5, 'utility', 'by the door')
    assert row['map_name'] == 'floor1'
    assert (row['map_x'], row['map_y'], row['map_yaw']) == (1.235, -2.0, 0.5)
    assert [r['name'] for r in poi.all_pois()] == ['charger']


def test_upsert_same_name_moves(poi):
    poi.upsert('a', 0, 0)
    row = poi.upsert('a', 3, 4, category='x')
    assert (row['map_x'], row['map_y'], row['category']) == (3.0, 4.0, 'x')
    assert len(poi.all_pois()) == 1


def test_update_fields_partial_rename_move(poi):
    poi.upsert('a', 1, 1, category='c', notes='n')
    r = poi.update_fields('a', {'notes': 'm'})
    assert (r['notes'], r['category'], r['map_x']) == ('m', 'c', 1.0)
    r = poi.update_fields('a', {'new_name': 'b', 'x': 5})
    assert (r['name'], r['map_x'], r['map_y']) == ('b', 5.0, 1.0)
    assert poi.get('a') is None
    assert poi.update_fields('nope', {}) is None


def test_rename_collision_rejected(poi):
    poi.upsert('a', 0, 0)
    poi.upsert('b', 1, 1)
    with pytest.raises(poi.PoiError):
        poi.update_fields('a', {'new_name': 'b'})


def test_delete(poi):
    poi.upsert('a', 0, 0)
    assert poi.delete('a') is True
    assert poi.delete('a') is False


@pytest.mark.parametrize('args', [
    ('', 0, 0), ('a', 'x', 0), ('a', True, 0), ('a', float('nan'), 0),
    ('a', 1e6, 0), ('x' * 65, 0, 0),
])
def test_validation_rejects(poi, args):
    with pytest.raises(poi.PoiError):
        poi.upsert(*args)


def test_yaw_normalized(poi):
    assert abs(poi.upsert('a', 0, 0, 7.0)['map_yaw'] - (7.0 - 2 * 3.141592653589793)) < 1e-3


def test_explicit_map_name_wins(poi):
    assert poi.upsert('a', 0, 0, map_name='floor2')['map_name'] == 'floor2'
