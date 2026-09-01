"""tags.db registry (docker/scout-skills/tags.py, loaded by path — separate
container, shared schema per ADR-0011): the v2 map_name migration (ADR-0029),
map-stamped sightings, and the per-call active-map read."""

import importlib.util
import json
import sqlite3
import sys
from pathlib import Path

import pytest

TAGS_PY = Path(__file__).resolve().parents[2] / 'docker' / 'scout-skills' / 'tags.py'


@pytest.fixture
def tagdb(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('skills_tags', TAGS_PY)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['skills_tags'] = mod
    spec.loader.exec_module(mod)
    monkeypatch.setattr(mod, 'DB_PATH', str(tmp_path / 'tags.db'))
    monkeypatch.setattr(mod, 'SITE_JSON', str(tmp_path / 'site.json'))
    return mod


def test_fresh_db_has_map_name(tagdb):
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    row = tagdb.all_tags()[0]
    assert row['map_name'] is None


def test_legacy_db_gains_map_name_idempotently(tagdb):
    # A pre-ADR-0029 db (no map_name column) migrates on _connect, and the
    # guard is rerun-safe.
    db = sqlite3.connect(tagdb.DB_PATH)
    db.execute(
        """CREATE TABLE tags(
             family TEXT NOT NULL, tag_id INTEGER NOT NULL,
             name TEXT NOT NULL UNIQUE, role TEXT NOT NULL DEFAULT '',
             size_m REAL NOT NULL DEFAULT 0.16,
             map_x REAL, map_y REAL, map_yaw REAL, last_seen TEXT,
             PRIMARY KEY(family, tag_id))"""
    )
    db.execute("INSERT INTO tags(family, tag_id, name) VALUES('tag36h11', 3, 'old')")
    db.commit()
    db.close()
    for _ in range(2):
        rows = tagdb.all_tags()
    assert rows[0]['map_name'] is None


def test_sighting_with_pose_stamps_map(tagdb):
    tagdb.upsert('t', 1, 'tag36h11', '', 0.16)
    tagdb.record_sighting('tag36h11', 1, (1.0, 2.0, 0.5), map_name='floor2')
    row = tagdb.lookup('tag36h11', 1)
    assert (row['map_x'], row['map_name']) == (1.0, 'floor2')


def test_poseless_sighting_keeps_map(tagdb):
    # A glimpse from the wrong floor (no solved pose) must not re-home the tag.
    tagdb.upsert('t', 1, 'tag36h11', '', 0.16)
    tagdb.record_sighting('tag36h11', 1, (1.0, 2.0, 0.5), map_name='floor2')
    tagdb.record_sighting('tag36h11', 1, None, map_name='floor1')
    row = tagdb.lookup('tag36h11', 1)
    assert (row['map_x'], row['map_name']) == (1.0, 'floor2')


def _write_site(tagdb, data):
    with open(tagdb.SITE_JSON, 'w') as f:
        if isinstance(data, str):
            f.write(data)
        else:
            json.dump(data, f)


def test_active_map_name_v2(tagdb):
    _write_site(tagdb, {'version': 2, 'active_map': 'floor1',
                        'maps': {'floor1': {}}})
    assert tagdb.active_map_name() == 'floor1'


def test_active_map_name_v1(tagdb):
    _write_site(tagdb, {'version': 1, 'default_map': 'office'})
    assert tagdb.active_map_name() == 'office'


def test_active_map_name_tolerates_garbage(tagdb):
    assert tagdb.active_map_name() is None       # missing file
    _write_site(tagdb, 'not json {')
    assert tagdb.active_map_name() is None       # unparseable
    _write_site(tagdb, {'version': 2, 'active_map': None})
    assert tagdb.active_map_name() is None       # no map yet
