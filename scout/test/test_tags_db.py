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
    row = tagdb.lookup('tag36h11', 1, active='floor2')
    assert (row['map_x'], row['map_name']) == (1.0, 'floor2')


def test_poseless_sighting_keeps_map(tagdb):
    # A glimpse from the wrong floor (no solved pose) must not re-home the tag.
    tagdb.upsert('t', 1, 'tag36h11', '', 0.16)
    tagdb.record_sighting('tag36h11', 1, (1.0, 2.0, 0.5), map_name='floor2')
    tagdb.record_sighting('tag36h11', 1, None, map_name='floor1')
    row = tagdb.lookup('tag36h11', 1, active='floor2')
    assert (row['map_x'], row['map_name']) == (1.0, 'floor2')
    assert tagdb.lookup('tag36h11', 1, active='floor1')['map_x'] is None


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


# =============================================================================
# U11: poses are per (family, tag_id, map_name). Everything below uses tmp files
# only — never a real sites/*/maps/tags.db, never the robot.
# =============================================================================

import importlib.util as _ilu  # noqa: E402
import threading  # noqa: E402

from scout.core import tag_homes  # noqa: E402

TAGPOSE_PY = TAGS_PY.parents[1] / 'fleet-status' / 'tagpose.py'

LEGACY_COLS = """family TEXT NOT NULL, tag_id INTEGER NOT NULL,
    name TEXT NOT NULL UNIQUE, role TEXT NOT NULL DEFAULT '',
    size_m REAL NOT NULL DEFAULT 0.16, map_x REAL, map_y REAL, map_yaw REAL,
    last_seen TEXT"""


@pytest.fixture
def fleet():
    spec = _ilu.spec_from_file_location('fleet_tagpose_u11', TAGPOSE_PY)
    mod = _ilu.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _legacy_db(path, with_map_name=True, rows=()):
    """A pre-U11 tags.db: `with_map_name=False` is the pre-ADR-0029 shape."""
    db = sqlite3.connect(path)
    extra = ', map_name TEXT' if with_map_name else ''
    db.execute(f'CREATE TABLE tags({LEGACY_COLS}{extra}, PRIMARY KEY(family, tag_id))')
    for r in rows:
        db.execute(
            'INSERT INTO tags(family, tag_id, name, role, map_x, map_y, map_yaw, '
            'last_seen' + (', map_name' if with_map_name else '') + ') VALUES('
            + ','.join('?' * (9 if with_map_name else 8)) + ')', r)
    db.commit()
    db.close()


ROWS_29 = [
    ('tag36h11', 0, 'doghouse', 'home', 1.25, -2.5, 0.125, 't0', 'floor1'),
    ('tag36h11', 1, 'elev', '', 7.0, 8.0, -3.0, 't1', None),          # legacy NULL
    ('tag36h11', 2, 'unsurveyed', '', None, None, None, 't2', None),
    ('tag36h11', 3, 'stray', '', 4.0, 4.0, 1.0, 't3', 'floor2'),
]


def _tag_poses(path):
    db = sqlite3.connect(path)
    try:
        return sorted(db.execute(
            'SELECT family, tag_id, map_name, map_x, map_y, map_yaw FROM tag_poses'))
    finally:
        db.close()


def _legacy_cols(path):
    db = sqlite3.connect(path)
    try:
        return db.execute(
            'SELECT family, tag_id, map_x, map_y, map_yaw, map_name FROM tags '
            'ORDER BY tag_id').fetchall()
    finally:
        db.close()


def _version(path):
    db = sqlite3.connect(path)
    try:
        return db.execute('PRAGMA user_version').fetchone()[0]
    finally:
        db.close()


def test_adr0029_db_migrates_with_poses_identical_and_legacy_columns_kept(tagdb):
    _legacy_db(tagdb.DB_PATH, True, ROWS_29)
    before = _legacy_cols(tagdb.DB_PATH)
    tagdb.all_tags()
    assert _version(tagdb.DB_PATH) == tagdb.SCHEMA_VERSION == 3
    assert _tag_poses(tagdb.DB_PATH) == [
        ('tag36h11', 0, 'floor1', 1.25, -2.5, 0.125),
        ('tag36h11', 1, '', 7.0, 8.0, -3.0),            # NULL map -> LEGACY, not guessed
        ('tag36h11', 3, 'floor2', 4.0, 4.0, 1.0)]
    assert _legacy_cols(tagdb.DB_PATH) == before        # untouched, still present


def test_pre_adr0029_db_gains_map_name_and_legacy_poses(tagdb):
    _legacy_db(tagdb.DB_PATH, False, [
        ('tag36h11', 5, 'old', '', 2.0, 3.0, 0.25, 't')])
    tagdb.all_tags()
    cols = {r[1] for r in sqlite3.connect(tagdb.DB_PATH).execute('PRAGMA table_info(tags)')}
    assert {'map_x', 'map_y', 'map_yaw', 'map_name'} <= cols
    assert _tag_poses(tagdb.DB_PATH) == [('tag36h11', 5, '', 2.0, 3.0, 0.25)]


def test_empty_and_missing_db_migrate_to_an_empty_registry(tagdb, tmp_path):
    assert tagdb.all_tags() == []                      # file did not exist
    assert _version(tagdb.DB_PATH) == 3
    other = tmp_path / 'zero.db'
    other.write_bytes(b'')
    tagdb.DB_PATH = str(other)
    assert tagdb.all_tags() == []
    assert _tag_poses(str(other)) == []


def test_migration_is_idempotent_and_never_overwrites_later_edits(tagdb):
    _legacy_db(tagdb.DB_PATH, True, ROWS_29)
    tagdb.all_tags()
    db = sqlite3.connect(tagdb.DB_PATH)
    db.execute("UPDATE tag_poses SET map_x=99.0 WHERE map_name='floor1'")
    db.execute('PRAGMA user_version = 0')              # force a full re-run
    db.commit()
    db.close()
    for _ in range(3):
        tagdb.all_tags()
    assert _version(tagdb.DB_PATH) == 3
    assert ('tag36h11', 0, 'floor1', 99.0, -2.5, 0.125) in _tag_poses(tagdb.DB_PATH)
    assert len(_tag_poses(tagdb.DB_PATH)) == 3


def test_unusable_legacy_values_are_not_copied(tagdb):
    _legacy_db(tagdb.DB_PATH, True, [
        ('tag36h11', 0, 'a', '', 'abc', 1.0, 1.0, 't', 'floor1'),
        ('tag36h11', 1, 'b', '', 1.0, None, 1.0, 't', 'floor1')])
    tagdb.all_tags()
    assert _tag_poses(tagdb.DB_PATH) == []


def test_two_connections_racing_the_migration_both_succeed_and_one_migrates(tagdb):
    _legacy_db(tagdb.DB_PATH, True, ROWS_29)
    barrier = threading.Barrier(4)
    errors = []

    def go():
        try:
            db = sqlite3.connect(tagdb.DB_PATH, timeout=10, isolation_level=None)
            barrier.wait()
            tagdb.migrate(db)
            db.close()
        except Exception as e:  # noqa: BLE001 — collected and asserted empty
            errors.append(e)

    threads = [threading.Thread(target=go) for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert errors == []
    assert _version(tagdb.DB_PATH) == 3
    assert len(_tag_poses(tagdb.DB_PATH)) == 3


def test_loser_of_the_race_rechecks_the_version_inside_the_lock(tagdb):
    _legacy_db(tagdb.DB_PATH, True, ROWS_29)
    tagdb.all_tags()                                   # the winner already migrated
    db = sqlite3.connect(tagdb.DB_PATH, timeout=10, isolation_level=None)
    real_execute, state = db.execute, {'first': True}

    class Stale:
        def __getattr__(self, name):
            return getattr(db, name)

        def execute(self, sql, *args):
            if sql == 'PRAGMA user_version' and state['first']:
                state['first'] = False
                return real_execute('SELECT 0')        # the loser's stale first look
            return real_execute(sql, *args)

    before = _tag_poses(tagdb.DB_PATH)
    tagdb.migrate(Stale())
    assert _tag_poses(tagdb.DB_PATH) == before and not db.in_transaction
    db.close()


def test_failed_migration_rolls_back_whole(tagdb, monkeypatch):
    _legacy_db(tagdb.DB_PATH, True, ROWS_29)
    monkeypatch.setattr(tagdb, 'SCHEMA_STATEMENTS',
                        tagdb.SCHEMA_STATEMENTS[:2] + ('NOT SQL',))
    with pytest.raises(sqlite3.Error):
        tagdb.all_tags()
    db = sqlite3.connect(tagdb.DB_PATH)
    tables = {r[0] for r in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    assert 'tag_poses' not in tables and _version(tagdb.DB_PATH) == 0


# --- per-map behaviour ------------------------------------------------------------

def _two_maps(tagdb):
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    assert tagdb.record_sighting('tag36h11', 0, (1.0, 2.0, 0.5), map_name='floor1')
    assert tagdb.record_sighting('tag36h11', 0, (10.0, 20.0, -0.5), map_name='floor1_plan')


def test_same_tag_surveyed_on_two_maps_each_lookup_returns_its_own_pose(tagdb):
    _two_maps(tagdb)
    a = tagdb.lookup('tag36h11', 0, active='floor1')
    b = tagdb.lookup('tag36h11', 0, active='floor1_plan')
    assert (a['map_x'], a['map_y'], a['map_yaw'], a['map_name']) == (1.0, 2.0, 0.5, 'floor1')
    assert (b['map_x'], b['map_y'], b['map_yaw'], b['map_name']) == (
        10.0, 20.0, -0.5, 'floor1_plan')
    assert set(a['poses']) == {'floor1', 'floor1_plan'}
    assert a['poses']['floor1_plan']['x'] == 10.0 and a['poses']['floor1']['surveyed_at']


def test_unsurveyed_active_map_reads_null_never_another_maps_pose(tagdb):
    _two_maps(tagdb)
    c = tagdb.lookup('tag36h11', 0, active='floor2')
    assert (c['map_x'], c['map_y'], c['map_yaw'], c['map_name']) == (None,) * 4
    assert set(c['poses']) == {'floor1', 'floor1_plan'}
    n = tagdb.lookup('tag36h11', 0, active=None)
    assert n['map_x'] is None


def test_listing_keeps_the_old_keys_and_adds_poses(tagdb):
    _two_maps(tagdb)
    row = tagdb.all_tags(active='floor1')[0]
    assert {'family', 'tag_id', 'name', 'role', 'size_m', 'map_x', 'map_y',
            'map_yaw', 'map_name', 'last_seen', 'poses'} <= set(row)
    assert set(row['poses']['floor1']) == {'x', 'y', 'yaw', 'surveyed_at'}


def test_active_map_comes_from_site_json_by_default(tagdb):
    _two_maps(tagdb)
    _write_site(tagdb, {'version': 2, 'active_map': 'floor1_plan'})
    assert tagdb.lookup('tag36h11', 0)['map_x'] == 10.0
    _write_site(tagdb, {'version': 2, 'active_map': 'floor1'})
    assert tagdb.all_tags()[0]['map_x'] == 1.0


@pytest.mark.parametrize('active', [['floor1'], {'a': 1}, 7])
def test_hostile_active_map_in_site_json_reads_as_no_active_map(tagdb, active):
    _write_site(tagdb, {'version': 2, 'active_map': active})
    assert tagdb.active_map_name() is None


def test_clear_default_clears_the_active_map_only(tagdb):
    _two_maps(tagdb)
    _write_site(tagdb, {'version': 2, 'active_map': 'floor1'})
    assert tagdb.clear_pose('doghouse') == 1
    assert tagdb.lookup('tag36h11', 0, active='floor1')['map_x'] is None
    assert tagdb.lookup('tag36h11', 0, active='floor1_plan')['map_x'] == 10.0


def test_clear_explicit_map_and_all_maps(tagdb):
    _two_maps(tagdb)
    assert tagdb.clear_pose('doghouse', map_name='floor1_plan') == 1
    assert tagdb.lookup('tag36h11', 0, active='floor1')['map_x'] == 1.0
    assert tagdb.record_sighting('tag36h11', 0, (3.0, 3.0, 0.0), map_name='floor1_plan')
    assert tagdb.clear_pose('doghouse', all_maps=True) == 2
    assert tagdb.lookup('tag36h11', 0, active='floor1')['poses'] == {}


def test_clear_unknown_tag_is_none_and_unsurveyed_is_zero(tagdb):
    assert tagdb.clear_pose('ghost') is None
    tagdb.upsert('t', 1, 'tag36h11', '', 0.16)
    _write_site(tagdb, {'version': 2, 'active_map': 'floor1'})
    assert tagdb.clear_pose('t') == 0


def test_clear_default_without_active_map_touches_only_a_legacy_pose(tagdb):
    _legacy_db(tagdb.DB_PATH, True, [
        ('tag36h11', 0, 'a', '', 1.0, 1.0, 0.0, 't', None)])
    assert tagdb.clear_pose('a') == 1                  # no site.json: LEGACY only
    tagdb.upsert('b', 1, 'tag36h11', '', 0.16)
    tagdb.record_sighting('tag36h11', 1, (1.0, 1.0, 0.0), map_name='floor1')
    assert tagdb.clear_pose('b') == 0                  # named pose is not guessed


def test_clearing_a_legacy_active_pose_removes_it(tagdb):
    _legacy_db(tagdb.DB_PATH, True, [
        ('tag36h11', 0, 'a', '', 1.0, 1.0, 0.0, 't', None)])
    _write_site(tagdb, {'version': 2, 'active_map': 'floor1'})
    assert tagdb.lookup('tag36h11', 0)['map_x'] == 1.0     # assume-active meaning kept
    assert tagdb.clear_pose('a') == 1
    assert tagdb.lookup('tag36h11', 0)['map_x'] is None


def test_clear_never_touches_the_frozen_legacy_columns(tagdb):
    _legacy_db(tagdb.DB_PATH, True, ROWS_29)
    before = _legacy_cols(tagdb.DB_PATH)
    _write_site(tagdb, {'version': 2, 'active_map': 'floor1'})
    tagdb.clear_pose('doghouse')
    tagdb.record_sighting('tag36h11', 2, (5.0, 5.0, 0.0), map_name='floor1')
    assert _legacy_cols(tagdb.DB_PATH) == before


def test_delete_removes_the_tag_and_all_poses(tagdb):
    _two_maps(tagdb)
    assert tagdb.delete('doghouse') is True
    assert _tag_poses(tagdb.DB_PATH) == [] and tagdb.all_tags() == []
    assert tagdb.delete('doghouse') is False


def test_an_older_image_deleting_a_tag_cannot_leave_poses_to_resurrect(tagdb):
    _two_maps(tagdb)
    db = sqlite3.connect(tagdb.DB_PATH)                # rollback image: plain DELETE
    db.execute("DELETE FROM tags WHERE name='doghouse'")
    db.commit()
    db.close()
    assert _tag_poses(tagdb.DB_PATH) == []
    tagdb.upsert('doghouse', 0, 'tag36h11', 'home', 0.16)
    assert tagdb.lookup('tag36h11', 0, active='floor1')['map_x'] is None


def test_sighting_does_not_replace_an_existing_survey(tagdb):
    tagdb.upsert('t', 1, 'tag36h11', '', 0.16)
    assert tagdb.record_sighting('tag36h11', 1, (1.0, 2.0, 0.5), map_name='floor1')
    assert not tagdb.record_sighting('tag36h11', 1, (9.0, 9.0, 0.0), map_name='floor1')
    assert tagdb.lookup('tag36h11', 1, active='floor1')['map_x'] == 1.0


def test_sighting_without_a_map_or_for_an_unknown_tag_stores_nothing(tagdb):
    tagdb.upsert('t', 1, 'tag36h11', '', 0.16)
    assert not tagdb.record_sighting('tag36h11', 1, (1.0, 2.0, 0.5), map_name=None)
    assert not tagdb.record_sighting('tag36h11', 1, (1.0, 2.0, 0.5), map_name='')
    assert not tagdb.record_sighting('tag36h11', 99, (1.0, 2.0, 0.5), map_name='floor1')
    assert _tag_poses(tagdb.DB_PATH) == []


def test_sighting_matches_the_registry_family_by_normalised_name(tagdb):
    tagdb.upsert('t', 1, 'tagStandard52h13', '', 0.16)
    assert tagdb.record_sighting('Standard52h13', 1, (1.0, 2.0, 0.5), map_name='floor1')


def test_legacy_pose_rule_is_assume_active_until_a_named_survey_exists(tagdb):
    _legacy_db(tagdb.DB_PATH, True, [
        ('tag36h11', 0, 'a', '', 1.0, 1.0, 0.0, 't', None)])
    for m in ('floor1', 'floor1_plan', 'anything'):
        assert tagdb.lookup('tag36h11', 0, active=m)['map_x'] == 1.0
    assert tagdb.lookup('tag36h11', 0, active='floor1')['map_name'] is None
    tagdb.record_sighting('tag36h11', 0, (5.0, 5.0, 0.0), map_name='floor1')
    assert tagdb.lookup('tag36h11', 0, active='floor1')['map_x'] == 5.0
    assert tagdb.lookup('tag36h11', 0, active='floor1_plan')['map_x'] is None


# --- read-only readers (tag_relocalizer / fleet-status) ---------------------------

def _ro(path):
    return sqlite3.connect(f'file:{path}?mode=ro', uri=True)


def test_read_only_reader_on_an_unmigrated_db_falls_back_to_the_legacy_columns(
        tagdb, fleet):
    _legacy_db(tagdb.DB_PATH, True, ROWS_29)
    db = _ro(tagdb.DB_PATH)
    assert tag_homes.registry_poses(db, 'tag36h11', 0) == {
        'floor1': {'x': 1.25, 'y': -2.5, 'yaw': 0.125}}
    assert tag_homes.registry_poses(db, 'tag36h11', 1) == {
        '': {'x': 7.0, 'y': 8.0, 'yaw': -3.0}}
    db.close()
    assert _version(tagdb.DB_PATH) == 0                 # reading never migrates
    rows = fleet.list_tags(tagdb.DB_PATH, 'floor1')
    assert {r['name']: r['map_x'] for r in rows} == {
        'doghouse': 1.25, 'elev': 7.0, 'unsurveyed': None, 'stray': None}
    assert _version(tagdb.DB_PATH) == 0


def test_read_only_reader_on_a_migrated_db_reads_per_map_poses(tagdb, fleet):
    _two_maps(tagdb)
    db = _ro(tagdb.DB_PATH)
    assert tag_homes.registry_poses(db, 'tag36h11', 0) == {
        'floor1': {'x': 1.0, 'y': 2.0, 'yaw': 0.5},
        'floor1_plan': {'x': 10.0, 'y': 20.0, 'yaw': -0.5}}
    db.close()
    assert fleet.list_tags(tagdb.DB_PATH, 'floor1_plan')[0]['map_x'] == 10.0


# --- the two images' copies cannot drift ------------------------------------------

def test_schema_copies_are_identical(tagdb, fleet):
    assert fleet.SCHEMA_VERSION == tagdb.SCHEMA_VERSION
    assert fleet.SCHEMA_STATEMENTS == tagdb.SCHEMA_STATEMENTS
    assert fleet.LEGACY == tagdb.LEGACY == tag_homes.LEGACY


def test_both_migrations_produce_identical_databases(tagdb, fleet, tmp_path):
    a, b = tmp_path / 'a.db', tmp_path / 'b.db'
    for path in (a, b):
        _legacy_db(path, True, ROWS_29)
    for path, mod in ((a, tagdb), (b, fleet)):
        db = sqlite3.connect(path, isolation_level=None)
        mod.migrate(db)
        db.close()

    def dump(path):
        db = sqlite3.connect(path)
        try:
            return list(db.iterdump()), db.execute('PRAGMA user_version').fetchone()
        finally:
            db.close()
    assert dump(a) == dump(b)


POSE_CASES = [
    ({}, 'floor1'), ({'floor1': 'A'}, 'floor1'), ({'floor2': 'A'}, 'floor1'),
    ({'': 'A'}, 'floor1'), ({'': 'A'}, None), ({'': 'A', 'floor2': 'B'}, 'floor1'),
    ({'': 'A', 'floor2': 'B'}, 'floor2'), ({'floor1': 'A', 'floor2': 'B'}, None),
]


@pytest.mark.parametrize('poses,active', POSE_CASES)
def test_active_pose_rule_is_the_same_in_all_three_places(
        tagdb, fleet, poses, active):
    want = tag_homes.active_pose(poses, active)
    assert tagdb.active_pose(poses, active) == want
    assert fleet.active_pose(poses, active) == want


def test_listings_agree_between_the_two_images(tagdb, fleet):
    _two_maps(tagdb)
    tagdb.upsert('other', 4, 'tag36h11', '', 0.2)
    tagdb.record_sighting('tag36h11', 4, (3.0, 3.0, 0.0), map_name='floor2')
    for active in ('floor1', 'floor1_plan', 'floor2', 'floor9', None):
        assert fleet.list_tags(tagdb.DB_PATH, active) == tagdb.all_tags(active)
