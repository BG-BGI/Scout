"""core.tag_homes: which map a sighted tag's survey belongs to (U11) — the
active-pose rule, the transit home choice, and the read-only registry reader
(against real sqlite files, migrated and not)."""

import sqlite3

import pytest

from scout.core import tag_homes

P = {'x': 1.0, 'y': 2.0, 'yaw': 0.5}
Q = {'x': 3.0, 'y': 4.0, 'yaw': -1.0}
R = {'x': 5.0, 'y': 6.0, 'yaw': 0.0}
MAPS = {'floor1': {'source': 'scan'}, 'floor1_plan': {'source': 'drawing'},
        'floor2': {'source': 'scan'}, 'floor2_plan': {'source': 'drawing'},
        'floor3': {'source': 'scan'}}


# --- active_pose -----------------------------------------------------------------

def test_named_active_pose_wins():
    assert tag_homes.active_pose({'floor1': P, 'floor2': Q}, 'floor1') == ('floor1', P)


def test_no_pose_for_the_active_map_is_none_never_another_maps():
    assert tag_homes.active_pose({'floor2': Q}, 'floor1') is None
    assert tag_homes.active_pose({'floor2': Q}, None) is None


def test_legacy_row_applies_to_whatever_is_active_when_alone():
    assert tag_homes.active_pose({'': P}, 'floor1') == (None, P)
    assert tag_homes.active_pose({'': P}, 'floor9') == (None, P)
    assert tag_homes.active_pose({'': P}, None) == (None, P)


def test_legacy_row_is_ignored_once_the_tag_has_a_named_pose():
    assert tag_homes.active_pose({'': P, 'floor2': Q}, 'floor1') is None
    assert tag_homes.active_pose({'': P, 'floor2': Q}, 'floor2') == ('floor2', Q)


def test_empty_poses_is_none():
    assert tag_homes.active_pose({}, 'floor1') is None


# --- choose_home ------------------------------------------------------------------

def test_pose_on_the_active_map_is_never_a_transit_candidate():
    kind, home, pose, cands = tag_homes.choose_home(
        {'floor1': P, 'floor2': Q}, 'floor1', MAPS)
    assert (kind, home, pose, cands) == ('active', 'floor1', P, [])


def test_legacy_pose_relocalizes_on_the_active_map():
    assert tag_homes.choose_home({'': P}, 'floor1', MAPS) == ('active', None, P, [])


def test_one_same_source_foreign_map_is_the_transit_home():
    kind, home, pose, cands = tag_homes.choose_home(
        {'floor2': Q, 'floor1_plan': R}, 'floor1', MAPS)
    assert (kind, home, pose) == ('transit', 'floor2', Q)
    assert cands == ['floor1_plan', 'floor2']


def test_same_source_choice_follows_the_active_maps_source():
    kind, home, _, _ = tag_homes.choose_home(
        {'floor2': Q, 'floor1_plan': R}, 'floor2_plan', MAPS)
    assert (kind, home) == ('transit', 'floor1_plan')


def test_a_single_other_source_candidate_goes_to_the_transit_path_to_be_refused():
    kind, home, pose, cands = tag_homes.choose_home(
        {'floor1_plan': R}, 'floor1', MAPS)
    assert (kind, home, pose, cands) == ('transit', 'floor1_plan', R, ['floor1_plan'])


def test_two_same_source_candidates_are_refused_and_named():
    kind, home, pose, cands = tag_homes.choose_home(
        {'floor2': Q, 'floor3': R}, 'floor1', MAPS)
    assert (kind, home, pose, cands) == ('refused', None, None, ['floor2', 'floor3'])


def test_two_other_source_candidates_are_refused():
    kind, _, _, cands = tag_homes.choose_home(
        {'floor1_plan': Q, 'floor2_plan': R}, 'floor1', MAPS)
    assert kind == 'refused' and cands == ['floor1_plan', 'floor2_plan']


def test_no_poses_means_none():
    assert tag_homes.choose_home({}, 'floor1', MAPS) == ('none', None, None, [])


def test_no_active_map_with_only_named_poses_means_none():
    assert tag_homes.choose_home({'floor2': Q}, None, MAPS) == ('none', None, None, [])


def test_legacy_plus_named_elsewhere_does_not_relocalize_here():
    kind, home, _, _ = tag_homes.choose_home({'': P, 'floor2': Q}, 'floor1', MAPS)
    assert (kind, home) == ('transit', 'floor2')


@pytest.mark.parametrize('maps', [None, {}, [], 'x'])
def test_unknown_maps_read_as_scan(maps):
    kind, home, _, _ = tag_homes.choose_home({'floor2': Q}, 'floor1', maps)
    assert (kind, home) == ('transit', 'floor2')


# --- registry_poses against real sqlite files ---------------------------------------

LEGACY_DDL = """CREATE TABLE tags(family TEXT NOT NULL, tag_id INTEGER NOT NULL,
    name TEXT NOT NULL UNIQUE, role TEXT NOT NULL DEFAULT '',
    size_m REAL NOT NULL DEFAULT 0.16, map_x REAL, map_y REAL, map_yaw REAL,
    last_seen TEXT{extra}, PRIMARY KEY(family, tag_id))"""


def _db(tmp_path, extra=', map_name TEXT'):
    path = tmp_path / 'tags.db'
    db = sqlite3.connect(path)
    db.execute(LEGACY_DDL.format(extra=extra))
    return db


def test_unmigrated_named_legacy_row(tmp_path):
    db = _db(tmp_path)
    db.execute("INSERT INTO tags VALUES('tag36h11',1,'a','',0.16,1.0,2.0,0.5,NULL,'floor1')")
    assert tag_homes.registry_poses(db, 'tag36h11', 1) == {'floor1': P}


def test_unmigrated_null_map_name_is_legacy(tmp_path):
    db = _db(tmp_path)
    db.execute("INSERT INTO tags VALUES('tag36h11',1,'a','',0.16,1.0,2.0,0.5,NULL,NULL)")
    assert tag_homes.registry_poses(db, 'tag36h11', 1) == {'': P}


def test_unmigrated_pre_adr0029_file_without_a_map_name_column(tmp_path):
    db = _db(tmp_path, extra='')
    db.execute("INSERT INTO tags(family, tag_id, name, map_x, map_y, map_yaw)"
               " VALUES('tag36h11',1,'a',1.0,2.0,0.5)")
    assert tag_homes.registry_poses(db, 'tag36h11', 1) == {'': P}


def test_unmigrated_unsurveyed_tag_has_no_poses(tmp_path):
    db = _db(tmp_path)
    db.execute("INSERT INTO tags(family, tag_id, name) VALUES('tag36h11',1,'a')")
    assert tag_homes.registry_poses(db, 'tag36h11', 1) == {}


def test_migrated_file_reads_tag_poses_not_the_legacy_columns(tmp_path):
    db = _db(tmp_path)
    db.execute("INSERT INTO tags VALUES('tag36h11',1,'a','',0.16,9.0,9.0,9.0,NULL,'old')")
    db.execute("""CREATE TABLE tag_poses(family TEXT, tag_id INTEGER, map_name TEXT,
        map_x REAL, map_y REAL, map_yaw REAL, surveyed_at TEXT)""")
    db.execute("INSERT INTO tag_poses VALUES('tag36h11',1,'floor1',1.0,2.0,0.5,NULL)")
    db.execute("INSERT INTO tag_poses VALUES('tag36h11',1,'floor1_plan',3.0,4.0,-1.0,NULL)")
    assert tag_homes.registry_poses(db, 'tag36h11', 1) == {
        'floor1': P, 'floor1_plan': Q}


@pytest.mark.parametrize('x,y,yaw', [
    ('abc', 2.0, 0.5), (1.0, None, 0.5), (1.0, 2.0, float('nan')),
    (float('inf'), 2.0, 0.5), (b'x', 2.0, 0.5)])
def test_unusable_coordinates_are_skipped(tmp_path, x, y, yaw):
    db = _db(tmp_path)
    db.execute("INSERT INTO tags(family, tag_id, name) VALUES('tag36h11',1,'a')")
    db.execute("""CREATE TABLE tag_poses(family TEXT, tag_id INTEGER, map_name TEXT,
        map_x, map_y, map_yaw, surveyed_at TEXT)""")
    db.execute("INSERT INTO tag_poses VALUES('tag36h11',1,'floor1',?,?,?,NULL)",
               (x, y, yaw))
    db.execute("INSERT INTO tag_poses VALUES('tag36h11',1,'floor2',1.0,2.0,0.5,NULL)")
    assert tag_homes.registry_poses(db, 'tag36h11', 1) == {'floor2': P}


def test_non_str_map_name_is_skipped(tmp_path):
    db = _db(tmp_path)
    db.execute("""CREATE TABLE tag_poses(family TEXT, tag_id INTEGER, map_name,
        map_x REAL, map_y REAL, map_yaw REAL, surveyed_at TEXT)""")
    db.execute("INSERT INTO tag_poses VALUES('tag36h11',1,7,1.0,2.0,0.5,NULL)")
    assert tag_homes.registry_poses(db, 'tag36h11', 1) == {}


def test_half_migrated_table_raises_sqlite_error_for_the_caller_to_catch(tmp_path):
    db = _db(tmp_path)
    db.execute('CREATE TABLE tag_poses(family TEXT)')
    with pytest.raises(sqlite3.Error):
        tag_homes.registry_poses(db, 'tag36h11', 1)
