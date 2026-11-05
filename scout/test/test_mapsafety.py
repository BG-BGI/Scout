"""Restart-proof SLAM mapping: scout.core.mapsafety rules + the fleet_status
mapsafe commit/recover REST logic (docker/fleet-status/mapsafe.py)."""
import importlib.util
import os
import pathlib

from scout.core import mapsafety as ms

_P = pathlib.Path(__file__).resolve().parents[2] / 'docker/fleet-status/mapsafe.py'
_spec = importlib.util.spec_from_file_location('fleet_mapsafe', _P)
fm = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fm)

BIG = (60_000, 5_000)
SMALL = (7_700, 102)  # the incident's near-empty pair


def pair(d, base, sizes=BIG, fill=b'x'):
    for ext, n in zip(ms.EXTS, sizes, strict=True):
        (d / (base + ext)).write_bytes(fill * n)


def test_floors_match_fleet_status():
    assert (ms.MIN_POSEGRAPH_BYTES, ms.MIN_DATA_BYTES) == (
        fm.MIN_POSEGRAPH_BYTES, fm.MIN_DATA_BYTES)
    assert ms.AUTOSAVE_BASE == fm.AUTOSAVE


def test_pair_status(tmp_path):
    assert not ms.pair_status(str(tmp_path), 'a')[0]
    pair(tmp_path, 'a', SMALL)
    ok, reason, _ = ms.pair_status(str(tmp_path), 'a')
    assert not ok and 'near-empty' in reason
    pair(tmp_path, 'a')
    assert ms.pair_status(str(tmp_path), 'a')[0]
    (tmp_path / 'a.data').unlink()
    assert 'missing' in ms.pair_status(str(tmp_path), 'a')[1]


def test_should_autosave_adaptive():
    assert ms.should_autosave(0, None, 0, 60, False)
    assert not ms.should_autosave(100, None, 0, 0, False)       # disabled
    assert not ms.should_autosave(100, None, 0, 60, True)       # in flight
    assert not ms.should_autosave(59, 0, 1.0, 60, False)
    assert ms.should_autosave(60, 0, 1.0, 60, False)
    # slow serialize (20 s > 60/4) stretches interval to 80 s
    assert not ms.should_autosave(79, 0, 20.0, 60, False)
    assert ms.should_autosave(80, 0, 20.0, 60, False)


def test_install_discards_trivial_and_keeps_existing(tmp_path):
    d = str(tmp_path)
    pair(tmp_path, ms.AUTOSAVE_BASE, BIG, b'g')
    pair(tmp_path, ms.TMP_BASE, SMALL)
    ok, reason = ms.install_autosave(d)
    assert not ok and 'not worth keeping' in reason
    assert (tmp_path / '_autosave.posegraph').read_bytes() == b'g' * 60_000
    assert not (tmp_path / '_autosave.tmp.posegraph').exists()


def test_install_rotates_generations(tmp_path):
    d = str(tmp_path)
    pair(tmp_path, ms.TMP_BASE, (60_000, 5_000), b'1')
    assert ms.install_autosave(d)[0]
    assert not (tmp_path / '_autosave.prev.posegraph').exists()
    pair(tmp_path, ms.TMP_BASE, (70_000, 5_000), b'2')
    assert ms.install_autosave(d)[0]
    assert (tmp_path / '_autosave.posegraph').read_bytes()[:1] == b'2'
    assert (tmp_path / '_autosave.prev.posegraph').read_bytes()[:1] == b'1'
    assert not (tmp_path / '_autosave.tmp.data').exists()


def test_install_small_new_session_does_not_evict_larger_prev(tmp_path):
    d = str(tmp_path)
    pair(tmp_path, ms.AUTOSAVE_BASE, (60_000, 5_000), b'c')
    pair(tmp_path, ms.PREV_BASE, (900_000, 50_000), b'p')
    pair(tmp_path, ms.TMP_BASE, (80_000, 5_000), b'n')
    assert ms.install_autosave(d)[0]
    assert (tmp_path / '_autosave.prev.posegraph').read_bytes()[:1] == b'p'
    assert (tmp_path / '_autosave.posegraph').read_bytes()[:1] == b'n'


def test_quarantine(tmp_path):
    pair(tmp_path, 'zone', SMALL)
    moved = ms.quarantine_pair(str(tmp_path), 'zone', ts='T1')
    assert len(moved) == 2 and all(os.path.exists(m) for m in moved)
    assert not (tmp_path / 'zone.posegraph').exists()
    assert moved[0].endswith('_quarantine/zone.T1.broken.posegraph')


def test_plan_valid_named_continues(tmp_path):
    pair(tmp_path, 'zone')
    p = ms.plan_slam_start(str(tmp_path), 'continue', 'zone')
    assert p['mode'] == 'continue' and p['map_base'].endswith('/zone')


def test_plan_broken_named_quarantined_then_new(tmp_path):
    pair(tmp_path, 'zone', SMALL)
    p = ms.plan_slam_start(str(tmp_path), 'continue', 'zone')
    assert p['mode'] == 'new' and p['quarantined']
    assert any('WARNING' in n for n in p['notes'])
    assert not (tmp_path / 'zone.posegraph').exists()


def test_plan_broken_named_falls_back_to_autosave(tmp_path):
    pair(tmp_path, 'zone', SMALL)
    pair(tmp_path, ms.AUTOSAVE_BASE)
    p = ms.plan_slam_start(str(tmp_path), 'continue', 'zone')
    assert p['mode'] == 'continue' and p['map_base'].endswith('_autosave')


def test_plan_resolved_new_adopts_autosave_unless_explicit(tmp_path):
    pair(tmp_path, ms.AUTOSAVE_BASE)
    assert ms.plan_slam_start(str(tmp_path), 'new', None)['mode'] == 'continue'
    assert ms.plan_slam_start(
        str(tmp_path), 'new', None, allow_autosave=False)['mode'] == 'new'


def test_plan_stale_autosave_ignored(tmp_path):
    pair(tmp_path, ms.AUTOSAVE_BASE)
    far = os.path.getmtime(tmp_path / '_autosave.posegraph') + 10 * 3600
    p = ms.plan_slam_start(str(tmp_path), 'new', None,
                           autosave_max_age_s=3600, now=far)
    assert p['mode'] == 'new'


def test_plan_missing_continue_raises_without_autosave(tmp_path):
    try:
        ms.plan_slam_start(str(tmp_path), 'continue', 'zone')
    except ValueError as e:
        assert 'zone' in str(e)
    else:
        raise AssertionError('expected ValueError')


# --- fleet_status REST logic --------------------------------------------------

def test_commit_refuses_trivial_graph(tmp_path):
    pair(tmp_path, '_pending', SMALL)
    code, out = fm.commit_pending(str(tmp_path), 'zone')
    assert code == 422 and 'trivially small' in out['error']
    assert not (tmp_path / 'zone.posegraph').exists()
    assert not (tmp_path / '_pending.posegraph').exists()


def test_commit_refuses_shrink_without_force_and_archives_with(tmp_path):
    pair(tmp_path, 'zone', (900_000, 90_000), b'o')
    pair(tmp_path, '_pending', BIG, b'n')
    code, out = fm.commit_pending(str(tmp_path), 'zone')
    assert code == 409 and out['needs_force']
    assert (tmp_path / 'zone.posegraph').read_bytes()[:1] == b'o'
    code, out = fm.commit_pending(str(tmp_path), 'zone', force=True)
    assert code == 200 and out['archived_previous']
    assert (tmp_path / 'zone.posegraph').read_bytes()[:1] == b'n'


def test_commit_grows_and_renames_grid(tmp_path):
    pair(tmp_path, 'zone', BIG, b'o')
    pair(tmp_path, '_pending', (90_000, 9_000), b'n')
    (tmp_path / '_pending.pgm').write_bytes(b'P5')
    (tmp_path / '_pending.yaml').write_text('image: /x/_pending.pgm\nresolution: 0.05\n')
    code, out = fm.commit_pending(str(tmp_path), 'zone')
    assert code == 200 and out['grid']
    assert 'image: zone.pgm' in (tmp_path / 'zone.yaml').read_text()
    assert (tmp_path / 'zone.pgm').exists()


def test_commit_rejects_bad_names(tmp_path):
    assert fm.commit_pending(str(tmp_path), '_autosave')[0] == 400
    assert fm.commit_pending(str(tmp_path), '../x')[0] == 400


def test_recover_autosave(tmp_path):
    d = str(tmp_path)
    assert fm.recover_autosave(d, 'zone')[0] == 422
    pair(tmp_path, fm.AUTOSAVE, BIG, b'a')
    st = fm.autosave_status(d)
    assert st['exists'] and st['valid'] and st['size_bytes'] == 65_000
    code, out = fm.recover_autosave(d, 'zone')
    assert code == 200 and (tmp_path / 'zone.data').read_bytes() == b'a' * 5_000
    assert (tmp_path / '_autosave.posegraph').exists()  # autosave untouched
    code, out = fm.recover_autosave(d, 'zone')
    assert code == 409
    code, out = fm.recover_autosave(d, 'zone', force=True)
    assert code == 200 and out['archived_previous']


def test_autosave_status_absent(tmp_path):
    st = fm.autosave_status(str(tmp_path))
    assert not st['exists'] and st['age_s'] is None
