"""Restart-proofing for slam_toolbox pose graphs. Pure: stdlib only.

Incident 2026-11-05: a ~10 min mapping session lived only in slam_toolbox's
memory; a SIGKILL restart lost it, a later Save Map wrote a near-empty graph
(7.7 KB .posegraph, 102 B .data), and the next start SEGFAULTED deserializing
it, crash-looping. Three defenses share the rules below:

  * map_autosave node  -- periodic serialize_map to `_autosave`, atomic,
                          two generations (`_autosave`, `_autosave.prev`)
  * slam.launch.py     -- validate/quarantine the named pair before `continue`
  * fleet_status       -- refuses near-empty saves (duplicated there, ADR-0011)

"Is this graph worth keeping" is judged by FILE SIZE of the serialized pair.
boost archives cannot be parsed here, and the live node count is not cheaply
exposed (graph_visualization markers are a full-graph republish -- exactly the
cost we are avoiding). Size tracks node count monotonically because every node
carries its scan; the floors below sit between the observed broken pair
(7.7 KB / 102 B) and a real session (the stationary session was already
11.8 MB). They are tunable params, not measured thresholds.
"""

import os
import shutil
import time

# Floors for "a pair slam_toolbox can safely load / is worth keeping".
MIN_POSEGRAPH_BYTES = 50_000
MIN_DATA_BYTES = 1_000
AUTOSAVE_BASE = '_autosave'
PREV_BASE = '_autosave.prev'
TMP_BASE = '_autosave.tmp'
QUARANTINE_DIR = '_quarantine'
EXTS = ('.posegraph', '.data')


def pair_paths(maps_dir, base):
    return [os.path.join(maps_dir, base + e) for e in EXTS]


def pair_status(maps_dir, base, min_posegraph=MIN_POSEGRAPH_BYTES,
                min_data=MIN_DATA_BYTES):
    """(ok, reason, size_bytes). ok = both files present and above the floors."""
    pg, dat = pair_paths(maps_dir, base)
    try:
        pg_size = os.path.getsize(pg)
    except OSError:
        return False, f'{base}.posegraph missing', 0
    try:
        dat_size = os.path.getsize(dat)
    except OSError:
        return False, f'{base}.data missing', pg_size
    total = pg_size + dat_size
    if pg_size < min_posegraph:
        return False, (f'{base}.posegraph is {pg_size} B '
                       f'(< {min_posegraph} B: near-empty graph)'), total
    if dat_size < min_data:
        return False, (f'{base}.data is {dat_size} B '
                       f'(< {min_data} B: near-empty graph)'), total
    return True, 'ok', total


def pair_info(maps_dir, base, now=None):
    """Read-only summary: {exists, valid, reason, size_bytes, age_s, mtime}."""
    ok, reason, size = pair_status(maps_dir, base)
    pg = pair_paths(maps_dir, base)[0]
    try:
        mtime = os.path.getmtime(pg)
    except OSError:
        return {'exists': False, 'valid': False, 'reason': reason,
                'size_bytes': 0, 'age_s': None, 'mtime': None}
    now = time.time() if now is None else now
    return {'exists': True, 'valid': ok, 'reason': reason,
            'size_bytes': size, 'age_s': max(0.0, now - mtime), 'mtime': mtime}


def next_autosave_due(last_start, last_duration, period_s):
    """Monotonic time at/after which the next autosave may start.

    Adaptive: a serialize that took longer than period/4 would eat a quarter of
    the period inside slam_toolbox, so the interval stretches to 4x the measured
    duration (duty cycle capped at one quarter)."""
    if last_start is None:
        return 0.0
    interval = max(float(period_s), 4.0 * (last_duration or 0.0))
    return last_start + interval


def should_autosave(now, last_start, last_duration, period_s, in_flight):
    if period_s <= 0 or in_flight:
        return False
    return now >= next_autosave_due(last_start, last_duration, period_s)


def _replace_pair(maps_dir, src_base, dst_base):
    for s, d in zip(pair_paths(maps_dir, src_base),
                    pair_paths(maps_dir, dst_base), strict=True):
        os.replace(s, d)  # atomic on one filesystem


def _remove_pair(maps_dir, base):
    for p in pair_paths(maps_dir, base):
        try:
            os.remove(p)
        except OSError:
            pass


def install_autosave(maps_dir, tmp_base=TMP_BASE, final_base=AUTOSAVE_BASE,
                     prev_base=PREV_BASE, min_posegraph=MIN_POSEGRAPH_BYTES,
                     min_data=MIN_DATA_BYTES):
    """Promote a freshly serialized tmp pair to `_autosave`.

    Returns (installed: bool, reason). The tmp pair is validated first; a
    trivially small one is discarded and the existing autosave is untouched.
    The current good autosave is demoted to `_autosave.prev` -- unless the
    existing prev is LARGER (a fresh small session must not evict the only copy
    of a big earlier one after a restart). Each file moves with os.replace, so a
    kill mid-way never leaves a torn file; between the two replaces of a pair
    there is a microsecond window where posegraph/data are different
    generations (pair_status would still pass) -- accepted, and the prev
    generation covers it."""
    ok, reason, tmp_size = pair_status(maps_dir, tmp_base, min_posegraph, min_data)
    if not ok:
        _remove_pair(maps_dir, tmp_base)
        return False, f'skipped, graph not worth keeping: {reason}'
    cur_ok, _, cur_size = pair_status(maps_dir, final_base, min_posegraph, min_data)
    if cur_ok:
        prev_ok, _, prev_size = pair_status(
            maps_dir, prev_base, min_posegraph, min_data)
        if not prev_ok or prev_size <= cur_size:
            _replace_pair(maps_dir, final_base, prev_base)
    _replace_pair(maps_dir, tmp_base, final_base)
    return True, f'autosaved {tmp_size} B'


def quarantine_pair(maps_dir, base, ts=None):
    """Move <base>.{posegraph,data} to _quarantine/<base>.<ts>.broken.<ext>.
    Returns the list of destination paths (only files that existed)."""
    ts = time.strftime('%Y%m%dT%H%M%S') if ts is None else ts
    qdir = os.path.join(maps_dir, QUARANTINE_DIR)
    os.makedirs(qdir, exist_ok=True)
    moved = []
    for ext in EXTS:
        src = os.path.join(maps_dir, base + ext)
        if os.path.exists(src):
            dst = os.path.join(qdir, f'{base}.{ts}.broken{ext}')
            shutil.move(src, dst)
            moved.append(dst)
    return moved


def plan_slam_start(maps_dir, mode, map_name, allow_autosave=True,
                    autosave_max_age_s=0.0, now=None, ts=None):
    """Crash-loop guard for a resolved slam mode (new | continue).

    Returns {'mode', 'map_base' (absolute basename or None), 'notes': [..],
    'quarantined': [..]}. Never raises for a bad pair -- the point is to start.
    Raises ValueError only for explicit `continue` of a map that is absent AND
    has no usable autosave (existing behavior: refuse, don't map into nothing).

    Rules:
      * new                      -> autosave if allow_autosave (callers pass False
                                    for an explicit operator `mode:=new`), else new.
      * continue, pair valid     -> continue <name>.
      * continue, pair present
        but invalid              -> quarantine pair, loud note, then autosave/new.
      * continue, pair absent    -> autosave/(ValueError if explicit continue).
      * autosave fallback        -> continue `_autosave` if it is a valid pair
                                    and (max_age == 0 or younger than max_age).
    """
    notes, quarantined = [], []
    named_missing = False
    if mode == 'continue' and map_name:
        ok, reason, _ = pair_status(maps_dir, map_name)
        if ok:
            return {'mode': 'continue',
                    'map_base': os.path.join(maps_dir, map_name),
                    'notes': notes, 'quarantined': quarantined}
        pg, dat = pair_paths(maps_dir, map_name)
        if os.path.exists(pg) or os.path.exists(dat):
            quarantined = quarantine_pair(maps_dir, map_name, ts)
            notes.append(
                f"WARNING: map '{map_name}' failed the sanity check ({reason}); "
                f'moved to {QUARANTINE_DIR}/ instead of crash-looping '
                f'slam_toolbox: {quarantined}')
        else:
            named_missing = True
    elif mode == 'continue':
        named_missing = True

    if allow_autosave:
        info = pair_info(maps_dir, AUTOSAVE_BASE, now)
        fresh = (not autosave_max_age_s or
                 (info['age_s'] is not None and info['age_s'] <= autosave_max_age_s))
        if info['valid'] and fresh:
            notes.append(
                f"WARNING: continuing from {AUTOSAVE_BASE} "
                f"({info['size_bytes']} B, {int(info['age_s'])} s old) -- "
                'recovered unsaved mapping session; Save Map to keep it')
            return {'mode': 'continue',
                    'map_base': os.path.join(maps_dir, AUTOSAVE_BASE),
                    'notes': notes, 'quarantined': quarantined}
        if info['valid'] and not fresh:
            notes.append(f'autosave exists but is older than '
                         f'{autosave_max_age_s:.0f} s; ignoring')
    if named_missing and mode == 'continue':
        raise ValueError(
            f"No saved map named '{map_name}' in {maps_dir} and no usable "
            f'{AUTOSAVE_BASE} to fall back on.')
    if quarantined:
        notes.append('starting a fresh `new` session')
    return {'mode': 'new', 'map_base': None, 'notes': notes,
            'quarantined': quarantined}
