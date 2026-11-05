"""Safe map save / autosave recovery for the active site's maps dir.

Pure stdlib. The floors and `_autosave` naming duplicate scout.core.mapsafety
(ADR-0011: share the schema, not the code -- this image has no scout package).

Why: 2026-11-05 a Save Map wrote a near-empty graph (7.7 KB .posegraph, 102 B
.data) over the operator's map name and slam_toolbox then segfaulted loading it.
So the webui no longer serializes straight to <name>: it serializes to
`_pending`, then POST /api/map/commit validates and installs it here.
"""

import os
import re
import shutil
import time

MIN_POSEGRAPH_BYTES = 50_000
MIN_DATA_BYTES = 1_000
AUTOSAVE = "_autosave"
PENDING = "_pending"
QUARANTINE = "_quarantine"
NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{0,31}$")
PAIR = (".posegraph", ".data")
GRID = (".yaml", ".pgm")


def _p(maps_dir, base, ext):
    return os.path.join(maps_dir, base + ext)


def _size(path):
    try:
        return os.path.getsize(path)
    except OSError:
        return None


def pair_check(maps_dir, base):
    """(ok, reason, total_bytes)."""
    pg, dat = (_size(_p(maps_dir, base, e)) for e in PAIR)
    if pg is None:
        return False, f"{base}.posegraph missing", 0
    if dat is None:
        return False, f"{base}.data missing", pg
    if pg < MIN_POSEGRAPH_BYTES:
        return False, (f"{base}.posegraph is {pg} B (< {MIN_POSEGRAPH_BYTES} B: "
                       "near-empty graph)"), pg + dat
    if dat < MIN_DATA_BYTES:
        return False, (f"{base}.data is {dat} B (< {MIN_DATA_BYTES} B: "
                       "near-empty graph)"), pg + dat
    return True, "ok", pg + dat


def _archive_existing(maps_dir, name, tag):
    """Move an existing <name> pair (+grid) to _quarantine/ so an overwrite is
    recoverable. Returns the destination paths."""
    qdir = os.path.join(maps_dir, QUARANTINE)
    os.makedirs(qdir, exist_ok=True)
    ts = time.strftime("%Y%m%dT%H%M%S")
    moved = []
    for ext in PAIR + GRID:
        src = _p(maps_dir, name, ext)
        if os.path.exists(src):
            dst = os.path.join(qdir, f"{name}.{ts}.{tag}{ext}")
            shutil.move(src, dst)
            moved.append(dst)
    return moved


def _guard_overwrite(maps_dir, name, new_total, force):
    """None if the write may proceed, else (409, error)."""
    ok, reason, old_total = pair_check(maps_dir, name)
    if ok and new_total < old_total and not force:
        return 409, {"error": (
            f"refusing to overwrite '{name}' ({old_total} B) with a smaller graph "
            f"({new_total} B). Pass force:true to replace it (the old pair is "
            f"kept in {QUARANTINE}/)."), "needs_force": True}
    return None


def commit_pending(maps_dir, name, force=False):
    """Install the `_pending` serialize/save_map output as <name>."""
    if not NAME_RE.match(name or "") or name.startswith("_"):
        return 400, {"error": f"invalid map name '{name}'"}
    ok, reason, new_total = pair_check(maps_dir, PENDING)
    if not ok:
        for e in PAIR + GRID:
            try:
                os.remove(_p(maps_dir, PENDING, e))
            except OSError:
                pass
        return 422, {"error": (
            f"refusing to save: the live graph is trivially small ({reason}). "
            "slam_toolbox probably has not received /map or just restarted -- "
            f"nothing was written. An autosave may exist: GET /api/map/autosave.")}
    refusal = _guard_overwrite(maps_dir, name, new_total, force)
    if refusal:
        return refusal
    archived = _archive_existing(maps_dir, name, "replaced") \
        if os.path.exists(_p(maps_dir, name, ".posegraph")) else []
    for e in PAIR:
        os.replace(_p(maps_dir, PENDING, e), _p(maps_dir, name, e))
    grid = False
    if os.path.exists(_p(maps_dir, PENDING, ".yaml")) \
            and os.path.exists(_p(maps_dir, PENDING, ".pgm")):
        os.replace(_p(maps_dir, PENDING, ".pgm"), _p(maps_dir, name, ".pgm"))
        with open(_p(maps_dir, PENDING, ".yaml")) as f:
            text = f.read()
        # map_saver records the image path; point it at the renamed file.
        text = re.sub(r"(?m)^image:.*$", f"image: {name}.pgm", text)
        tmp = _p(maps_dir, name, ".yaml.tmp")
        with open(tmp, "w") as f:
            f.write(text)
        os.replace(tmp, _p(maps_dir, name, ".yaml"))
        os.remove(_p(maps_dir, PENDING, ".yaml"))
        grid = True
    return 200, {"ok": True, "name": name, "bytes": new_total, "grid": grid,
                 "archived_previous": archived}


def autosave_status(maps_dir, now=None):
    now = time.time() if now is None else now
    ok, reason, total = pair_check(maps_dir, AUTOSAVE)
    mt = _size(_p(maps_dir, AUTOSAVE, ".posegraph")) is not None
    mtime = os.path.getmtime(_p(maps_dir, AUTOSAVE, ".posegraph")) if mt else None
    return {"exists": mt, "valid": ok, "reason": reason, "size_bytes": total,
            "age_s": None if mtime is None else max(0.0, now - mtime)}


def recover_autosave(maps_dir, name, force=False):
    """Copy `_autosave` -> <name>. Refuses an invalid autosave, and an existing
    valid <name> unless force (then the old pair is archived, not deleted)."""
    if not NAME_RE.match(name or "") or name.startswith("_"):
        return 400, {"error": f"invalid map name '{name}'"}
    ok, reason, total = pair_check(maps_dir, AUTOSAVE)
    if not ok:
        return 422, {"error": f"no usable autosave: {reason}"}
    old_ok, _, _ = pair_check(maps_dir, name)
    if old_ok and not force:
        return 409, {"error": (
            f"'{name}' already exists and is valid; pass force:true to replace "
            f"it (the old pair is kept in {QUARANTINE}/)."), "needs_force": True}
    archived = _archive_existing(maps_dir, name, "replaced") \
        if os.path.exists(_p(maps_dir, name, ".posegraph")) \
        or os.path.exists(_p(maps_dir, name, ".data")) else []
    for e in PAIR:
        tmp = _p(maps_dir, name, e + ".tmp")
        shutil.copyfile(_p(maps_dir, AUTOSAVE, e), tmp)
        os.replace(tmp, _p(maps_dir, name, e))
    return 200, {"ok": True, "name": name, "bytes": total,
                 "copied_from": AUTOSAVE, "archived_previous": archived,
                 "note": "posegraph pair only; no grid (.yaml/.pgm) -- continue "
                         "on it and Save Map to produce one for localization."}
