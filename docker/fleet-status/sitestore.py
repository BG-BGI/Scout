"""site.json schema for fleet_status: map entries, normalizing, patch validation.

Pure stdlib, no docker/requests, so a module that registers map entries (a grid
installer) can import it without pulling server.py. The schema duplicates
scout.core.sites (ADR-0011: share the schema, not the code -- this image has no
scout package); test_sitestore.py pins the two copies together.

v2 (ADR-0029): a site holds multiple labeled maps; active_map is the one
slam/amcl runs on. v1 files (default_map + top-level map_start_pose) are
normalized on read and write-upgraded on the next write.

`source` is where a map's grid came from: `scan` (SLAM) or `drawing` (imported
floorplan, grid pair with no posegraph). An unknown value reads as `scan` and is
rejected on write.
"""

import json
import os
import re
import threading

SITE_NAME_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{0,31}$")
SLAM_MODES = ("auto", "new", "localization", "continue")
SOURCES = ("scan", "drawing")
SITE_LOCK = threading.Lock()

_MAP_DEFAULTS = {
    "label": "",
    "floor": None,
    "source": "scan",
    "map_start_pose": [0.0, 0.0, 0.0],
}


def norm_map_entry(name, entry):
    m = dict(_MAP_DEFAULTS)
    if isinstance(entry, dict):
        for key in m:
            if key in entry and entry[key] is not None:
                m[key] = entry[key]
    if not m["label"]:
        m["label"] = name
    if m["source"] not in SOURCES:
        m["source"] = _MAP_DEFAULTS["source"]
    return m


def normalize_site(data):
    """v1 or v2 site.json dict -> normalized v2 dict (mirror of
    scout.core.sites.load_site, minus the raise-on-invalid strictness --
    an HTTP status API shouldn't crash-loop on a bad file)."""
    site = {"version": 2, "display_name": "", "active_map": None,
            "slam_mode": "auto", "maps": {}}
    if not isinstance(data, dict):
        return site
    for key in ("display_name", "slam_mode"):
        if data.get(key) is not None:
            site[key] = data[key]
    if "created" in data:
        site["created"] = data["created"]
    if isinstance(data.get("maps"), dict):
        site["maps"] = {n: norm_map_entry(n, e)
                        for n, e in data["maps"].items()}
        if data.get("active_map") in site["maps"]:
            site["active_map"] = data["active_map"]
    elif data.get("default_map"):
        name = data["default_map"]
        site["maps"] = {name: norm_map_entry(
            name, {"map_start_pose": data.get("map_start_pose")})}
        site["active_map"] = name
    return site


def validate_map_entry(entry):
    """Error string for a bad per-map patch value, else None."""
    if "floor" in entry and entry["floor"] is not None \
            and not isinstance(entry["floor"], int):
        return "floor must be an integer or null"
    if "map_start_pose" in entry and (
            not isinstance(entry["map_start_pose"], list)
            or len(entry["map_start_pose"]) != 3):
        return "map_start_pose must be [x, y, theta]"
    if "source" in entry and entry["source"] not in SOURCES:
        return f'source must be one of {", ".join(SOURCES)}'
    return None


def apply_maps_patch(site, patch_maps):
    """Merge a per-map patch into a normalized site in place; null deletes an
    entry. Returns an error string (site possibly part-applied: the caller
    must not write it) or None."""
    for name, entry in patch_maps.items():
        if not SITE_NAME_RE.match(name or ""):
            return f"invalid map name '{name}'"
        if entry is None:
            site["maps"].pop(name, None)
            if site["active_map"] == name:
                site["active_map"] = None
            continue
        if not isinstance(entry, dict):
            return f"maps['{name}'] must be an object or null"
        err = validate_map_entry(entry)
        if err:
            return err
        merged = {**site["maps"].get(name, norm_map_entry(name, None))}
        merged.update({k: entry[k] for k in _MAP_DEFAULTS if k in entry})
        site["maps"][name] = norm_map_entry(name, merged)
    return None


def site_files(site_dir):
    """{basename: {"posegraph": bool, "grid": bool}} from the maps dir."""
    maps_dir = os.path.join(site_dir, "maps")
    files = {}
    if os.path.isdir(maps_dir):
        for f in os.listdir(maps_dir):
            if f.startswith("_"):      # _autosave/_pending: not operator maps
                continue
            if f.endswith(".posegraph"):
                files.setdefault(f[:-len(".posegraph")], {})["posegraph"] = True
            elif f.endswith(".yaml"):
                # Grid (.yaml/.pgm) availability decides whether localization
                # mode can start at all (slam.launch.py hard-refuses without
                # it, ADR-0028); pre-ADR-0028 sites have the posegraph only.
                files.setdefault(f[:-len(".yaml")], {})["grid"] = True
    return files


def resolved_mode(slam_mode, active_map, maps, files):
    """What slam_mode will run as. Mirror of scout.core.sites.resolve_slam's
    auto policy, minus its raises: a status payload reports, it never refuses."""
    if slam_mode != "auto":
        return slam_mode
    if not active_map:
        return "new"
    present = files.get(active_map, {})
    drawing = maps.get(active_map, {}).get("source") == "drawing"
    if drawing and present.get("grid"):
        return "localization"
    return "continue" if present.get("posegraph") else "new"


def describe_site(meta, files):
    """Add per-map `posegraph`/`grid` flags and `resolved_mode` to a site meta
    dict in place. Files on disk with no site.json entry (hand-copied, pre-v2
    saves) are listed as unregistered so nothing saved by hand disappears."""
    maps = meta.setdefault("maps", {})
    for name, entry in maps.items():
        entry["posegraph"] = files.get(name, {}).get("posegraph", False)
        entry["grid"] = files.get(name, {}).get("grid", False)
    for name, present in sorted(files.items()):
        if name not in maps:
            maps[name] = {
                **norm_map_entry(name, None), "unregistered": True,
                "posegraph": present.get("posegraph", False),
                "grid": present.get("grid", False)}
    meta["resolved_mode"] = resolved_mode(
        meta.get("slam_mode", "auto"), meta.get("active_map"), maps, files)
    return meta


def write_site_json(site_dir, data):
    """Atomic site.json write (tmp + os.replace). Callers that read-modify-write
    hold SITE_LOCK across the read and this write."""
    tmp = os.path.join(site_dir, ".site.json.tmp")
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
    os.replace(tmp, os.path.join(site_dir, "site.json"))
