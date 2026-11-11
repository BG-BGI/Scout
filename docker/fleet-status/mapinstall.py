"""Install a finished occupancy-grid pair into the active site over HTTP, no posegraph.

Pure stdlib; imports sitestore, mapsafe and gridstore, never server.py. Used for
drawing-sourced maps (a floorplan rasterized off the Pi): the pair is a
`<name>.pgm` + `<name>.yaml` in the active site's maps dir, a `<name>.drawing.json`
sidecar records where it came from, and site.json gets an entry with
`source: "drawing"`. A scan map can never be replaced or quarantined through this
module (R11): there is no `force` field, and any request carrying one is a 400.

POST /api/maps/install      (only when SITE_SCAFFOLD == "pi"; otherwise 404)
  Request: Content-Type application/json, Content-Length required and at most
  MAX_BODY_BYTES (411 / 413 before the body is read). One JSON object; unknown
  fields are a 400:
    name            str, required. ^[a-z0-9][a-z0-9_-]{0,31}$ (fullmatch), no
                    leading "_" (reserved for _autosave/_pending/_quarantine).
    yaml            str, required, at most MAX_YAML_BYTES. nav2 map_server yaml
                    text. Required keys: resolution (>0), origin [x, y, yaw],
                    negate (0 or 1), occupied_thresh and free_thresh (0 <= free
                    < occupied <= 1). Optional: mode (trinary | scale | raw,
                    default trinary). `image:` is ignored and rewritten. The
                    stored yaml is regenerated from the validated values
                    (image, mode, resolution, origin, negate, occupied_thresh,
                    free_thresh); any other key is dropped.
    pgm_b64         str, required. Standard base64 (no newlines) of a binary P5
                    PGM, maxval 255, at most MAX_INSTALL_BYTES decoded, at most
                    MAX_CELLS cells, pixel data exactly width*height bytes.
    sidecar         object, required. Provenance JSON, at most MAX_SIDECAR_BYTES
                    serialized. Stored as `<name>.drawing.json`. Its presence on
                    disk is what marks `<name>` as a drawing map that may be
                    replaced in place.
    label           str, optional, at most MAX_LABEL chars, no control chars.
                    Default: the name (a replace keeps the existing label).
    floor           int or null, optional (a replace keeps the existing floor).
    map_start_pose  [x, y, theta], finite numbers, optional, default [0, 0, 0]
                    (a replace keeps the existing pose).
    source          optional; if present it must be "drawing".
  Success 200: {"ok": true, "name", "replaced": bool, "source": "drawing",
    "width", "height", "resolution", "origin": [x, y, yaw],
    "bytes": {"pgm", "yaml", "sidecar"}, "entry": <the site.json map entry>,
    "active_map": <site.json active_map, unchanged by the install>}
  Errors, always {"error": str}:
    400 bad JSON / field / name / label / floor / pose / source / base64
    404 not a "pi" scaffold, or the active site has no maps dir
    409 no active site; `<name>.posegraph` or `.data` exists; the entry is not
        source drawing; grid files or an entry exist without a sidecar; site.json
        is not valid JSON. Nothing on disk changes.
    411 Content-Length missing      413 body, pgm, yaml or sidecar too large
    422 pgm header / size / cell cap / payload length, or yaml keys, invalid
    500 a write failed (see the rollback rules in `install`)
  Install never changes active_map and never reloads the running localizer:
  the operator activates the map separately.

GET /api/maps/<name>.pgm  and  /api/maps/<name>.yaml
  Raw file from the active site's maps dir, any map (scan or drawing). 200 with
  the file bytes, Content-Type image/x-portable-graymap or text/yaml, and
  Cache-Control: no-store. 400 invalid name (traversal, leading "_", newline),
  404 unknown name or extension.
"""

import base64
import binascii
import json
import math
import os
import unicodedata

import gridstore
import mapsafe
import sitestore

MAX_INSTALL_BYTES = 16 * 1024 * 1024
MAX_YAML_BYTES = 16 * 1024
MAX_SIDECAR_BYTES = 64 * 1024
MAX_BODY_BYTES = MAX_INSTALL_BYTES * 4 // 3 + MAX_YAML_BYTES + MAX_SIDECAR_BYTES + 4096
MAX_CELLS = 4000 * 4000
MAX_LABEL = 64
MAX_ABS = 1.0e6
MAX_RESOLUTION = 10.0
PGM_MAXVAL = 255
SOURCE = "drawing"
SIDECAR_EXT = ".drawing.json"
NAME_RE = mapsafe.NAME_RE
FIELDS = frozenset({"name", "yaml", "pgm_b64", "sidecar", "label", "floor",
                    "map_start_pose", "source"})
YAML_MODES = ("trinary", "scale", "raw")
YAML_KEYS = ("resolution", "origin", "negate", "occupied_thresh", "free_thresh")
RAW_TYPES = {".pgm": "image/x-portable-graymap", ".yaml": "text/yaml; charset=utf-8"}


class InstallError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code
        self.message = message


def valid_name(name):
    return isinstance(name, str) and bool(NAME_RE.fullmatch(name)) \
        and not name.startswith("_")


def check_body_length(header):
    """None if the Content-Length header allows reading the body, else (code, payload)."""
    if header is None:
        return 411, {"error": "Content-Length required"}
    try:
        length = int(header)
    except ValueError:
        return 400, {"error": "invalid Content-Length"}
    if length < 0:
        return 400, {"error": "invalid Content-Length"}
    if length > MAX_BODY_BYTES:
        return 413, {"error": f"body is {length} B (max {MAX_BODY_BYTES} B)"}
    return None


def _finite(value):
    v = float(value)
    if not math.isfinite(v) or abs(v) > MAX_ABS:
        raise ValueError(value)
    return v


def _is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(v) and abs(v) <= MAX_ABS


def _entry_patch(body):
    if body.get("source", SOURCE) != SOURCE:
        raise InstallError(400, f"source must be '{SOURCE}' for an installed map")
    patch = {"source": SOURCE}
    if "label" in body:
        label = body["label"]
        if not isinstance(label, str) or len(label) > MAX_LABEL \
                or any(unicodedata.category(c) == "Cc" for c in label):
            raise InstallError(
                400, f"label must be a string of at most {MAX_LABEL} chars "
                     "with no control characters")
        patch["label"] = label
    if "floor" in body:
        floor = body["floor"]
        if isinstance(floor, bool):
            raise InstallError(400, "floor must be an integer or null")
        patch["floor"] = floor
    if "map_start_pose" in body:
        pose = body["map_start_pose"]
        if not isinstance(pose, list) or len(pose) != 3 \
                or not all(_is_number(v) for v in pose):
            raise InstallError(400, "map_start_pose must be [x, y, theta] numbers")
        patch["map_start_pose"] = [float(v) for v in pose]
    err = sitestore.validate_map_entry(patch)
    if err:
        raise InstallError(400, err)
    return patch


def _parse_yaml(text):
    if not isinstance(text, str):
        raise InstallError(400, "yaml must be a string")
    try:
        size = len(text.encode())
    except UnicodeEncodeError as e:
        raise InstallError(400, "yaml is not valid UTF-8 text") from e
    if size > MAX_YAML_BYTES:
        raise InstallError(413, f"yaml is over {MAX_YAML_BYTES} B")
    kv = {}
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if ":" in line:
            k, v = line.split(":", 1)
            kv[k.strip()] = v.strip()
    missing = [k for k in YAML_KEYS if k not in kv]
    if missing:
        raise InstallError(422, f"yaml is missing: {', '.join(missing)}")
    try:
        res = _finite(kv["resolution"])
        origin = [_finite(t) for t in kv["origin"].strip("[]").split(",")]
        occupied, free = _finite(kv["occupied_thresh"]), _finite(kv["free_thresh"])
        negate = int(kv["negate"])
    except ValueError as e:
        raise InstallError(422, f"yaml has a non-numeric or non-finite value: {e}") from e
    mode = kv.get("mode", "trinary")
    if not 0 < res <= MAX_RESOLUTION or len(origin) != 3 or negate not in (0, 1) \
            or not 0 <= free < occupied <= 1 or mode not in YAML_MODES:
        raise InstallError(
            422, "yaml needs resolution in (0, 10], origin [x, y, yaw], negate 0 or 1, "
                 f"0 <= free_thresh < occupied_thresh <= 1, mode one of {', '.join(YAML_MODES)}")
    return {"resolution": res, "origin": origin, "negate": negate, "mode": mode,
            "occupied": occupied, "free": free}


def _render_yaml(name, p):
    ox, oy, oyaw = p["origin"]
    return (f"image: {name}.pgm\nmode: {p['mode']}\nresolution: {p['resolution']!r}\n"
            f"origin: [{ox!r}, {oy!r}, {oyaw!r}]\nnegate: {p['negate']}\n"
            f"occupied_thresh: {p['occupied']!r}\nfree_thresh: {p['free']!r}\n")


def _parse_pgm(b64):
    if not isinstance(b64, str):
        raise InstallError(400, "pgm_b64 must be a string")
    if len(b64) > MAX_INSTALL_BYTES * 4 // 3 + 8:
        raise InstallError(413, f"pgm is over {MAX_INSTALL_BYTES} B")
    try:
        raw = base64.b64decode(b64, validate=True)
    except (binascii.Error, ValueError) as e:
        raise InstallError(400, "pgm_b64 is not valid base64") from e
    if len(raw) > MAX_INSTALL_BYTES:
        raise InstallError(413, f"pgm is {len(raw)} B (max {MAX_INSTALL_BYTES} B)")
    try:
        w, h, maxval, offset = gridstore.parse_pgm_header(raw, "pgm")
    except gridstore.GridError as e:
        raise InstallError(422, str(e)) from e
    if maxval != PGM_MAXVAL:
        raise InstallError(422, f"pgm maxval must be {PGM_MAXVAL}, got {maxval}")
    if w * h > MAX_CELLS:
        raise InstallError(422, f"pgm is {w}x{h} cells (max {MAX_CELLS})")
    if len(raw) - offset != w * h:
        raise InstallError(
            422, f"pgm pixel data is {len(raw) - offset} B, header says {w}x{h} = {w * h}")
    return raw, w, h


def _sidecar_text(sidecar):
    if not isinstance(sidecar, dict):
        raise InstallError(400, "sidecar must be an object")
    text = json.dumps(sidecar, indent=2)
    if len(text.encode()) > MAX_SIDECAR_BYTES:
        raise InstallError(413, f"sidecar is over {MAX_SIDECAR_BYTES} B")
    return text


def parse_body(raw):
    """Request bytes -> validated {name, patch, yaml, pgm, width, height, sidecar}.
    Raises InstallError; touches nothing on disk."""
    try:
        body = json.loads(raw)
    except ValueError as e:
        raise InstallError(400, "body is not valid JSON") from e
    if not isinstance(body, dict):
        raise InstallError(400, "body must be a JSON object")
    unknown = sorted(set(body) - FIELDS)
    if unknown:
        raise InstallError(400, f"unknown field(s): {', '.join(unknown)[:80]}")
    if not valid_name(body.get("name")):
        raise InstallError(
            400, "invalid map name (a-z0-9_-, max 32, no leading underscore)")
    name = body["name"]
    patch = _entry_patch(body)
    params = _parse_yaml(body.get("yaml"))
    pgm, width, height = _parse_pgm(body.get("pgm_b64"))
    return {"name": name, "patch": patch, "params": params, "pgm": pgm,
            "width": width, "height": height,
            "sidecar": _sidecar_text(body.get("sidecar")),
            "yaml": _render_yaml(name, params)}


def _load_site(site_dir):
    path = os.path.join(site_dir, "site.json")
    try:
        with open(path) as f:
            raw = json.load(f)
    except FileNotFoundError:
        raw = {}
    except json.JSONDecodeError as e:
        raise InstallError(409, "site.json is not valid JSON; refusing to overwrite it") from e
    site = sitestore.normalize_site(raw)
    if isinstance(raw, dict) and site["active_map"] is None \
            and isinstance(raw.get("active_map"), str):
        site["active_map"] = raw["active_map"]
    return site


def _plan(site, maps_dir, name):
    """True when this install replaces a drawing map in place; raises 409 when
    anything on disk or in site.json says the name is not a drawing map."""
    def has(ext):
        return os.path.lexists(os.path.join(maps_dir, name + ext))

    if has(".posegraph") or has(".data"):
        raise InstallError(
            409, f"'{name}' is a scanned map (it has a posegraph); "
                 "install never replaces a scan map")
    entry = site["maps"].get(name)
    if entry is not None and entry["source"] != SOURCE:
        raise InstallError(409, f"'{name}' is registered as a {entry['source']} map")
    present = entry is not None or has(".yaml") or has(".pgm")
    if present and not has(SIDECAR_EXT):
        raise InstallError(
            409, f"'{name}' already exists and has no drawing sidecar, so it is not "
                 "a drawing map this endpoint may replace")
    return present


def _write_files(maps_dir, name, files, written):
    tmps = []
    try:
        for ext, data in files:
            final = os.path.join(maps_dir, name + ext)
            tmp = final + ".tmp"
            with open(tmp, "wb") as f:
                f.write(data)
                f.flush()
                os.fsync(f.fileno())
            tmps.append((tmp, final))
        for tmp, final in tmps:
            os.replace(tmp, final)
            written.append(final)
    finally:
        for tmp, _ in tmps:
            if os.path.exists(tmp):
                os.remove(tmp)


def _remove_quietly(paths):
    for p in paths:
        try:
            os.remove(p)
        except OSError:
            pass


def install(site_dir, req):
    """Validated request -> (200, payload). Under SITE_LOCK: check the name is
    free or a drawing map, build the new site.json in memory, write pgm, sidecar,
    yaml (each tmp + os.replace), then site.json. Rollback: a FRESH install whose
    write or registration fails removes every file it wrote. A REPLACE that fails
    leaves the files it already swapped (the new grid under the old entry is a
    consistent state; the entry stays registered as drawing) and returns 500 so
    the caller retries."""
    maps_dir = os.path.join(site_dir, "maps")
    if not os.path.isdir(maps_dir):
        raise InstallError(404, "no active site maps dir")
    name = req["name"]
    with sitestore.SITE_LOCK:
        site = _load_site(site_dir)
        replaced = _plan(site, maps_dir, name)
        err = sitestore.apply_maps_patch(site, {name: req["patch"]})
        if err:
            raise InstallError(400, err)
        site["default_map"] = site["active_map"]
        files = [(".pgm", req["pgm"]), (SIDECAR_EXT, req["sidecar"].encode()),
                 (".yaml", req["yaml"].encode())]
        written = []
        try:
            _write_files(maps_dir, name, files, written)
            sitestore.write_site_json(site_dir, site)
        except OSError as e:
            if not replaced:
                _remove_quietly(written)
            raise InstallError(500, f"install failed: {e.strerror or e}") from e
    return 200, {
        "ok": True, "name": name, "replaced": replaced, "source": SOURCE,
        "width": req["width"], "height": req["height"],
        "resolution": req["params"]["resolution"], "origin": req["params"]["origin"],
        "bytes": {"pgm": len(req["pgm"]), "yaml": len(req["yaml"]),
                  "sidecar": len(req["sidecar"])},
        "entry": site["maps"][name], "active_map": site["active_map"]}


def install_request(site_dir, raw):
    """Request bytes -> (code, payload); every refusal is an InstallError."""
    try:
        return install(site_dir, parse_body(raw))
    except InstallError as e:
        return e.code, {"error": e.message}
    except OSError as e:
        return 500, {"error": f"install failed: {e.strerror or e}"}


def get_file(maps_dir, filename):
    """Raw map file -> (200, bytes, content_type) or (code, {"error": ...}, None)."""
    name, ext = os.path.splitext(filename or "")
    if not valid_name(name):
        return 400, {"error": "invalid map name"}, None
    if ext not in RAW_TYPES:
        return 404, {"error": "not found"}, None
    try:
        with open(os.path.join(maps_dir, name + ext), "rb") as f:
            return 200, f.read(), RAW_TYPES[ext]
    except OSError:
        return 404, {"error": f"no such map file '{name}{ext}'"}, None
