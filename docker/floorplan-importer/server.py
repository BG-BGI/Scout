"""Floorplan importer HTTP service: PDF or IFC in, nav2 grid pair(s) out, installed on the Pi.

Stdlib http.server (ThreadingHTTPServer, fleet_status handler style) over pdfsource, calib and
planraster. Runs off the Pi (the companion, or a laptop); the only thing it talks to besides its
clients is fleet_status on the Pi (FLEET_STATUS_URL), which owns the install. All JSON errors are
{"error": str}; no traceback ever reaches a client. This docstring is the contract the web UI
is written against.

Environment (the service never takes a URL, path or command from a request)
    IMPORTER_PORT          listen port, default 9004
    IMPORTER_CORS_ORIGIN   Access-Control-Allow-Origin value, default "*"
    FLEET_STATUS_URL       the Pi's fleet_status base, e.g. http://<pi>:9003 (http/https, no
                           credentials). Unset or unusable (a warning is logged) = the
                           service still starts and installs answer 503.
    IMPORTER_VERSION       build id recorded in the sidecar (CI passes the commit sha)
    IMPORTER_MAX_IFC_BYTES IFC upload cap in bytes, default 80 MiB (see the memory note on
                           MAX_IFC_BYTES; raise the container memory first)
    IMPORTER_IFC_THREADS   geometry-iterator threads per slice, default 1 (1 CPU container)
    IMPORTER_IFC_FACE_BUDGET  triangle budget per slice, default ifcsource's 3,000,000
    IMPORTER_MEMORY_LIMIT_MB  the container memory limit the IFC memory gate works against;
                           default: read from the cgroup (v2 or v1); unreadable = gate off

Conventions
  Page units / page frame / PNG pixel mapping are exactly pdfsource's: page units are PDF
  user-space units; the page frame is the VISIBLE page (after /Rotate and /CropBox) with its
  origin at the bottom-left and y UP. Every point the client sends (p1, p2, origin, spans,
  region) is in the page frame, page units. Page indices are 0-based (PageInfo.index).
  The PNG at GET .../page/<n>.png is the visible page; pixel (px, py) has py counted from the
  top. With scale_x / scale_y (pixels per page unit) and height (page units) from the page JSON:
        page_x = px / scale_x            page_y = height - py / scale_y
        px = page_x * scale_x            py = (height - page_y) * scale_y
  A map is built by calib.map_frame(m_per_unit, rotation, origin): the page point `origin`
  becomes map (0, 0) and the map +X axis points `rotation_deg` counter-clockwise from the page
  +x axis (y up). m_per_unit is metres per page unit (the same number the proposal calls
  m_per_point).
  Heavy operations (upload parse, page render, scale proposal, calibrate) run one at a time
  process-wide: a second one answers 429 {"error": "busy"} with Retry-After (seconds) instead
  of queueing; so does a request for a job that is mid-operation. Retry after the delay.
  Jobs live in memory: at most 3 retained (the oldest idle one is evicted when a fourth is
  uploaded; 429 if none is idle), each dropped after 2 h without a request. An unknown or
  expired job id is 404. At most ONE IFC model is open at a time (it is ~10x its file in
  memory): uploading an IFC evicts the open one and, if needed, other idle jobs BEFORE the
  body is read, even if the new file then fails to parse; 429 if one of them is mid-operation.

GET /api/health
  200 {"ok": true, "version", "fleet_status_url" (null when unset), "jobs": n, "busy": bool,
       "ifc": bool (false when ifcopenshell / shapely are not installed),
       "rss_mb": resident set in MiB (null off Linux)}

POST /api/import[?filename=<name>]
  Request: the file bytes as the raw body (any Content-Type; no multipart). Content-Length is
  required (411); above the larger of the two caps (pdfsource.MAX_PDF_BYTES for a PDF,
  IMPORTER_MAX_IFC_BYTES for an IFC) it is 413 before a byte is read. Otherwise only the first
  2 KiB are read to sniff the kind and a body over THAT kind's cap is 413 without reading the
  rest (an IFC 413 also carries {"count", "limit"}).
  `filename` is for display and the sidecar only; it is reduced to a safe base name, never a
  path. The kind is sniffed from the bytes: a %PDF- header makes a "pdf" job; an
  ISO-10303-21 header makes an "ifc" job (415 when this build has no IFC support); anything
  else is 415. An IFC is streamed to a private temp file (nothing of it is held in memory),
  parsed from that file and the file is deleted as soon as the parse ends: only sha256 and
  size are kept.
  An IFC upload is first checked against the importer's memory: if the current resident size
  plus ~15x the file would pass 90 % of the container limit it is 503 {"error", "rss_mb",
  "needed_mb", "limit_mb"} with Retry-After 15, every open import is dropped and the process
  restarts itself (compose brings it back); 429 instead when an import is mid-operation.
  201 {"id", "kind": "pdf", "filename", "sha256", "bytes", "pages": [<page>], "storeys": []}
  or, for an IFC, {"id", "kind": "ifc", "filename", "sha256", "bytes", "pages": [],
  "storeys": [<storey>]} (see IFC jobs below).
  <page> = pdfsource.PageInfo as an object {index, mediabox [x0,y0,x1,y1], width, height,
  rotation, userunit, n_layers, n_paths, n_images, has_vectors} plus "scale": the
  page's scale report or null if not computed yet (computed eagerly for documents of at most
  10 pages, otherwise by GET .../page/<n>).
  Errors map pdfsource's codes: not_pdf 415; corrupt_pdf, encrypted_pdf, empty_pdf,
  extraction_failed 422; too_large, too_many_pages 413; page_out_of_range 404; bad_options 400.
  IFC: IfcParseError, NoStoreysError, IfcSourceError 422; FileTooLargeError, FaceBudgetError
  413 {"error", "count", "limit"}. 400 bad Content-Length; 429 busy.

GET /api/import/<id>
  200 {"id", "kind", "filename", "sha256", "bytes", "expires_in" (seconds), "pages": [<page>],
       "storeys": [] (a PDF job; see IFC jobs below for an IFC job),
       "selected_page" (the page of the current calibration, else null),
       "calibration": the last calibrate request's effective values or null,
       "extraction": {"kind", "reason", "diagnostics"} or null,
       "preview": {"width", "height", "resolution", "origin": [x, y, yaw],
                   "cells": {"occupied", "free", "unknown"},
                   "interior_fraction" (free cells / bounding box of the occupied cells),
                   "occupied_extent_m": {"width", "height"}} or null,
       "last_install": {"status", "at", "response"} or null}
  A scale report is {"proposal": {"m_per_unit", "m_per_point" (same number), "evidence":
  [str], "confidence": "high"|"medium"|"low"|"none"}, "calibration_required": bool,
  "raster": bool}. m_per_unit is null when there is no proposal; the operator must accept a
  proposal explicitly (method "proposal"); it is never applied on its own.

DELETE /api/import/<id>        200 {"ok": true, "id"}; the PDF is closed and the job dropped.

GET /api/import/<id>/page/<n>[?px_per_pt=1.5]
  200 {"index", "page": <page>, "scale": <scale report>, "png": {"url", "width_px",
       "height_px", "scale_x", "scale_y", "width", "height"}} (renders and caches the PNG).
GET /api/import/<id>/page/<n>.png[?px_per_pt=1.5]
  200 image/png, no-store. Mapping also in headers X-Scale-X, X-Scale-Y (pixels per page
  unit), X-Page-Width, X-Page-Height (page units). px_per_pt is pixels per physical point,
  0.1..8, clamped so the image stays under pdfsource.MAX_PNG_PIXELS (read the real scale
  from the JSON or headers, never assume it). 404 page out of range; 400 bad px_per_pt.

POST /api/import/<id>/calibrate
  Request JSON object (unknown fields are 400), at most 64 KiB. Fields:
    page            int, default 0 for a one-page PDF, otherwise required
    method          "proposal" | "two_point" | "typed" | "robot"
    <method params> flat in the object (or inside an optional "params" object):
        proposal   none. Accept the page's scale proposal (400 if it has none).
        two_point  p1 [x, y], p2 [x, y] (page frame), metres (> 0).
        typed      exactly one of scale (text such as "1/4\\" = 1'-0\\"" or "1:100") or
                   ratio (real length / paper length, number > 0); optional paper_in [w, h]
                   inches of the intended FULL-SIZE sheet when the PDF is a reduced print
                   (orientation ignored; must match the PDF's aspect within 3 %).
        robot      spans [{"p1","p2","metres"}, ...]: lidar-measured wall-pair spans against
                   the same pair marked on the page. At least 2; 400 "need two non-parallel
                   spans" unless some pair differs in direction by 20 degrees or more.
                   Scale = calib.check_spans mean. Spans that disagree by more than
                   calib.SPAN_TOLERANCE (1 %) are NOT an error: 200 with a warning and the
                   mean is still applied.
    rotation_deg    number, default 0 (see Conventions)
    origin          [x, y] page point that becomes map (0, 0). Default: the lower-left corner
                    of the extracted wall geometry's bounding box (reported back).
    resolution      metres per cell, 0.01..1.0, default 0.05
    close_m         doorway closing length passed to planraster.rasterize, 0 < close_m <= 5,
                    default planraster.CLOSE_M (1.2). A wall gap at least this wide makes the
                    interior read unknown (a warning is added when interior_fraction < 0.1).
    wall_options    object of pdfsource.WallOptions fields (vector extraction)
    raster_options  object of pdfsource.RasterOptions fields (raster extraction)
    force_raster    bool, default false. Extraction is raster (render + threshold) when this
                    is true or the page is raster; otherwise vector.
  200 {"page", "m_per_unit", "method", "rotation_deg", "origin": [x, y], "resolution",
       "close_m", "detail" (method-specific: proposal evidence; robot per_span, mean, spread,
       consistent; typed ratio), "extraction": {"kind", "reason", "diagnostics"},
       "preview": {as in the job state}, "warnings": [str]}
  The extraction runs and the grid is rasterized and cached on the job; calibrating again
  REPLACES the cached grid (recalibrate regenerates, it never overlays).
  Errors: 400 bad field / values / options / calibration (coincident points, bad spans); 404
  page out of range; 422 no wall geometry (vector page, nothing wall-like: retry with
  force_raster or wall_options), grid too large (cap planraster.MAX_CELLS) or empty; 429 busy.

GET /api/import/<id>/preview.png
  200 the cached grid as PNG (occupied dark, free white, unknown mid-grey; row 0 is the top).
GET /api/import/<id>/preview.pgm
  200 image/x-portable-graymap, exactly the bytes install sends (planraster.encode_pgm).
  Both 409 until a calibrate has succeeded; no-store.

POST /api/import/<id>/install
  Request JSON {"name", "label"?, "floor"?, "map_start_pose"?}; nothing else (unknown fields
  400, there is no force). Builds fleet_status's POST /api/maps/install payload (name, yaml =
  planraster.yaml_text, pgm_b64 = base64 of the cached grid's PGM, sidecar, source "drawing",
  and label / floor / map_start_pose only when given) and sends it to FLEET_STATUS_URL
  (timeout 15 s). fleet_status's status and JSON body are relayed unchanged, so its 409
  (name taken by a scan map) and 422 reach the UI as they are. 409 {"error"} before any
  calibrate; 503 FLEET_STATUS_URL unset; 502 {"error", "upstream"} when fleet_status is
  unreachable, times out or answers non-JSON (the job is kept; retry). Install never changes
  the active map; the operator activates the map separately.
  The sidecar records: importer_version, kind, source_file {name, sha256, bytes}, page,
  calibration {method, params, m_per_unit, rotation_deg, origin, resolution, close_m},
  extraction {kind, compact diagnostics}, created_utc.

IFC jobs (kind "ifc"; a model is sliced per storey, there is no calibration step)
  <storey> = {"id" (int, the STEP id; stable for one file), "global_id", "name" (raw, may be
  ""), "label" (name, else long name, else "Storey N"), "elevation_m", "floor" (1..N by
  elevation), "slice": <slice> | null, "last_install": {"status", "at", "response"} | null}.
  Storeys are listed in floor order. "slice" is null until that storey is sliced, and again
  if its cached grid is dropped to keep the job's cached grids within 128 MiB (the oldest
  sliced storey goes first; the new slice's "warnings" says so): slice it again.
  GET /api/import/<id> for an IFC: {"id", "kind": "ifc", "filename", "sha256", "bytes",
  "expires_in", "pages": [], "storeys": [<storey>]}.
  The PDF routes (page, calibrate, preview, install) on an IFC job are 409 naming the
  storey routes; the storey routes on a PDF job are 409 naming the PDF routes. DELETE and GET
  /api/import/<id> work for both. An unknown storey id is 404.

POST /api/import/<id>/storey/<sid>/slice
  Request JSON object (unknown fields 400), at most 64 KiB; every field optional:
    slice_height   metres above the storey's elevation, 0.01..5.0, default 0.241 (the lidar
                   height); the plane is at z = elevation_m + slice_height
    rotation_deg   default 0; the map +X axis points this many degrees counter-clockwise from
                   the IFC +x (east) axis
    origin         [x, y] IFC world metres that become map (0, 0). Default: the lower-left
                   corner of the slice bounds (reported back; real exports can sit hundreds of
                   metres from (0, 0))
    resolution, close_m   as for calibrate (0.01..1.0 default 0.05; 0 < close_m <= 5 default 1.2)
  200 <slice> = {"storey": {"id", "label", "elevation_m", "floor"}, "z", "slice_height",
       "rotation_deg", "origin": [x, y], "resolution", "close_m", "bounds": [minx, miny, maxx,
       maxy] (slice bounds, IFC world metres), "faces" (triangles examined), "preview": {as for
       a PDF}, "warnings": [str]}
  The scale is 1 (IFC is metric: calib.map_frame(1.0, rotation, origin)). Slicing again
  REPLACES that storey's cached grid; a failed slice leaves the old one. One storey per call;
  it holds the single heavy slot (429 busy otherwise) for the whole parse-free slice.
  Errors: 400 bad field; 404 unknown storey; 409 not an IFC job; 413 FaceBudgetError
  {"error", "count", "limit"} (the count is the running total when iteration stopped); 422
  EmptySliceError (nothing solid at that height; the message names the storey), a grid over
  planraster's cap, or other IfcSourceError; 429 busy.

GET /api/import/<id>/storey/<sid>/preview.png  and  .../preview.pgm
  As the PDF previews (the PGM is exactly what install sends); 409 until that storey is sliced.

POST /api/import/<id>/storey/<sid>/install
  Request JSON {"name", "label"?, "floor"?, "map_start_pose"?} (unknown fields 400, no
  force). label and floor default to the storey's own label and floor when omitted (floor may
  be given as null). The payload, relay, 502 / 503 / 409 rules and the result recorded as
  last_install (on the storey) are as for a PDF install. Sidecar: {"importer_version",
  "kind": "ifc", "source_file": {"name", "sha256", "bytes"}, "storey": {"id", "global_id",
  "name", "label", "elevation_m"}, "slice": {"slice_height", "z", "rotation_deg", "origin",
  "resolution", "close_m", "faces"}, "created_utc"}.

CORS: every response carries Access-Control-Allow-Origin; OPTIONS answers 204 with
Allow-Methods GET, POST, DELETE, OPTIONS and Allow-Headers Content-Type. There is no motion
or robot surface here.
"""

import base64
import contextlib
import ctypes
import ctypes.util
import dataclasses
import datetime
import gc
import hashlib
import http.client
import json
import math
import os
import re
import secrets
import socket
import socketserver
import sys
import tempfile
import threading
import time
import traceback
import typing
import urllib.error
import urllib.parse
import urllib.request
from collections import OrderedDict
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import calib
import numpy as np
import pdfsource
import planraster
import pymupdf

try:
    import ifcjob
except ImportError as e:  # ifcopenshell / shapely not installed: PDF import still works
    ifcjob = None
    IFC_UNAVAILABLE = str(e)

VERSION = os.environ.get("IMPORTER_VERSION", "dev")
DEFAULT_PORT = 9004
MAX_JSON_BYTES = 64 * 1024
MAX_JOBS = 3
JOB_TTL_S = 2 * 3600
UPSTREAM_TIMEOUT_S = 15.0
UPSTREAM_MAX_BYTES = 1024 * 1024
EAGER_SCALE_PAGES = 10
PNG_CACHE = 4
DEFAULT_RESOLUTION = 0.05
MIN_RESOLUTION = 0.01
MAX_RESOLUTION = 1.0
MAX_CLOSE_M = 5.0
PARALLEL_DEG = 20.0
RETRY_AFTER_S = 2
MIN_PX_PER_PT = 0.1
MAX_PX_PER_PT = 8.0
MAX_FILENAME = 120
MAX_RATIO = 1.0e5
PAPER_ASPECT_TOL = 0.03
INTERIOR_WARN_FRACTION = 0.1
SIDECAR_BUDGET = 48 * 1024
SIDECAR_CLASSES = 12
SIDECAR_LAYERS = 30
HEADER_SNIFF = 2048
IFC_MAGIC = b"ISO-10303-21"
# IFC budgets, sized for the compose cap mem_limit 2g (2048 MiB). Measured here on an
# entity-dense synthetic model (20 MB, 6000 rooms) through this server with one iterator
# thread, so real exports can differ either way: python + numpy + PyMuPDF + ifcopenshell +
# shapely ~ 90-100 MiB resident. The upload is streamed to a temp file and opened by path:
# the open model is ~7x the file (peak while parsing ~7x, the file itself is not in memory)
# and slicing one storey adds ~8x, so the slice peak is ~15x. (Opening from bytes costs ~12x
# and ~20x; ifcsource's own docstring says 6x. Do not size from either.) With one IFC job and
# at most 2 other jobs retained (3 jobs total), each PDF job worst case 64 MiB stream + ~2
# parsed copies (~192 MiB), and a 128 MiB cap on cached grids, the slice peak is
# 100 + 2 * 192 + 128 + 15 * C <= 0.9 * 2048 = 1843, which gives C <= 82 MiB; C = 80 MiB
# leaves ~11 % headroom. IMPORTER_MAX_IFC_BYTES overrides it; raise mem_limit first
# (3g ~ 128 MiB, 6g ~ 256 MiB).
# CAVEAT: after an IFC model is freed, ifcopenshell hands back only part of its memory (the
# library alone, no server, kept ~3x the file per open/free cycle on macOS; Linux is
# unverified). So the arithmetic holds for the first model, not for an unbounded number of
# them: memory_gate() refuses an IFC that would not fit on top of the CURRENT resident size
# and recycles the process (compose restarts it), rather than let the kernel OOM-kill it in
# the middle of someone's slice. GET /api/health reports rss_mb to watch this.
IFC_PEAK_FACTOR = 15
MEMORY_HEADROOM = 0.9
RECYCLE_DELAY_S = 1.0
RECYCLE_EXIT_CODE = 75
MAX_IFC_BYTES = 80 * 1024 * 1024
IFC_GRID_BUDGET = 128 * 1024 * 1024
# compose gives this service 1 CPU while os.cpu_count() reports the host's cores, so the
# library's own default (min(4, cores)) would run 4 threads on one core and hold 4x the
# working set; one thread is also what the numbers above were measured with.
IFC_THREADS = 1
UPLOAD_CHUNK = 1024 * 1024
MIN_SLICE_HEIGHT = 0.01
MAX_SLICE_HEIGHT = 5.0
PREVIEW_GRAY = {planraster.OCCUPIED: 20, planraster.FREE: 255, planraster.UNKNOWN: 170}
JOB_ID_RE = r"[A-Za-z0-9_-]{16,64}"
FILENAME_OK = re.compile(r"[^A-Za-z0-9 ._()+,-]")
CODE_STATUS = {
    "not_pdf": 415, "corrupt_pdf": 422, "encrypted_pdf": 422, "empty_pdf": 422,
    "too_large": 413, "too_many_pages": 413, "page_out_of_range": 404, "bad_options": 400,
    "extraction_failed": 422,
}
CALIBRATE_FIELDS = frozenset({
    "page", "method", "params", "rotation_deg", "origin", "resolution", "close_m",
    "wall_options", "raster_options", "force_raster"})
METHOD_FIELDS = {
    "proposal": frozenset(),
    "two_point": frozenset({"p1", "p2", "metres"}),
    "typed": frozenset({"scale", "ratio", "paper_in"}),
    "robot": frozenset({"spans"}),
}
INSTALL_FIELDS = frozenset({"name", "label", "floor", "map_start_pose"})
SLICE_FIELDS = frozenset({"slice_height", "rotation_deg", "origin", "resolution", "close_m"})
KIND_HINTS = {
    "ifc": "this import is an IFC model: use POST /api/import/{id}/storey/<sid>/slice, GET "
           "/api/import/{id}/storey/<sid>/preview.png|pgm and POST "
           "/api/import/{id}/storey/<sid>/install",
    "pdf": "this import is a PDF: use GET /api/import/{id}/page/<n>, POST "
           "/api/import/{id}/calibrate, GET /api/import/{id}/preview.png|pgm and POST "
           "/api/import/{id}/install",
}


class ApiError(Exception):
    def __init__(self, code, message, headers=None, extra=None):
        super().__init__(message)
        self.code = code
        self.message = message
        self.headers = headers or {}
        self.extra = extra or {}


@dataclasses.dataclass(eq=False)
class Raw:
    code: int
    data: bytes
    content_type: str
    headers: dict = dataclasses.field(default_factory=dict)


@dataclasses.dataclass(eq=False)
class Job:
    """One import. A "pdf" job carries the page state (grid, preview, last_install live on
    the job itself); an "ifc" job carries the open model and one StoreyState per storey."""

    id: str
    kind: str
    filename: str
    sha256: str
    n_bytes: int
    created: float
    touched: float
    pdf: object = None
    pages: list = dataclasses.field(default_factory=list)
    model: object = None
    storeys: dict = dataclasses.field(default_factory=dict)
    slice_seq: int = 0
    lock: threading.Lock = dataclasses.field(default_factory=threading.Lock)
    scale: dict = dataclasses.field(default_factory=dict)
    pngs: OrderedDict = dataclasses.field(default_factory=OrderedDict)
    selected_page: int | None = None
    calibration: dict | None = None
    extraction: dict | None = None
    grid: object = None
    meta: dict | None = None
    preview: dict | None = None
    last_install: dict | None = None


# --- helpers ------------------------------------------------------------------------------

class IfcUpload(typing.NamedTuple):
    """An IFC upload already streamed to a private temp file (never held in memory)."""

    path: str
    sha256: str
    size: int


def rss_mb():
    """This process's resident set in MiB (Linux /proc), else None."""
    try:
        with open("/proc/self/statm") as fh:
            pages = int(fh.read().split()[1])
        return round(pages * os.sysconf("SC_PAGE_SIZE") / (1024 * 1024), 1)
    except (OSError, ValueError, IndexError):
        return None


CGROUP_LIMIT_FILES = ("/sys/fs/cgroup/memory.max",
                      "/sys/fs/cgroup/memory/memory.limit_in_bytes")


def cgroup_limit_mb(paths=CGROUP_LIMIT_FILES):
    """The container's memory limit in MiB from the cgroup files, or None."""
    for path in paths:
        try:
            with open(path) as fh:
                raw = fh.read().strip()
        except OSError:
            continue
        if raw.isdigit() and int(raw) < 2**40:
            return int(raw) / (1024 * 1024)
    return None


def _schedule_exit():
    """Exit shortly, after the 503 has been sent; the compose restart policy brings it back."""
    timer = threading.Timer(RECYCLE_DELAY_S, os._exit, args=(RECYCLE_EXIT_CODE,))
    timer.daemon = True
    timer.start()


def _trim_heap():
    """Hand freed heap back to the OS after an IFC model is dropped (glibc only)."""
    with contextlib.suppress(OSError, AttributeError, TypeError):
        libc = ctypes.CDLL(ctypes.util.find_library("c") or "libc.so.6")
        libc.malloc_trim(0)


def _now_utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds")


def _clean(obj):
    """JSON-safe copy: dataclasses and numpy scalars unwrapped, non-finite numbers -> null."""
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        obj = dataclasses.asdict(obj)
    if isinstance(obj, dict):
        return {str(k): _clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_clean(v) for v in obj]
    if isinstance(obj, np.ndarray):
        return _clean(obj.tolist())
    if isinstance(obj, np.generic):
        obj = obj.item()
    if isinstance(obj, float) and not math.isfinite(obj):
        return None
    return obj


def safe_filename(raw, default="upload.pdf"):
    """A display-only base name: no directories, no control or odd characters."""
    name = (raw or "").replace("\\", "/").rsplit("/", 1)[-1]
    name = FILENAME_OK.sub("_", name).strip(" .")[:MAX_FILENAME]
    return name or default


def _check_upstream(url):
    if not url:
        return None
    parts = urllib.parse.urlsplit(url.strip())
    if parts.scheme not in ("http", "https") or not parts.hostname:
        raise ValueError("FLEET_STATUS_URL must be an http(s) URL with a host")
    if parts.username or parts.password or parts.query or parts.fragment:
        raise ValueError("FLEET_STATUS_URL must not carry credentials, a query or a fragment")
    return f"{parts.scheme}://{parts.netloc}{parts.path.rstrip('/')}"


def _number(value, what, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ApiError(400, f"{what} must be a finite number")
    if (low is not None and value < low) or (high is not None and value > high):
        raise ApiError(400, f"{what} must be within {low}..{high}")
    return float(value)


def _point(value, what):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ApiError(400, f"{what} must be [x, y]")
    return (_number(value[0], f"{what}[0]"), _number(value[1], f"{what}[1]"))


def _strict_object(body, allowed, what):
    if not isinstance(body, dict):
        raise ApiError(400, f"{what} must be a JSON object")
    unknown = sorted(set(body) - allowed)
    if unknown:
        raise ApiError(400, f"unknown field(s) {unknown}; allowed: {sorted(allowed)}")
    return body


def _options(cls, raw, what):
    if raw is None:
        return cls()
    if not isinstance(raw, dict):
        raise ApiError(400, f"{what} must be an object")
    names = {f.name for f in dataclasses.fields(cls)}
    unknown = sorted(set(raw) - names)
    if unknown:
        raise ApiError(400, f"unknown {what} field(s) {unknown}; allowed: {sorted(names)}")
    return cls(**{k: tuple(v) if isinstance(v, list) else v for k, v in raw.items()})


def _directions_differ(spans):
    """True when some pair of span directions differs by at least PARALLEL_DEG (mod 180)."""
    angles = [math.atan2(p2[1] - p1[1], p2[0] - p1[0]) % math.pi for p1, p2, _ in spans]
    for i, a in enumerate(angles):
        for b in angles[i + 1:]:
            d = abs(a - b)
            if min(d, math.pi - d) >= math.radians(PARALLEL_DEG):
                return True
    return False


def _png_of_grid(grid):
    lut = np.full(256, 128, dtype=np.uint8)
    for value, gray in PREVIEW_GRAY.items():
        lut[value] = gray
    img = np.ascontiguousarray(lut[grid])
    pix = pymupdf.Pixmap(pymupdf.csGRAY, img.shape[1], img.shape[0], img.tobytes(), False)
    return pix.tobytes("png")


class _NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


# --- calibrate request ---------------------------------------------------------------------

def _calibration_request(body, job):
    """Validate a calibrate body into a flat dict (no I/O, no extraction)."""
    body = dict(_strict_object(body, CALIBRATE_FIELDS | set().union(*METHOD_FIELDS.values()),
                               "calibrate body"))
    nested = body.pop("params", None)
    if nested is not None:
        body.update(_strict_object(nested, set().union(*METHOD_FIELDS.values()), "params"))
    method = body.get("method")
    if method not in METHOD_FIELDS:
        raise ApiError(400, f"method must be one of {sorted(METHOD_FIELDS)}")
    stray = sorted(set(body) & set().union(*METHOD_FIELDS.values()) - METHOD_FIELDS[method])
    if stray:
        raise ApiError(400, f"field(s) {stray} do not belong to method '{method}'")
    page = body.get("page", 0 if len(job.pages) == 1 else None)
    if page is None:
        raise ApiError(400, "page is required for a multi-page PDF")
    if isinstance(page, bool) or not isinstance(page, int):
        raise ApiError(400, "page must be an integer")
    if not 0 <= page < len(job.pages):
        raise ApiError(404, f"page must be 0..{len(job.pages) - 1}")
    req = {k: body[k] for k in body if k in METHOD_FIELDS[method]}
    req.update(
        page=page, method=method,
        rotation_deg=_number(body.get("rotation_deg", 0.0), "rotation_deg", -3600, 3600),
        resolution=_number(body.get("resolution", DEFAULT_RESOLUTION), "resolution",
                           MIN_RESOLUTION, MAX_RESOLUTION),
        close_m=_number(body.get("close_m", planraster.CLOSE_M), "close_m", 1e-6, MAX_CLOSE_M),
        origin=None if body.get("origin") is None else _point(body["origin"], "origin"),
        force_raster=body.get("force_raster", False),
        wall_options=_options(pdfsource.WallOptions, body.get("wall_options"), "wall_options"),
        raster_options=_options(pdfsource.RasterOptions, body.get("raster_options"),
                                "raster_options"))
    if not isinstance(req["force_raster"], bool):
        raise ApiError(400, "force_raster must be true or false")
    return req


def _typed_scale(req, info):
    text, ratio = req.get("scale"), req.get("ratio")
    if (text is None) == (ratio is None):
        raise ApiError(400, "typed needs exactly one of scale (text) or ratio (number)")
    if text is not None:
        if not isinstance(text, str) or len(text) > 200:
            raise ApiError(400, "scale must be text such as '1/4\" = 1\\'-0\"' or '1:100'")
        notes = calib.parse_scale_notes(text)
        if not notes or len({round(n.ratio, 6) for n in notes}) != 1:
            raise ApiError(400, f"could not read one drawing scale from {text!r}")
        ratio = notes[0].ratio
    ratio = _number(ratio, "ratio", 1e-6, MAX_RATIO)
    factor, warnings = 1.0, []
    if req.get("paper_in") is not None:
        factor = _paper_factor(req["paper_in"], info)
        warnings.append(f"paper_in applied: scale multiplied by {factor:.4g}")
    m = ratio * calib.INCH_M * info.userunit / calib.PT_PER_INCH * factor
    return m, {"ratio": ratio, "paper_factor": factor}, warnings


def _paper_factor(paper, info):
    if not isinstance(paper, (list, tuple)) or len(paper) != 2:
        raise ApiError(400, "paper_in must be [width_in, height_in]")
    want = sorted(_number(v, "paper_in", 1e-3, 1e3) for v in paper)
    have = sorted((info.width * info.userunit / calib.PT_PER_INCH,
                   info.height * info.userunit / calib.PT_PER_INCH))
    if abs(want[1] / want[0] - have[1] / have[0]) > PAPER_ASPECT_TOL * have[1] / have[0]:
        raise ApiError(400, "paper_in does not match the PDF page's proportions")
    return want[1] / have[1]


def _robot_scale(req):
    raw = req.get("spans")
    if not isinstance(raw, list) or len(raw) < 2 or len(raw) > 32:
        raise ApiError(400, "robot needs 2..32 spans, each {p1, p2, metres}")
    spans = []
    for i, s in enumerate(raw):
        _strict_object(s, {"p1", "p2", "metres"}, f"span {i}")
        spans.append((_point(s.get("p1"), f"span {i} p1"), _point(s.get("p2"), f"span {i} p2"),
                      _number(s.get("metres"), f"span {i} metres", 1e-9)))
    for i, (p1, p2, _) in enumerate(spans):
        if math.dist(p1, p2) < 1e-9:
            raise ApiError(400, f"span {i}: p1 and p2 coincide")
    if not _directions_differ(spans):
        raise ApiError(400, "need two non-parallel spans (directions at least "
                            f"{PARALLEL_DEG:g} degrees apart)")
    check = calib.check_spans(spans)
    warnings = []
    if not check.consistent:
        warnings.append(
            f"spans disagree by {check.spread * 100:.1f}% "
            f"(limit {calib.SPAN_TOLERANCE * 100:g}%); the mean scale is applied, "
            "verify before installing")
    detail = {"mean": check.m_per_point, "per_span": list(check.per_span),
              "spread": check.spread, "consistent": check.consistent}
    return check.m_per_point, detail, warnings


def _proposal_scale(report):
    prop = report.proposal
    if report.calibration_required or prop.m_per_point is None:
        raise ApiError(400, "this page has no scale proposal; use two_point, typed or robot")
    warnings = []
    if prop.confidence != calib.HIGH:
        warnings.append(f"scale proposal confidence is {prop.confidence}; verify with two_point")
    return prop.m_per_point, {"confidence": prop.confidence, "evidence": list(prop.evidence)}, \
        warnings


def _two_point_scale(req):
    if "p1" not in req or "p2" not in req or "metres" not in req:
        raise ApiError(400, "two_point needs p1, p2 and metres")
    p1, p2 = _point(req["p1"], "p1"), _point(req["p2"], "p2")
    metres = _number(req["metres"], "metres", 1e-9)
    return calib.two_point(p1, p2, metres), {"p1": p1, "p2": p2, "metres": metres}, []


def _slice_request(body):
    """Validate a storey slice body (no I/O)."""
    _strict_object(body, SLICE_FIELDS, "slice body")
    return {
        "slice_height": _number(body.get("slice_height", ifcjob.DEFAULT_SLICE_HEIGHT),
                                "slice_height", MIN_SLICE_HEIGHT, MAX_SLICE_HEIGHT),
        "rotation_deg": _number(body.get("rotation_deg", 0.0), "rotation_deg", -3600, 3600),
        "origin": None if body.get("origin") is None else _point(body["origin"], "origin"),
        "resolution": _number(body.get("resolution", DEFAULT_RESOLUTION), "resolution",
                              MIN_RESOLUTION, MAX_RESOLUTION),
        "close_m": _number(body.get("close_m", planraster.CLOSE_M), "close_m", 1e-6,
                           MAX_CLOSE_M)}


def _interior_warnings(preview):
    if preview["interior_fraction"] >= INTERIOR_WARN_FRACTION:
        return []
    return [f"little enclosed interior ({preview['interior_fraction'] * 100:.0f}% of the "
            "walls' bounding box is free): the walls do not close, or an opening is at "
            "least close_m wide (raise close_m)"]


# --- the importer ---------------------------------------------------------------------------

class Importer:
    """Job registry and the operations behind the HTTP routes."""

    def __init__(self, fleet_status_url=None, *, max_jobs=MAX_JOBS, ttl=JOB_TTL_S,
                 upstream_timeout=UPSTREAM_TIMEOUT_S, clock=time.monotonic, version=VERSION,
                 ifc_max_bytes=MAX_IFC_BYTES, ifc_threads=IFC_THREADS, ifc_face_budget=None,
                 ifc_grid_budget=IFC_GRID_BUDGET, memory_limit_mb=None, rss_reader=rss_mb,
                 recycle=_schedule_exit):
        self.fleet_status_url = _check_upstream(fleet_status_url)
        self.memory_limit_mb = memory_limit_mb
        self.rss_reader = rss_reader
        self.recycle = recycle
        self.ifc_max_bytes = ifc_max_bytes
        self.ifc_threads = ifc_threads
        self.ifc_face_budget = ifc_face_budget
        self.ifc_grid_budget = ifc_grid_budget
        self.max_jobs = max_jobs
        self.ttl = ttl
        self.upstream_timeout = upstream_timeout
        self.clock = clock
        self.version = version
        self.heavy = threading.Lock()
        self._registry = threading.Lock()
        self.jobs = OrderedDict()
        self._opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), _NoRedirect)

    # slots and registry
    @contextlib.contextmanager
    def heavy_slot(self):
        if not self.heavy.acquire(blocking=False):
            raise ApiError(429, "busy", {"Retry-After": str(RETRY_AFTER_S)})
        try:
            yield
        finally:
            self.heavy.release()

    @contextlib.contextmanager
    def job_slot(self, job):
        if not job.lock.acquire(blocking=False):
            raise ApiError(429, "busy", {"Retry-After": str(RETRY_AFTER_S)})
        try:
            yield
        finally:
            job.lock.release()

    def _drop(self, job):
        self.jobs.pop(job.id, None)
        self._release(job)

    @staticmethod
    def _release(job):
        """Free what a job holds: the PDF document, or the open IFC model and its grids."""
        if job.kind == "pdf":
            pdfsource.close_pdf(job.pdf)
            return
        job.model = None
        job.storeys.clear()
        gc.collect()
        _trim_heap()

    def sweep(self):
        with self._registry:
            for job in list(self.jobs.values()):
                if self.clock() - job.touched > self.ttl and job.lock.acquire(blocking=False):
                    try:
                        self._drop(job)
                    finally:
                        job.lock.release()

    def check_idle(self):
        """Cheap early refusal (before a body is read) while a heavy operation is running."""
        if self.heavy.locked():
            raise ApiError(429, "busy", {"Retry-After": str(RETRY_AFTER_S)})

    def check_capacity(self):
        with self._registry:
            full = len(self.jobs) >= self.max_jobs
            if full and all(j.lock.locked() for j in self.jobs.values()):
                raise ApiError(429, "too many active imports; finish or delete one",
                               {"Retry-After": str(RETRY_AFTER_S)})

    def _too_many(self):
        return ApiError(429, "too many active imports; finish or delete one",
                        {"Retry-After": str(RETRY_AFTER_S)})

    def _evict(self, job):
        """Drop an idle job (registry lock held by the caller); 429 if it is mid-operation."""
        if not job.lock.acquire(blocking=False):
            raise self._too_many()
        try:
            self._drop(job)
        finally:
            job.lock.release()

    def reserve(self, kind):
        """Make room before the big body is read and parsed. An IFC model is ~10x its file
        in memory (see MAX_IFC_BYTES), so an upload evicts the open IFC model and enough idle
        jobs to leave a free slot up-front (the budget has no room for two models); this
        happens even if the new file then fails to parse. PDF uploads evict after a good parse."""
        if kind == "pdf":
            return self.check_capacity()
        with self._registry:
            for old in [j for j in self.jobs.values() if j.kind == "ifc"]:
                self._evict(old)
            self._make_room()

    def _make_room(self):
        """Evict the oldest idle jobs until one slot is free (registry lock held)."""
        while len(self.jobs) >= self.max_jobs:
            idle = [j for j in self.jobs.values() if not j.lock.locked()]
            if not idle:
                raise self._too_many()
            self._evict(min(idle, key=lambda j: j.touched))

    def _add(self, job):
        """Register a job, evicting the oldest idle ones to stay within the caps."""
        with self._registry:
            try:
                if job.kind == "ifc":
                    for old in [j for j in self.jobs.values() if j.kind == "ifc"]:
                        self._evict(old)
                self._make_room()
            except ApiError:
                self._release(job)
                raise
            self.jobs[job.id] = job

    def get_job(self, job_id):
        with self._registry:
            job = self.jobs.get(job_id)
            if job is None:
                raise ApiError(404, "no such import (unknown or expired)")
            job.touched = self.clock()
            return job

    def delete_job(self, job_id):
        job = self.get_job(job_id)
        with self.job_slot(job), self._registry:
            self._drop(job)
        return {"ok": True, "id": job_id}

    def health(self):
        return {"ok": True, "version": self.version, "fleet_status_url": self.fleet_status_url,
                "jobs": len(self.jobs), "busy": self.heavy.locked(), "ifc": ifcjob is not None,
                "rss_mb": rss_mb()}

    # upload
    def max_upload_bytes(self):
        """The largest body any kind may carry; anything bigger is 413 before a byte is read."""
        return max(pdfsource.MAX_PDF_BYTES, self.ifc_max_bytes if ifcjob else 0)

    def memory_gate(self, kind, length):
        """Refuse an IFC that cannot fit on top of the current resident size (see the CAVEAT
        by IFC_PEAK_FACTOR). Idle jobs are dropped and the process recycles itself; if a job
        is mid-operation it is not killed, the caller is told to retry."""
        if kind != "ifc" or not self.memory_limit_mb:
            return
        rss = self.rss_reader()
        if rss is None:
            return
        need = rss + IFC_PEAK_FACTOR * length / (1024 * 1024)
        if need <= self.memory_limit_mb * MEMORY_HEADROOM:
            return
        retry = {"Retry-After": "15"}
        detail = {"rss_mb": rss, "needed_mb": round(need, 1), "limit_mb": self.memory_limit_mb}
        with self._registry:
            if any(j.lock.locked() for j in self.jobs.values()):
                raise ApiError(429, "not enough free memory for this IFC while another import "
                                    "is busy; retry shortly", retry, detail)
            for job in list(self.jobs.values()):
                self._drop(job)
        self.recycle()
        raise ApiError(503, "the importer is restarting to reclaim memory held from earlier "
                            "IFC imports; every open import was dropped; retry in ~15 s",
                       retry, detail)

    def check_size(self, kind, length):
        limit = pdfsource.MAX_PDF_BYTES if kind == "pdf" else self.ifc_max_bytes
        if length > limit:
            extra = {"count": length, "limit": limit} if kind == "ifc" else {}
            raise ApiError(413, f"body is {length} B (max {limit} B)", extra=extra)

    def upload(self, kind, source, filename):
        """Parse an upload into a registered job (the caller holds the heavy slot). `source`
        is the bytes of a PDF or an IfcUpload."""
        job = OPENERS[kind](self, source, filename)
        self._add(job)
        return job

    @staticmethod
    def sniff(head):
        """The kind of an upload from its first bytes."""
        if pdfsource.PDF_HEADER in head[:pdfsource.HEADER_WINDOW]:
            return "pdf"
        if IFC_MAGIC in head[:HEADER_SNIFF]:
            if ifcjob is None:
                raise ApiError(415, "IFC import is not available on this importer "
                                    "(ifcopenshell / shapely are not installed)")
            return "ifc"
        raise ApiError(415, "unsupported file: not a PDF or IFC")

    def _open_pdf(self, data, filename):
        pdf = pdfsource.open_pdf(data)
        try:
            pages = pdfsource.list_pages(pdf)
            job = Job(id=secrets.token_urlsafe(16), kind="pdf", filename=safe_filename(filename),
                      sha256=hashlib.sha256(data).hexdigest(), n_bytes=len(data), pdf=pdf,
                      pages=pages, created=self.clock(), touched=self.clock())
            if len(pages) <= EAGER_SCALE_PAGES:
                for info in pages:
                    self._scale(job, info.index)
        except BaseException:
            pdfsource.close_pdf(pdf)
            raise
        return job

    def _open_ifc(self, upload, filename):
        """Parse an IFC from its temp file (ifcopenshell.open is far leaner than parsing text);
        the model is kept, the file is deleted by the caller, only sha256 and size remain."""
        model, storeys = ifcjob.open_model(upload.path, self.ifc_max_bytes)
        return Job(id=secrets.token_urlsafe(16), kind="ifc",
                   filename=safe_filename(filename, "upload.ifc"), sha256=upload.sha256,
                   n_bytes=upload.size, created=self.clock(), touched=self.clock(),
                   model=model, storeys=storeys)

    # pages
    def _scale(self, job, page):
        if page not in job.scale:
            job.scale[page] = pdfsource.propose_page_scale(job.pdf, page)
        return job.scale[page]

    @staticmethod
    def _scale_json(report):
        if report is None:
            return None
        prop = report.proposal
        return {"proposal": {"m_per_unit": prop.m_per_point, "m_per_point": prop.m_per_point,
                             "evidence": list(prop.evidence), "confidence": prop.confidence},
                "calibration_required": report.calibration_required, "raster": report.raster}

    def _page_json(self, job, index):
        out = _clean(job.pages[index])
        out["scale"] = self._scale_json(job.scale.get(index))
        return out

    def _png(self, job, page, px_per_pt):
        key = (page, round(px_per_pt, 3))
        if key not in job.pngs:
            job.pngs[key] = pdfsource.render_page_png(job.pdf, page, px_per_pt=px_per_pt)
            while len(job.pngs) > PNG_CACHE:
                job.pngs.popitem(last=False)
        return job.pngs[key]

    def _storeys_json(self, job):
        return [ifcjob.storey_json(state) for state in job.storeys.values()]

    def job_json(self, job):
        head = {"id": job.id, "kind": job.kind, "filename": job.filename, "sha256": job.sha256,
                "bytes": job.n_bytes,
                "expires_in": max(0, round(self.ttl - (self.clock() - job.touched)))}
        if job.kind == "ifc":
            return _clean({**head, "pages": [], "storeys": self._storeys_json(job)})
        return _clean({
            **head, "pages": [self._page_json(job, p.index) for p in job.pages],
            "storeys": [], "selected_page": job.selected_page, "calibration": job.calibration,
            "extraction": job.extraction, "preview": job.preview,
            "last_install": job.last_install})

    def upload_json(self, job):
        out = {"id": job.id, "kind": job.kind, "filename": job.filename, "sha256": job.sha256,
               "bytes": job.n_bytes, "pages": [], "storeys": []}
        if job.kind == "ifc":
            out["storeys"] = _clean(self._storeys_json(job))
        else:
            out["pages"] = [self._page_json(job, p.index) for p in job.pages]
        return out

    def page_json(self, job, page, px_per_pt):
        with self.heavy_slot(), self.job_slot(job):
            self._check_page(job, page)
            self._scale(job, page)
            png = self._png(job, page, px_per_pt)
        return _clean({
            "index": page, "page": self._page_json(job, page),
            "scale": self._scale_json(job.scale.get(page)),
            "png": {"url": f"/api/import/{job.id}/page/{page}.png", "width_px": png.width_px,
                    "height_px": png.height_px, "scale_x": png.scale_x, "scale_y": png.scale_y,
                    "width": png.width, "height": png.height}})

    def page_png(self, job, page, px_per_pt):
        with self.heavy_slot(), self.job_slot(job):
            self._check_page(job, page)
            png = self._png(job, page, px_per_pt)
        return Raw(200, png.png, "image/png", {
            "X-Scale-X": repr(png.scale_x), "X-Scale-Y": repr(png.scale_y),
            "X-Page-Width": repr(png.width), "X-Page-Height": repr(png.height)})

    @staticmethod
    def _check_page(job, page):
        if not 0 <= page < len(job.pages):
            raise ApiError(404, f"page must be 0..{len(job.pages) - 1}")

    # calibrate
    def calibrate(self, job, body):
        req = _calibration_request(body, job)
        with self.heavy_slot(), self.job_slot(job):
            page = req["page"]
            report = self._scale(job, page)
            m, detail, warnings = self._scale_for(req, report, job.pages[page])
            ext, raster = self._extract(job, page, m, req, report)
            if raster:
                warnings.append("raster extraction: every dark pixel is a wall; use "
                                "raster_options.region / min_thickness_m to exclude clutter")
            origin = req["origin"] or self._default_origin(ext)
            tf = calib.map_frame(m, math.radians(req["rotation_deg"]), origin)
            grid, meta = planraster.rasterize(ext.geometry, tf, req["resolution"],
                                              close_m=req["close_m"])
            preview = self._preview_summary(grid, meta)
            warnings.extend(_interior_warnings(preview))
            job.grid, job.meta, job.preview, job.selected_page = grid, meta, preview, page
            job.extraction = {"kind": ext.kind, "reason": ext.reason,
                              "diagnostics": pdfsource.diagnostics_dict(ext)}
            job.calibration = {
                "page": page, "method": req["method"], "m_per_unit": m,
                "rotation_deg": req["rotation_deg"], "origin": list(origin),
                "resolution": req["resolution"], "close_m": req["close_m"], "detail": detail}
            return _clean({**job.calibration, "extraction": job.extraction,
                           "preview": preview, "warnings": warnings})

    @staticmethod
    def _scale_for(req, report, info):
        method = req["method"]
        if method == "proposal":
            return _proposal_scale(report)
        if method == "two_point":
            return _two_point_scale(req)
        if method == "typed":
            return _typed_scale(req, info)
        return _robot_scale(req)

    @staticmethod
    def _extract(job, page, m, req, report):
        if req["force_raster"] or report.raster:
            ext = pdfsource.extract_raster_walls(job.pdf, page, m, req["resolution"],
                                                 req["raster_options"])
            raster = True
        else:
            ext = pdfsource.extract_walls(job.pdf, page, req["wall_options"])
            raster = False
        if ext.kind == "none":
            hint = ("" if raster else "; retry with force_raster=true, or adjust wall_options "
                    "(mode, min_stroke_width, wall_layers, region)")
            raise ApiError(422, f"no wall geometry found: {ext.reason}{hint}")
        return ext, raster

    @staticmethod
    def _default_origin(ext):
        bbox = ext.diagnostics.bbox
        if bbox is None:
            raise ApiError(422, "extracted geometry has no extent")
        return (bbox[0], bbox[1])

    @staticmethod
    def _preview_summary(grid, meta):
        occ = grid == planraster.OCCUPIED
        rows, cols = np.nonzero(occ.any(axis=1))[0], np.nonzero(occ.any(axis=0))[0]
        res = meta["resolution"]
        free = int((grid == planraster.FREE).sum())
        box = (rows[-1] - rows[0] + 1) * (cols[-1] - cols[0] + 1)
        return {"width": int(grid.shape[1]), "height": int(grid.shape[0]), "resolution": res,
                "origin": list(meta["origin"]),
                "cells": {"occupied": int(occ.sum()), "free": free,
                          "unknown": int((grid == planraster.UNKNOWN).sum())},
                "interior_fraction": float(free / box),
                "occupied_extent_m": {"width": float((cols[-1] - cols[0] + 1) * res),
                                      "height": float((rows[-1] - rows[0] + 1) * res)}}

    # storeys (IFC)
    @staticmethod
    def storey_state(job, storey_id):
        state = job.storeys.get(storey_id)
        if state is None:
            raise ApiError(404, f"no storey with id {storey_id} in this model")
        return state

    def slice_storey(self, job, state, body):
        """Slice one storey and cache its grid, replacing any earlier one (no calibration:
        IFC is metric, so the scale is 1)."""
        req = _slice_request(body)
        with self.heavy_slot(), self.job_slot(job):
            result, origin, grid, meta = ifcjob.run_slice(
                job.model, state.storey, req, threads=self.ifc_threads,
                face_budget=self.ifc_face_budget)
            preview = self._preview_summary(grid, meta)
            warnings = _interior_warnings(preview)
            job.slice_seq += 1
            state.grid, state.meta, state.preview, state.seq = grid, meta, preview, job.slice_seq
            dropped = ifcjob.trim_grids(job.storeys, self.ifc_grid_budget, keep=state)
            if dropped:
                warnings.append(f"previews of {dropped} were dropped to stay within the "
                                "memory budget; slice them again before installing")
            state.slice = ifcjob.slice_json(result, req, origin, preview, warnings)
            return _clean(state.slice)

    # preview
    @staticmethod
    def _need_grid(target, hint="calibrate first"):
        if target.grid is None:
            raise ApiError(409, f"no preview yet: {hint}")

    def preview_pgm(self, job, target=None, hint="calibrate first"):
        target = target or job
        with self.job_slot(job):
            self._need_grid(target, hint)
            return Raw(200, planraster.encode_pgm(target.grid), "image/x-portable-graymap")

    def preview_png(self, job, target=None, hint="calibrate first"):
        target = target or job
        with self.job_slot(job):
            self._need_grid(target, hint)
            return Raw(200, _png_of_grid(target.grid), "image/png")

    # install
    def install(self, job, body):
        return self._install(job, job, body, lambda: self._sidecar(job), {})

    def install_storey(self, job, state, body):
        """label and floor default to the storey's own when the body omits them."""
        storey = state.storey
        return self._install(
            job, state, body,
            lambda: ifcjob.sidecar(self.version, job, state, _now_utc()),
            {"label": storey.label, "floor": storey.floor}, "slice this storey first")

    def _install(self, job, target, body, make_sidecar, defaults, hint="calibrate first"):
        req = {**defaults, **self._install_request(body)}
        if not self.fleet_status_url:
            raise ApiError(503, "FLEET_STATUS_URL is not configured on this importer")
        with self.job_slot(job):
            self._need_grid(target, hint)
            payload = self._install_payload(target, req, make_sidecar())
            try:
                code, out = self._post_upstream(payload)
            except ApiError as e:
                target.last_install = {"status": e.code, "at": _now_utc(),
                                       "response": {"error": e.message, **e.extra}}
                raise
            target.last_install = {"status": code, "at": _now_utc(), "response": out}
        return code, out

    @staticmethod
    def _install_request(body):
        _strict_object(body, INSTALL_FIELDS, "install body")
        name = body.get("name")
        if not isinstance(name, str) or not name:
            raise ApiError(400, "name is required (a short lowercase map name)")
        out = {"name": name}
        if "label" in body:
            if not isinstance(body["label"], str):
                raise ApiError(400, "label must be a string")
            out["label"] = body["label"]
        if "floor" in body:
            floor = body["floor"]
            if floor is not None and (isinstance(floor, bool) or not isinstance(floor, int)):
                raise ApiError(400, "floor must be an integer or null")
            out["floor"] = floor
        if "map_start_pose" in body:
            pose = body["map_start_pose"]
            if not isinstance(pose, (list, tuple)) or len(pose) != 3:
                raise ApiError(400, "map_start_pose must be [x, y, theta]")
            out["map_start_pose"] = [_number(v, "map_start_pose") for v in pose]
        return out

    def _install_payload(self, target, req, sidecar):
        pgm = planraster.encode_pgm(target.grid)
        payload = {"name": req["name"], "yaml": planraster.yaml_text(target.meta),
                   "pgm_b64": base64.b64encode(pgm).decode("ascii"),
                   "sidecar": sidecar, "source": "drawing"}
        payload.update({k: v for k, v in req.items() if k != "name"})
        return payload

    def _sidecar(self, job):
        diag = dict((job.extraction or {}).get("diagnostics") or {})
        diag["stroke_classes"] = diag.get("stroke_classes", [])[:SIDECAR_CLASSES]
        diag["layers"] = diag.get("layers", [])[:SIDECAR_LAYERS]
        side = {"importer_version": self.version, "kind": job.kind,
                "source_file": {"name": job.filename, "sha256": job.sha256, "bytes": job.n_bytes},
                "page": job.selected_page, "calibration": job.calibration,
                "extraction": {"kind": (job.extraction or {}).get("kind"), "diagnostics": diag},
                "created_utc": _now_utc()}
        side = _clean(side)
        if len(json.dumps(side)) > SIDECAR_BUDGET:
            side["extraction"]["diagnostics"] = {}
        return side

    def _post_upstream(self, payload):
        url = self.fleet_status_url + "/api/maps/install"
        request = urllib.request.Request(
            url, data=json.dumps(payload).encode(), method="POST",
            headers={"Content-Type": "application/json"})
        try:
            with self._opener.open(request, timeout=self.upstream_timeout) as resp:
                code, raw = resp.status, resp.read(UPSTREAM_MAX_BYTES)
        except urllib.error.HTTPError as e:
            code, raw = e.code, e.read(UPSTREAM_MAX_BYTES)
        except (urllib.error.URLError, http.client.HTTPException, OSError, ValueError) as e:
            reason = getattr(e, "reason", e)
            raise ApiError(502, f"fleet_status unreachable: {reason}",
                           extra={"upstream": url}) from e
        try:
            out = json.loads(raw)
        except ValueError as e:
            raise ApiError(502, f"fleet_status answered {code} with a non-JSON body",
                           extra={"upstream": url}) from e
        return code, out


OPENERS = {"pdf": Importer._open_pdf, "ifc": Importer._open_ifc}


# --- HTTP ------------------------------------------------------------------------------------

class ImporterServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, importer, cors_origin="*"):
        self.importer = importer
        self.cors_origin = cors_origin
        super().__init__(address, Handler)

    def server_bind(self):
        socketserver.TCPServer.server_bind(self)
        self.server_port = self.socket.getsockname()[1]

    def get_request(self):
        conn, addr = super().get_request()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return conn, addr


def _parse_px(query):
    raw = query.get("px_per_pt", [None])[-1]
    if raw is None:
        return pdfsource.DEFAULT_PNG_PX_PER_PT
    try:
        return _number(float(raw), "px_per_pt", MIN_PX_PER_PT, MAX_PX_PER_PT)
    except ValueError as e:
        raise ApiError(400, "px_per_pt must be a number") from e


class Handler(BaseHTTPRequestHandler):
    timeout = 60

    def log_message(self, fmt, *args):
        pass

    # responses
    def _headers(self, headers=None):
        origin = self.server.cors_origin
        self.send_header("Access-Control-Allow-Origin", origin)
        self.send_header("Access-Control-Expose-Headers",
                         "Retry-After, X-Scale-X, X-Scale-Y, X-Page-Width, X-Page-Height")
        if origin != "*":
            self.send_header("Vary", "Origin")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        for key, value in (headers or {}).items():
            self.send_header(key, value)

    def _send(self, code, data, content_type, headers=None):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self._headers(headers)
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, code, payload, headers=None):
        self._send(code, json.dumps(_clean(payload)).encode(), "application/json", headers)

    def send_error(self, code, message=None, explain=None):
        """Malformed requests and unsupported methods answer JSON like every other error."""
        self.close_connection = True
        with contextlib.suppress(OSError):
            self._send_json(code, {"error": message or HTTPStatus(code).phrase})

    def do_OPTIONS(self):
        self.send_response(204)
        self._headers({"Access-Control-Allow-Methods": "GET, POST, DELETE, OPTIONS",
                       "Access-Control-Allow-Headers": "Content-Type",
                       "Access-Control-Max-Age": "600"})
        self.end_headers()

    # body helpers
    def _length(self, limit):
        header = self.headers.get("Content-Length")
        if header is None:
            raise ApiError(411, "Content-Length required")
        if not header.isdigit():
            raise ApiError(400, "invalid Content-Length")
        length = int(header)
        if length > limit:
            raise ApiError(413, f"body is {length} B (max {limit} B)")
        return length

    def _read(self, length):
        data = self.rfile.read(length)
        if len(data) != length:
            raise ApiError(400, "body shorter than Content-Length")
        return data

    def _json_body(self):
        raw = self._read(self._length(MAX_JSON_BYTES))
        try:
            return json.loads(raw or b"{}")
        except ValueError as e:
            raise ApiError(400, "body is not valid JSON") from e

    # routing
    def do_GET(self):
        self._dispatch("GET")

    def do_POST(self):
        self._dispatch("POST")

    def do_DELETE(self):
        self._dispatch("DELETE")

    def _dispatch(self, method):
        try:
            parts = urllib.parse.urlsplit(self.path)
            query = urllib.parse.parse_qs(parts.query)
            imp = self.server.importer
            imp.sweep()
            result = self._route(imp, method, parts.path, query)
            if isinstance(result, Raw):
                self._send(result.code, result.data, result.content_type, result.headers)
            else:
                self._send_json(*result)
        except ApiError as e:
            self._send_json(e.code, {"error": e.message, **e.extra}, e.headers)
        except pdfsource.PdfSourceError as e:
            self._send_json(CODE_STATUS.get(e.code, 422), {"error": str(e), "code": e.code})
        except calib.CalibrationError as e:
            self._send_json(400, {"error": str(e)})
        except planraster.PlanRasterError as e:
            self._send_json(422, {"error": str(e)})
        except IFC_ERRORS as e:
            self._send_json(*ifcjob.error_response(e))
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass
        except Exception:  # noqa: BLE001 - last resort: a JSON 500, details on stderr only
            traceback.print_exc(file=sys.stderr)
            with contextlib.suppress(OSError):
                self._send_json(500, {"error": "internal error"})

    def _route(self, imp, method, path, query):
        for verb, pattern, fn in ROUTES:
            m = pattern.fullmatch(path)
            if m and verb == method:
                return fn(self, imp, m, query)
        if any(p.fullmatch(path) for _, p, _ in ROUTES):
            raise ApiError(405, "method not allowed")
        raise ApiError(404, "not found")


IFC_ERRORS = (ifcjob.ifcsource.IfcSourceError,) if ifcjob else ()


# --- route functions -------------------------------------------------------------------------

def _health(h, imp, m, query):
    return 200, imp.health()


@contextlib.contextmanager
def _body_source(h, kind, head, length):
    """The rest of the body as what the kind's opener wants. A PDF is read into memory. An
    IFC is streamed in chunks to a private temp dir (hashing as it goes) and parsed from the
    file: the model is ~6x the file when opened from a path against ~10x from text, and no
    copy of the upload sits in memory. The directory is removed when the block ends."""
    if kind == "pdf":
        yield head + h._read(length - len(head)) if length > len(head) else head
        return
    digest = hashlib.sha256(head)
    with tempfile.TemporaryDirectory(prefix="importer-ifc-") as tmp:
        path = os.path.join(tmp, "model.ifc")
        left = length - len(head)
        with open(path, "wb") as fh:
            fh.write(head)
            while left > 0:
                chunk = h._read(min(left, UPLOAD_CHUNK))
                fh.write(chunk)
                digest.update(chunk)
                left -= len(chunk)
        yield IfcUpload(path, digest.hexdigest(), length)


def _upload(h, imp, m, query):
    """Order matters: sizes and capacity are refused before the big body is read. Only the
    first HEADER_SNIFF bytes are read to learn the kind; a too-large PDF or IFC is 413
    without reading the rest."""
    length = h._length(imp.max_upload_bytes())
    imp.check_idle()
    head = h._read(min(length, HEADER_SNIFF))
    kind = imp.sniff(head)
    imp.check_size(kind, length)
    imp.memory_gate(kind, length)
    imp.reserve(kind)
    with _body_source(h, kind, head, length) as source, imp.heavy_slot():
        job = imp.upload(kind, source, query.get("filename", [""])[-1])
    return 201, imp.upload_json(job)


def _of_kind(imp, m, kind):
    """The job, or 409 naming the routes of its actual kind."""
    job = imp.get_job(m["id"])
    if job.kind != kind:
        raise ApiError(409, KIND_HINTS[job.kind].format(id=job.id))
    return job


def _job(h, imp, m, query):
    return 200, imp.job_json(imp.get_job(m["id"]))


def _delete(h, imp, m, query):
    return 200, imp.delete_job(m["id"])


def _page(h, imp, m, query):
    return 200, imp.page_json(_of_kind(imp, m, "pdf"), int(m["n"]), _parse_px(query))


def _page_png(h, imp, m, query):
    return imp.page_png(_of_kind(imp, m, "pdf"), int(m["n"]), _parse_px(query))


def _calibrate(h, imp, m, query):
    job = _of_kind(imp, m, "pdf")
    return 200, imp.calibrate(job, h._json_body())


def _preview_png(h, imp, m, query):
    return imp.preview_png(_of_kind(imp, m, "pdf"))


def _preview_pgm(h, imp, m, query):
    return imp.preview_pgm(_of_kind(imp, m, "pdf"))


def _install(h, imp, m, query):
    job = _of_kind(imp, m, "pdf")
    return imp.install(job, h._json_body())


def _storey(imp, m):
    job = _of_kind(imp, m, "ifc")
    return job, imp.storey_state(job, int(m["sid"]))


def _slice(h, imp, m, query):
    job, state = _storey(imp, m)
    return 200, imp.slice_storey(job, state, h._json_body())


def _storey_png(h, imp, m, query):
    job, state = _storey(imp, m)
    return imp.preview_png(job, state, "slice this storey first")


def _storey_pgm(h, imp, m, query):
    job, state = _storey(imp, m)
    return imp.preview_pgm(job, state, "slice this storey first")


def _storey_install(h, imp, m, query):
    job, state = _storey(imp, m)
    return imp.install_storey(job, state, h._json_body())


def _route_table():
    jid = rf"/api/import/(?P<id>{JOB_ID_RE})"
    rows = [
        ("GET", r"/api/health", _health),
        ("POST", r"/api/import", _upload),
        ("GET", jid, _job),
        ("DELETE", jid, _delete),
        ("GET", jid + r"/page/(?P<n>\d{1,4})", _page),
        ("GET", jid + r"/page/(?P<n>\d{1,4})\.png", _page_png),
        ("POST", jid + "/calibrate", _calibrate),
        ("GET", jid + r"/preview\.png", _preview_png),
        ("GET", jid + r"/preview\.pgm", _preview_pgm),
        ("POST", jid + "/install", _install),
        ("POST", jid + r"/storey/(?P<sid>\d{1,12})/slice", _slice),
        ("GET", jid + r"/storey/(?P<sid>\d{1,12})/preview\.png", _storey_png),
        ("GET", jid + r"/storey/(?P<sid>\d{1,12})/preview\.pgm", _storey_pgm),
        ("POST", jid + r"/storey/(?P<sid>\d{1,12})/install", _storey_install),
    ]
    return [(verb, re.compile(pattern), fn) for verb, pattern, fn in rows]


ROUTES = _route_table()


def _env_int(env, name, default):
    raw = env.get(name)
    if not raw:
        return default
    try:
        value = int(raw)
        if value <= 0:
            raise ValueError("must be > 0")
    except ValueError as e:
        print(f"WARNING: ignoring {name}={raw!r}: {e}", file=sys.stderr, flush=True)
        return default
    return value


def importer_from_env(env):
    """An unusable FLEET_STATUS_URL (e.g. PI_IP unset) must not crash-loop the container and
    fail the companion deploy's smoke check: start unconfigured, installs answer 503."""
    knobs = {"ifc_max_bytes": _env_int(env, "IMPORTER_MAX_IFC_BYTES", MAX_IFC_BYTES),
             "ifc_threads": _env_int(env, "IMPORTER_IFC_THREADS", IFC_THREADS),
             "ifc_face_budget": _env_int(env, "IMPORTER_IFC_FACE_BUDGET", None),
             "memory_limit_mb": _env_int(env, "IMPORTER_MEMORY_LIMIT_MB", cgroup_limit_mb())}
    try:
        return Importer(env.get("FLEET_STATUS_URL"), **knobs)
    except ValueError as e:
        print(f"WARNING: ignoring FLEET_STATUS_URL {env.get('FLEET_STATUS_URL')!r}: {e}",
              file=sys.stderr, flush=True)
        return Importer(None, **knobs)


def main():
    port = int(os.environ.get("IMPORTER_PORT", DEFAULT_PORT))
    importer = importer_from_env(os.environ)
    server = ImporterServer(("0.0.0.0", port), importer,
                            os.environ.get("IMPORTER_CORS_ORIGIN", "*"))
    print(f"floorplan importer {VERSION} on :{port}, fleet_status "
          f"{importer.fleet_status_url or 'unset'}, IFC "
          f"{'on' if ifcjob else 'off (' + IFC_UNAVAILABLE + ')'}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
