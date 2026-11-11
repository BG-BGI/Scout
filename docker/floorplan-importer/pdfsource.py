"""PDF ingestion for the floorplan importer: PyMuPDF + numpy + calib + planraster.

Pure library, no HTTP and no module state. One PdfDoc per upload; not thread-safe (the HTTP
layer serialises calls). Every failure on hostile input is a PdfSourceError subclass whose
`.code` is a stable string the server can map to a status.

Flow for the HTTP layer (the server stays thin):

    pdf   = open_pdf(data)                                   # -> PdfDoc
    pages = list_pages(pdf)                                  # -> [PageInfo]
    rep   = propose_page_scale(pdf, i)                       # -> ScaleReport
    png   = render_page_png(pdf, i)                          # -> PagePng
    ext   = extract_walls(pdf, i, WallOptions(...))          # -> WallExtraction (vector)
    ...operator accepts rep.proposal or calibrates (calib.two_point / calib.similarity)...
    tf    = calib.map_frame(m_per_unit, rotation, origin)    # origin/rotation in page units
    grid, meta = planraster.rasterize(ext.geometry, tf, resolution, close_m=...)
    pgm, yml   = planraster.encode_pgm(grid), planraster.yaml_text(meta)

    # when rep.raster or ext.kind == "none": calibrate first, then
    ext = extract_raster_walls(pdf, i, m_per_unit, resolution, RasterOptions(...))
    # ext.geometry carries a planraster.Mask and goes through the same rasterize() call.

Exact signatures:
    open_pdf(data: bytes | bytearray | memoryview) -> PdfDoc
    close_pdf(pdf)
    list_pages(pdf, stats_limit=MAX_STATS_PAGES) -> list[PageInfo]
    page_info(pdf, index, stats=True) -> PageInfo
    propose_page_scale(pdf, index) -> ScaleReport
    render_page_png(pdf, index, px_per_pt=1.5, max_pixels=MAX_PNG_PIXELS) -> PagePng
    extract_walls(pdf, index, options: WallOptions | None = None) -> WallExtraction
    extract_raster_walls(pdf, index, m_per_point, resolution,
                         options: RasterOptions | None = None) -> WallExtraction
    diagnostics_dict(extraction) -> dict            # JSON-ready view of extraction.diagnostics

Results (frozen dataclasses; dataclasses.asdict() makes them JSON-ready):
    PageInfo      index, mediabox (x0,y0,x1,y1 as in the file), width, height (visible page,
                  page units), rotation, userunit, n_layers, n_paths / n_images /
                  has_vectors (None past stats_limit)
    ScaleReport   proposal (calib.ScaleProposal), calibration_required (no proposal),
                  raster (page has no usable vector paths: no proposal is offered)
    PagePng       png (bytes), width_px, height_px, scale_x, scale_y (pixels per page unit),
                  width, height (page units)
    WallExtraction  kind "vector" | "raster" | "none", geometry (planraster.Geometry; empty
                  when "none"), diagnostics (WallDiagnostics | RasterDiagnostics), reason
                  (why "none"). "none" on a vector page means nothing wall-like was found:
                  tune WallOptions (mode, min_stroke_width, wall_layers, region) or fall back
                  to extract_raster_walls after calibrating.
    WallDiagnostics  what the classifier did: mode chosen and why, layer_mode, frame and
                  region used, stroke width classes with a kept flag, ink totals, counts of
                  paths kept per kind and dropped per reason (layer, frame, outside_region,
                  no_paint, light, dashed, colored, speck, short, text, thin, not_wall_class).

Errors (all PdfSourceError, a ValueError; `.code` is the stable string):
    NotPdfError not_pdf, CorruptPdfError corrupt_pdf, EncryptedPdfError encrypted_pdf (needs
    a password; owner-password-only files open), EmptyPdfError empty_pdf, PdfTooLargeError
    too_large (> MAX_PDF_BYTES), TooManyPagesError too_many_pages (> MAX_PAGES),
    PageRangeError page_out_of_range, ExtractionError extraction_failed (> MAX_DRAWINGS
    paths), OptionsError bad_options.

Wall classes. A page's walls are either dark filled shapes (poche) or the heaviest dark strokes;
the mode picks whichever carries more ink. Both ignore light/white/coloured paint, dashed
lines, the sheet border, the title-block strip, tiny marks and paths lying under text. Door
gaps wider than planraster's close_m leave the interior unknown: pass close_m to rasterize().

COORDINATE CONVENTIONS (the web UI and the server must use exactly these)
    page units   PDF user-space units (what the file's own coordinates are in; 1/72 in when the
                 page has no /UserUnit). Physical size = units * userunit / 72 inches, so
                 calib.propose_scale(text, (0, 0, width, height), userunit) and every
                 m_per_point / m_per_unit in this API are metres per page unit.
    page frame   the VISIBLE page (after /Rotate and /CropBox): origin at its bottom-left,
                 x to the right, y UP. width/height in PageInfo are in this frame. This is
                 NOT PyMuPDF's native frame (top-left origin, y down, unrotated, scaled by
                 UserUnit); all of that is undone here. Geometry returned by extract_walls
                 is in the page frame and planraster.Polyline widths are in page units.
    png pixels   PagePng is the visible page. Pixel (px, py) has py counted from the top:
                     page_x = px / scale_x        page_y = height - py / scale_y
                 and the inverse px = page_x * scale_x, py = (height - page_y) * scale_y.
    thresholds   WallOptions lengths/areas/widths are physical points (1/72 in, UserUnit
                 applied) so they mean the same thing on every sheet.
    region       WallOptions.region and RasterOptions.region are (x0, y0, x1, y1) in the page
                 frame, page units.
"""

import contextlib
import math
import re
from collections import Counter
from dataclasses import asdict, dataclass

import calib
import numpy as np
import planraster
import pymupdf

MAX_PDF_BYTES = 64 * 1024 * 1024
MAX_PAGES = 500
MAX_STATS_PAGES = 50
MAX_DRAWINGS = 1_000_000
MAX_PNG_PIXELS = 25_000_000
DEFAULT_PNG_PX_PER_PT = 1.5
MAX_RASTER_PIXELS = 36_000_000
RASTER_OVERSAMPLE = 4.0
MAX_UNIT = 1.0e4
PDF_HEADER = b"%PDF-"
HEADER_WINDOW = 1024
MIN_VECTOR_PATHS = 1
RASTER_IMAGE_COVERAGE = 0.5
VECTOR_OVERRIDE_PATHS = 500
WIDTH_STEP = 0.05
JOIN_TOL = 1e-3
TITLE_CUT_MARGIN = 1.0
OPACITY_MIN = 0.5
MODES = ("auto", "fills", "strokes", "both")
DEFAULT_WALL_LAYERS = (r"wall", r"partition", r"\bcmu\b", r"colum", r"struct")
DEFAULT_SKIP_LAYERS = (
    r"dim", r"anno", r"text", r"note", r"hatch", r"patt", r"furn", r"grid", r"title",
    r"border", r"defpoints", r"equip", r"fixt", r"ceil", r"elec", r"mech", r"plumb", r"symbol",
)


class PdfSourceError(ValueError):
    code = "pdf_error"


class NotPdfError(PdfSourceError):
    code = "not_pdf"


class CorruptPdfError(PdfSourceError):
    code = "corrupt_pdf"


class EncryptedPdfError(PdfSourceError):
    code = "encrypted_pdf"


class EmptyPdfError(PdfSourceError):
    code = "empty_pdf"


class PdfTooLargeError(PdfSourceError):
    code = "too_large"


class TooManyPagesError(PdfSourceError):
    code = "too_many_pages"


class PageRangeError(PdfSourceError):
    code = "page_out_of_range"


class ExtractionError(PdfSourceError):
    code = "extraction_failed"


class OptionsError(PdfSourceError):
    code = "bad_options"


@dataclass(eq=False)
class PdfDoc:
    doc: object
    n_bytes: int
    n_pages: int
    n_layers: int
    layer_names: tuple
    hidden_layers: tuple


@dataclass(frozen=True)
class PageInfo:
    index: int
    mediabox: tuple
    width: float
    height: float
    rotation: int
    userunit: float
    n_layers: int
    n_paths: int | None
    n_images: int | None
    has_vectors: bool | None


@dataclass(frozen=True)
class ScaleReport:
    proposal: calib.ScaleProposal
    calibration_required: bool
    raster: bool


@dataclass(frozen=True)
class PagePng:
    png: bytes
    width_px: int
    height_px: int
    scale_x: float
    scale_y: float
    width: float
    height: float


@dataclass(frozen=True)
class WallOptions:
    mode: str = "auto"
    wall_layers: tuple = DEFAULT_WALL_LAYERS
    skip_layers: tuple = DEFAULT_SKIP_LAYERS
    min_stroke_width: float | None = None
    width_floor: float = 0.5
    heavy_ratio: float = 0.6
    min_class_share: float = 0.03
    max_luminance: float = 0.5
    max_saturation: float = 0.3
    min_fill_area: float = 8.0
    min_fill_thickness: float = 1.0
    min_stroke_length: float = 4.0
    mask_text: bool = True
    text_pad: float = 1.0
    text_cover: float = 0.5
    drop_frame: bool = True
    frame_fraction: float = 0.8
    detect_title_block: bool = True
    title_block_span: float = 0.9
    title_block_band: float = 0.4
    region: tuple | None = None
    curve_segments: int = 8


@dataclass(frozen=True)
class RasterOptions:
    oversample: float = RASTER_OVERSAMPLE
    dark_threshold: int = 128
    max_pixels: int = MAX_RASTER_PIXELS
    min_thickness_m: float = 0.0
    margin_fraction: float = 0.03
    region: tuple | None = None


@dataclass(frozen=True)
class WallDiagnostics:
    mode: str
    mode_reason: str
    layer_mode: str
    layers: tuple
    frame: tuple | None
    region: tuple
    width_threshold: float | None
    stroke_classes: tuple
    ink_fill: float
    ink_stroke: float
    total_paths: int
    kept_fills: int
    kept_strokes: int
    dropped: dict
    bbox: tuple | None


@dataclass(frozen=True)
class RasterDiagnostics:
    width_px: int
    height_px: int
    scale: float
    pixel_m: float
    oversample: float
    clamped: bool
    dark_pixels: int
    dark_fraction: float
    opened_px: int
    region: tuple | None
    bbox: tuple | None


@dataclass(frozen=True)
class WallExtraction:
    kind: str
    geometry: planraster.Geometry
    diagnostics: object
    reason: str


# --- open ---------------------------------------------------------------------------------

@contextlib.contextmanager
def _guard(what):
    try:
        yield
    except PdfSourceError:
        raise
    except Exception as e:
        raise CorruptPdfError(f"{what}: {e}") from e


def open_pdf(data):
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise NotPdfError("upload must be bytes")
    data = bytes(data)
    if not data:
        raise NotPdfError("empty upload")
    if len(data) > MAX_PDF_BYTES:
        raise PdfTooLargeError(f"PDF is {len(data)} bytes; limit is {MAX_PDF_BYTES}")
    if PDF_HEADER not in data[:HEADER_WINDOW]:
        raise NotPdfError("not a PDF (no %PDF- header)")
    with _guard("cannot open PDF"):
        doc = pymupdf.open(stream=data, filetype="pdf")
        if doc.needs_pass:
            doc.close()
            raise EncryptedPdfError("PDF needs a password")
        n_pages = doc.page_count
        if n_pages < 1:
            doc.close()
            raise EmptyPdfError("PDF has no pages")
        if n_pages > MAX_PAGES:
            doc.close()
            raise TooManyPagesError(f"PDF has {n_pages} pages; limit is {MAX_PAGES}")
        ocgs = doc.get_ocgs() or {}
    names = tuple(sorted({str(o.get("name", "")) for o in ocgs.values()}))
    hidden = tuple(sorted({str(o.get("name", "")) for o in ocgs.values()
                           if not o.get("on", True)}))
    return PdfDoc(doc, len(data), n_pages, len(ocgs), names, hidden)


def close_pdf(pdf):
    with contextlib.suppress(Exception):
        pdf.doc.close()


# --- page space ---------------------------------------------------------------------------

@dataclass(frozen=True)
class _Space:
    unit: float
    mat: tuple
    height: float
    width: float

    def phys(self, pts):
        """(N, 2) native PyMuPDF points -> visible page, y up, physical points."""
        a, b, c, d, e, f = self.mat
        x = a * pts[:, 0] + c * pts[:, 1] + e
        y = b * pts[:, 0] + d * pts[:, 1] + f
        return np.stack([x, self.height - y], axis=1)

    def box(self, x0, y0, x1, y1):
        p = self.phys(np.array([[x0, y0], [x1, y1]], dtype=float))
        return (p[:, 0].min(), p[:, 1].min(), p[:, 0].max(), p[:, 1].max())


def _page(pdf, index):
    if isinstance(index, bool) or not isinstance(index, int) or not 0 <= index < pdf.n_pages:
        raise PageRangeError(f"page index must be 0..{pdf.n_pages - 1}")
    with _guard(f"cannot load page {index}"):
        return pdf.doc[index]


def _userunit(pdf, page):
    with contextlib.suppress(Exception):
        kind, value = pdf.doc.xref_get_key(page.xref, "UserUnit")
        unit = float(value) if kind in ("int", "real", "float") else 1.0
        if math.isfinite(unit) and 0 < unit <= MAX_UNIT:
            return unit
    return 1.0


def _space(pdf, page):
    """Visible-page transform. page.rotation_matrix is not used: with a /UserUnit its
    translation stays unscaled while the drawing coordinates are scaled."""
    unit = _userunit(pdf, page)
    rect = page.rect
    rotation = int(page.rotation) % 360
    w0, h0 = (rect.height, rect.width) if rotation in (90, 270) else (rect.width, rect.height)
    mats = {
        0: (1, 0, 0, 1, 0, 0),
        90: (0, 1, -1, 0, h0, 0),
        180: (-1, 0, 0, -1, w0, h0),
        270: (0, -1, 1, 0, 0, w0),
    }
    return _Space(unit, mats.get(rotation, mats[0]), rect.height, rect.width)


# --- pages --------------------------------------------------------------------------------

def page_info(pdf, index, stats=True):
    page = _page(pdf, index)
    sp = _space(pdf, page)
    with _guard(f"cannot read page {index}"):
        mb = page.mediabox
        n_paths = n_images = has_vectors = None
        if stats:
            n_paths = len(page.get_cdrawings())
            infos = page.get_image_info()
            n_images = len(infos)
            has_vectors = _has_vectors(n_paths, infos, sp)
    return PageInfo(
        index=index, mediabox=(mb.x0, mb.y0, mb.x1, mb.y1), width=sp.width / sp.unit,
        height=sp.height / sp.unit, rotation=int(page.rotation), userunit=sp.unit,
        n_layers=pdf.n_layers, n_paths=n_paths, n_images=n_images, has_vectors=has_vectors)


def _has_vectors(n_paths, infos, sp):
    covered = 0.0
    for info in infos:
        x0, y0, x1, y1 = info["bbox"]
        covered += max(0.0, x1 - x0) * max(0.0, y1 - y0)
    coverage = covered / max(sp.width * sp.height, 1.0)
    if n_paths >= VECTOR_OVERRIDE_PATHS:
        return True
    return n_paths >= MIN_VECTOR_PATHS and coverage < RASTER_IMAGE_COVERAGE


def list_pages(pdf, stats_limit=MAX_STATS_PAGES):
    """Every page; path/image statistics only for the first `stats_limit` (None = unknown)."""
    return [page_info(pdf, i, stats=i < stats_limit) for i in range(pdf.n_pages)]


def propose_page_scale(pdf, index):
    info = page_info(pdf, index)
    if not info.has_vectors:
        proposal = calib.ScaleProposal(
            None, ("raster page: no vector geometry, calibrate manually",), calib.NONE)
        return ScaleReport(proposal, True, True)
    page = _page(pdf, index)
    with _guard(f"cannot read text of page {index}"):
        blobs = [b[4] for b in page.get_text("blocks") if len(b) > 6 and b[6] == 0]
    proposal = calib.propose_scale(blobs, (0, 0, info.width, info.height), info.userunit)
    return ScaleReport(proposal, proposal.m_per_point is None, False)


def render_page_png(pdf, index, px_per_pt=DEFAULT_PNG_PX_PER_PT, max_pixels=MAX_PNG_PIXELS):
    """The visible page as PNG; the scale is clamped so width*height <= max_pixels."""
    page = _page(pdf, index)
    sp = _space(pdf, page)
    scale = _clamped_scale(px_per_pt, sp.width, sp.height, max_pixels)
    with _guard(f"cannot render page {index}"):
        pix = page.get_pixmap(matrix=pymupdf.Matrix(scale, scale), alpha=False, annots=False)
        png = pix.tobytes("png")
    w_units, h_units = sp.width / sp.unit, sp.height / sp.unit
    return PagePng(png, pix.width, pix.height, pix.width / w_units, pix.height / h_units,
                   w_units, h_units)


def _clamped_scale(want, width, height, max_pixels):
    want = _positive(want, "px_per_pt")
    pixels = (width * want) * (height * want)
    return want if pixels <= max_pixels else want * math.sqrt(max_pixels / pixels)


# --- wall extraction: collect -------------------------------------------------------------

def _rgb(c):
    if not c:
        return None
    if len(c) == 1:
        return (c[0],) * 3
    if len(c) == 4:
        k = 1.0 - c[3]
        return tuple((1.0 - c[i]) * k for i in range(3))
    return tuple(c[:3])


def _lum_sat(rgb):
    if rgb is None:
        return math.nan, math.nan
    hi, lo = max(rgb), min(rgb)
    return 0.299 * rgb[0] + 0.587 * rgb[1] + 0.114 * rgb[2], ((hi - lo) / hi if hi > 0 else 0.0)


def _is_dashed(d):
    dashes = (d.get("dashes") or "").strip()
    return bool(dashes) and not dashes.startswith("[]")


@dataclass(eq=False)
class _Paths:
    n: int
    bbox: np.ndarray
    has_fill: np.ndarray
    has_stroke: np.ndarray
    width: np.ndarray
    fill_lum: np.ndarray
    fill_sat: np.ndarray
    stroke_lum: np.ndarray
    stroke_sat: np.ndarray
    dashed: np.ndarray
    opaque_fill: np.ndarray
    opaque_stroke: np.ndarray
    layer: list


def _collect(drawings, sp):
    n = len(drawings)
    out = _Paths(n, np.zeros((n, 4)), np.zeros(n, bool), np.zeros(n, bool), np.zeros(n),
                 np.full(n, np.nan), np.full(n, np.nan), np.full(n, np.nan), np.full(n, np.nan),
                 np.zeros(n, bool), np.ones(n, bool), np.ones(n, bool), [""] * n)
    for i, d in enumerate(drawings):
        r = d["rect"]
        out.bbox[i] = sp.box(r.x0, r.y0, r.x1, r.y1)
        fill, stroke = _rgb(d.get("fill")), _rgb(d.get("color"))
        out.has_fill[i], out.has_stroke[i] = fill is not None, stroke is not None
        out.fill_lum[i], out.fill_sat[i] = _lum_sat(fill)
        out.stroke_lum[i], out.stroke_sat[i] = _lum_sat(stroke)
        out.width[i] = float(d.get("width") or 0.0)
        out.dashed[i] = _is_dashed(d)
        out.opaque_fill[i] = (d.get("fill_opacity") or 1.0) >= OPACITY_MIN
        out.opaque_stroke[i] = (d.get("stroke_opacity") or 1.0) >= OPACITY_MIN
        out.layer[i] = d.get("layer") or ""
    return out


def _words(page, sp, pad):
    with _guard("cannot read text"):
        raw = page.get_text("words")
    boxes = [sp.box(w[0], w[1], w[2], w[3]) for w in raw]
    if not boxes:
        return np.zeros((0, 4))
    b = np.array(boxes)
    return b + np.array([-pad, -pad, pad, pad])


# --- wall extraction: geometry from drawing items ------------------------------------------

def _rect_pts(item_rect):
    r = item_rect
    return [(r.x0, r.y0), (r.x1, r.y0), (r.x1, r.y1), (r.x0, r.y1)]


def _bezier(p1, c1, c2, p2, segs):
    t = np.linspace(0.0, 1.0, segs + 1)[:, None]
    pts = ((1 - t) ** 3 * np.array(tuple(p1)) + 3 * (1 - t) ** 2 * t * np.array(tuple(c1))
           + 3 * (1 - t) * t**2 * np.array(tuple(c2)) + t**3 * np.array(tuple(p2)))
    return [tuple(p) for p in pts]


def _runs(d, segs):
    """Connected point runs (native coordinates) of a drawing and whether each is closed."""
    runs, cur = [], None
    for item in d["items"]:
        kind = item[0]
        if kind in ("re", "qu"):
            if cur:
                runs.append((cur, False))
                cur = None
            pts = _rect_pts(item[1]) if kind == "re" else [tuple(p) for p in (
                item[1].ul, item[1].ur, item[1].lr, item[1].ll)]
            runs.append((pts, True))
            continue
        pts = ([tuple(item[1]), tuple(item[2])] if kind == "l"
               else _bezier(item[1], item[2], item[3], item[4], segs) if kind == "c" else None)
        if pts is None:
            continue
        if cur and math.dist(cur[-1], pts[0]) <= JOIN_TOL:
            cur.extend(pts[1:])
        else:
            if cur:
                runs.append((cur, False))
            cur = list(pts)
    if cur:
        runs.append((cur, False))
    return runs


def _rings(d, sp, segs):
    rings = []
    for pts, _ in _runs(d, segs):
        arr = np.array(pts, dtype=float)
        if len(arr) > 1 and math.dist(arr[0], arr[-1]) <= JOIN_TOL:
            arr = arr[:-1]
        if len(arr) >= 3:
            rings.append(sp.phys(arr))
    return rings


def _polylines(d, sp, segs):
    lines = []
    for pts, flag in _runs(d, segs):
        arr = np.array(pts, dtype=float)
        if len(arr) < 2:
            continue
        if (flag or d.get("closePath")) and math.dist(arr[0], arr[-1]) > JOIN_TOL:
            arr = np.vstack([arr, arr[:1]])
        lines.append(sp.phys(arr))
    return lines


def _ring_stats(rings):
    area = perimeter = 0.0
    for r in rings:
        x, y = r[:, 0], r[:, 1]
        xn, yn = np.roll(x, -1), np.roll(y, -1)
        area += abs(float(np.sum(x * yn - xn * y))) / 2
        perimeter += float(np.hypot(xn - x, yn - y).sum())
    return area, (2 * area / perimeter if perimeter > 0 else 0.0)


def _line_length(lines):
    return float(sum(np.hypot(*np.diff(p, axis=0).T).sum() for p in lines))


def _box_of(rings_or_lines):
    pts = np.vstack(rings_or_lines)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    return (lo[0], lo[1], hi[0], hi[1])


# --- wall extraction: classify ------------------------------------------------------------

class _Tally:
    def __init__(self, n):
        self.alive = np.ones(n, bool)
        self.dropped = Counter()

    def drop(self, cond, reason):
        hit = self.alive & cond
        if hit.any():
            self.dropped[reason] += int(hit.sum())
            self.alive = self.alive & ~hit


def _check_options(opts):
    if opts.mode not in MODES:
        raise OptionsError(f"mode must be one of {MODES}")
    for name in ("max_luminance", "max_saturation", "heavy_ratio", "min_class_share",
                 "frame_fraction", "title_block_span", "title_block_band", "width_floor",
                 "min_fill_area", "min_fill_thickness", "min_stroke_length", "text_pad",
                 "text_cover"):
        v = getattr(opts, name)
        if not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0:
            raise OptionsError(f"{name} must be a finite number >= 0")
    if opts.min_stroke_width is not None and not (
            math.isfinite(opts.min_stroke_width) and opts.min_stroke_width >= 0):
        raise OptionsError("min_stroke_width must be None or a finite number >= 0")
    if not isinstance(opts.curve_segments, int) or not 1 <= opts.curve_segments <= 64:
        raise OptionsError("curve_segments must be an integer 1..64")
    _check_region(opts.region)


def _check_region(region):
    if region is None:
        return
    try:
        x0, y0, x1, y1 = (float(v) for v in region)
    except (TypeError, ValueError) as e:
        raise OptionsError("region must be (x0, y0, x1, y1)") from e
    if not all(map(math.isfinite, (x0, y0, x1, y1))) or x1 <= x0 or y1 <= y0:
        raise OptionsError("region must be a finite, non-empty (x0, y0, x1, y1)")


def _layer_ok(paths, opts, pdf):
    """(keep mask, layer_mode, layer names seen) from the OCG layer of every path."""
    skip = [re.compile(p, re.IGNORECASE) for p in opts.skip_layers]
    allow = [re.compile(p, re.IGNORECASE) for p in opts.wall_layers]

    def hit(name, patterns):
        return bool(name) and any(p.search(name) for p in patterns)

    seen = tuple(sorted({n for n in paths.layer if n}))
    is_skip = np.array([hit(n, skip) for n in paths.layer], dtype=bool)
    is_wall = np.array([hit(n, allow) and not hit(n, skip) for n in paths.layer], dtype=bool)
    if pdf.n_layers and is_wall.any():
        return is_wall, "allowlist", seen
    return ~is_skip, ("fallback" if pdf.n_layers else "none"), seen


def _frame_boxes(paths, page_box, opts):
    """Mask of page-sized paths and the innermost stroked one (the sheet frame), if any."""
    pw, ph = page_box[2] - page_box[0], page_box[3] - page_box[1]
    w = paths.bbox[:, 2] - paths.bbox[:, 0]
    h = paths.bbox[:, 3] - paths.bbox[:, 1]
    covers = (w >= opts.frame_fraction * pw) & (h >= opts.frame_fraction * ph)
    stroked = covers & paths.has_stroke
    if not stroked.any():
        return covers, None
    area = np.where(stroked, w * h, np.inf)
    return covers, tuple(float(v) for v in paths.bbox[int(np.argmin(area))])


def _region(paths, frame, page_box, opts, unit):
    if opts.region is not None:
        return tuple(float(v) * unit for v in opts.region)
    if frame is None:
        return page_box
    x0, y0, x1, y1 = frame
    if not opts.detect_title_block:
        return frame
    w, h = x1 - x0, y1 - y0
    pw = paths.bbox[:, 2] - paths.bbox[:, 0]
    ph = paths.bbox[:, 3] - paths.bbox[:, 1]
    usable = paths.has_stroke & ~paths.dashed
    box = paths.bbox
    tall = usable & (ph >= opts.title_block_span * h) & (pw <= opts.title_block_band * w)
    wide = usable & (pw >= opts.title_block_span * w) & (ph <= opts.title_block_band * h)
    right = tall & (box[:, 0] >= x0 + (1 - opts.title_block_band) * w)
    left = tall & (box[:, 2] <= x0 + opts.title_block_band * w)
    top = wide & (box[:, 1] >= y0 + (1 - opts.title_block_band) * h)
    bottom = wide & (box[:, 3] <= y0 + opts.title_block_band * h)
    if right.any():
        x1 = min(x1, float(box[right, 0].min()) - TITLE_CUT_MARGIN)
    if left.any():
        x0 = max(x0, float(box[left, 2].max()) + TITLE_CUT_MARGIN)
    if top.any():
        y1 = min(y1, float(box[top, 1].min()) - TITLE_CUT_MARGIN)
    if bottom.any():
        y0 = max(y0, float(box[bottom, 3].max()) + TITLE_CUT_MARGIN)
    return (x0, y0, x1, y1)


def _inside(centres, box):
    x0, y0, x1, y1 = box
    return ((centres[:, 0] >= x0) & (centres[:, 0] <= x1)
            & (centres[:, 1] >= y0) & (centres[:, 1] <= y1))


def _in_text(box, words, min_cover):
    """True when one word box covers at least `min_cover` of the (slightly grown) path box."""
    if not len(words):
        return False
    x0, y0, x1, y1 = box[0] - 0.5, box[1] - 0.5, box[2] + 0.5, box[3] + 0.5
    ix = np.minimum(words[:, 2], x1) - np.maximum(words[:, 0], x0)
    iy = np.minimum(words[:, 3], y1) - np.maximum(words[:, 1], y0)
    inter = np.clip(ix, 0, None) * np.clip(iy, 0, None)
    return bool((inter >= min_cover * (x1 - x0) * (y1 - y0)).any())


def _paint_masks(paths, opts):
    fill_ok = (paths.has_fill & (paths.fill_lum <= opts.max_luminance)
               & (paths.fill_sat <= opts.max_saturation) & paths.opaque_fill)
    stroke_ok = (paths.has_stroke & (paths.stroke_lum <= opts.max_luminance)
                 & (paths.stroke_sat <= opts.max_saturation) & paths.opaque_stroke
                 & ~paths.dashed)
    return fill_ok, stroke_ok


def _drop_unpaintable(tally, paths, fill_ok, stroke_ok, opts):
    tally.drop(~(paths.has_fill | paths.has_stroke), "no_paint")
    too_light = ((~paths.has_fill | (paths.fill_lum > opts.max_luminance))
                 & (~paths.has_stroke | (paths.stroke_lum > opts.max_luminance)))
    tally.drop(~(fill_ok | stroke_ok) & too_light, "light")
    tally.drop(~(fill_ok | stroke_ok) & paths.dashed & ~paths.has_fill, "dashed")
    tally.drop(~(fill_ok | stroke_ok), "colored")


def _width_classes(widths, lengths):
    classes = {}
    for w, length in zip(widths, lengths, strict=True):
        key = round(round(w / WIDTH_STEP) * WIDTH_STEP, 4)
        count, total = classes.get(key, (0, 0.0))
        classes[key] = (count + 1, total + length)
    return classes


def _heavy_threshold(classes, opts):
    if opts.min_stroke_width is not None:
        return float(opts.min_stroke_width)
    if not classes:
        return None
    total = sum(length for _, length in classes.values())
    floor = opts.min_class_share * total
    significant = [w for w, (_, length) in classes.items() if length >= floor]
    return max(opts.width_floor, opts.heavy_ratio * max(significant or classes))


def _choose_mode(opts, fills, strokes, ink_fill, ink_stroke):
    if opts.mode != "auto":
        return opts.mode, "requested"
    if fills and (not strokes or ink_fill >= ink_stroke):
        return "fills", "dark fills carry at least as much ink as heavy strokes"
    if strokes:
        return "strokes", "heavy strokes carry more ink than dark fills" if fills else \
            "no dark fills; heavy strokes"
    return "none", "no wall-like fills or strokes"


def extract_walls(pdf, index, options=None):
    """Wall geometry of a vector page, in the page frame (y up, page units)."""
    opts = options or WallOptions()
    _check_options(opts)
    page = _page(pdf, index)
    sp = _space(pdf, page)
    with _guard(f"cannot read drawings of page {index}"):
        drawings = page.get_drawings()
    if len(drawings) > MAX_DRAWINGS:
        raise ExtractionError(f"page has {len(drawings)} paths; limit is {MAX_DRAWINGS}")
    paths = _collect(drawings, sp)
    words = _words(page, sp, opts.text_pad) if opts.mask_text else np.zeros((0, 4))
    page_box = (0.0, 0.0, sp.width, sp.height)
    tally = _Tally(paths.n)
    layer_keep, layer_mode, seen = _layer_ok(paths, opts, pdf)
    tally.drop(~layer_keep, "layer")
    covers, frame = _frame_boxes(paths, page_box, opts)
    if opts.drop_frame:
        tally.drop(covers, "frame")
    region = _region(paths, frame, page_box, opts, sp.unit)
    centres = (paths.bbox[:, :2] + paths.bbox[:, 2:]) / 2
    tally.drop(~_inside(centres, region), "outside_region")
    fill_ok, stroke_ok = _paint_masks(paths, opts)
    _drop_unpaintable(tally, paths, fill_ok, stroke_ok, opts)
    cands = _candidates(drawings, paths, tally.alive, fill_ok, stroke_ok, sp, words, opts, tally)
    return _assemble(cands, paths, tally, sp, opts, layer_mode, seen, frame, region)


@dataclass(eq=False)
class _Cands:
    fills: list
    strokes: list


def _candidates(drawings, paths, alive, fill_ok, stroke_ok, sp, words, opts, tally):
    fills, strokes = [], []
    reasons = {}
    for i in np.nonzero(alive)[0]:
        d = drawings[i]
        if fill_ok[i]:
            rings = _rings(d, sp, opts.curve_segments)
            area, thick = _ring_stats(rings) if rings else (0.0, 0.0)
            if not rings or area < opts.min_fill_area or thick < opts.min_fill_thickness:
                reasons[i] = "speck"
            elif opts.mask_text and _in_text(_box_of(rings), words, opts.text_cover):
                reasons[i] = "text"
            else:
                fills.append((int(i), rings, area))
                reasons.pop(i, None)
        if stroke_ok[i]:
            lines = _polylines(d, sp, opts.curve_segments)
            length = _line_length(lines) if lines else 0.0
            if not lines or length < opts.min_stroke_length:
                reasons.setdefault(i, "short")
            elif opts.mask_text and _in_text(_box_of(lines), words, opts.text_cover):
                reasons.setdefault(i, "text")
            else:
                strokes.append((int(i), lines, length, float(paths.width[i])))
                reasons.pop(i, None)
    kept = {f[0] for f in fills} | {s[0] for s in strokes}
    for i, why in reasons.items():
        if i not in kept:
            tally.dropped[why] += 1
    return _Cands(fills, strokes)


def _assemble(cands, paths, tally, sp, opts, layer_mode, seen, frame, region):
    classes = _width_classes([s[3] for s in cands.strokes], [s[2] for s in cands.strokes])
    thr = _heavy_threshold(classes, opts)
    heavy = [s for s in cands.strokes if thr is not None and s[3] >= thr - 1e-9]
    ink_fill = float(sum(f[2] for f in cands.fills))
    ink_stroke = float(sum(s[2] * max(s[3], 0.0) for s in heavy))
    mode, why = _choose_mode(opts, cands.fills, heavy, ink_fill, ink_stroke)
    fills = cands.fills if mode in ("fills", "both") else []
    strokes = heavy if mode in ("strokes", "both") else []
    _count_unused(tally, cands, fills, strokes, heavy, mode)
    geometry = _geometry(fills, strokes, sp)
    diag = WallDiagnostics(
        mode=mode, mode_reason=why, layer_mode=layer_mode, layers=seen, frame=frame,
        region=tuple(float(v) for v in region), width_threshold=thr,
        stroke_classes=tuple(
            {"width": w, "count": c, "length": ln, "kept": thr is not None and w >= thr - 1e-9}
            for w, (c, ln) in sorted(classes.items())),
        ink_fill=ink_fill, ink_stroke=ink_stroke, total_paths=paths.n,
        kept_fills=len(fills), kept_strokes=len(strokes), dropped=dict(tally.dropped),
        bbox=_bbox(geometry))
    empty = not geometry.polylines and not geometry.fills
    return WallExtraction("none" if empty else "vector", geometry, diag,
                          "no wall-like geometry found" if empty else "")


def _count_unused(tally, cands, fills, strokes, heavy, mode):
    """Candidates that survived the filters but are not part of the chosen wall class."""
    kept = {f[0] for f in fills} | {s[0] for s in strokes}
    pool = {f[0] for f in cands.fills} | {s[0] for s in cands.strokes}
    leftover = pool - kept
    heavy_idx = {s[0] for s in heavy}
    thin = set()
    if mode in ("strokes", "both"):
        thin = {s[0] for s in cands.strokes} - heavy_idx - kept
    tally.dropped["thin"] += len(thin & leftover)
    tally.dropped["not_wall_class"] += len(leftover - thin)


def _geometry(fills, strokes, sp):
    unit = sp.unit
    out_fills = tuple(planraster.Fill(rings=tuple(r / unit for r in f[1])) for f in fills)
    lines = []
    for _, polys, _, width in strokes:
        lines.extend(planraster.Polyline(points=p / unit, width=width / unit) for p in polys)
    return planraster.Geometry(polylines=tuple(lines), fills=out_fills)


def _bbox(geometry):
    pts = [p.points for p in geometry.polylines]
    pts += [r for f in geometry.fills for r in f.rings]
    if not pts:
        return None
    allp = np.vstack(pts)
    lo, hi = allp.min(axis=0), allp.max(axis=0)
    return (float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1]))


# --- raster fallback ----------------------------------------------------------------------

def extract_raster_walls(pdf, index, m_per_point, resolution, options=None):
    """Render the page and take dark pixels as walls; needs the calibration (m per page unit)
    because the render DPI is derived from the target grid resolution."""
    opts = options or RasterOptions()
    m_per_point = _positive(m_per_point, "m_per_point")
    resolution = _positive(resolution, "resolution")
    _check_raster_options(opts)
    page = _page(pdf, index)
    sp = _space(pdf, page)
    w_units, h_units = sp.width / sp.unit, sp.height / sp.unit
    pixel_units = resolution / (opts.oversample * m_per_point)
    wanted = 1.0 / (pixel_units * sp.unit)
    scale_mu = _clamped_scale(wanted, sp.width, sp.height, opts.max_pixels)
    with _guard(f"cannot render page {index}"):
        pix = page.get_pixmap(matrix=pymupdf.Matrix(scale_mu, scale_mu), colorspace=pymupdf.csGRAY,
                              alpha=False, annots=False)
        gray = np.frombuffer(pix.samples, dtype=np.uint8).reshape(pix.height, pix.width)
    sx, sy = pix.width / w_units, pix.height / h_units
    dark = _trim_margin(gray < opts.dark_threshold, opts.margin_fraction)
    if opts.region is not None:
        dark = _crop(dark, opts.region, sx, sy, h_units)
    pixel_m = m_per_point / sx
    opened = _open_box(dark, round(opts.min_thickness_m / (2 * pixel_m))) if (
        opts.min_thickness_m > 0) else dark
    mask = planraster.Mask(opened, 0.0, h_units, 1.0 / sx, 1.0 / sy)
    n_dark = int(opened.sum())
    diag = RasterDiagnostics(
        width_px=int(pix.width), height_px=int(pix.height), scale=float(sx),
        pixel_m=float(pixel_m),
        oversample=float(resolution / pixel_m), clamped=bool(scale_mu < 0.999 * wanted),
        dark_pixels=n_dark, dark_fraction=n_dark / max(opened.size, 1),
        opened_px=int(dark.sum()) - n_dark, region=opts.region,
        bbox=_mask_bbox(opened, sx, sy, h_units))
    if n_dark == 0:
        return WallExtraction("none", planraster.Geometry(), diag, "no dark pixels on the page")
    return WallExtraction("raster", planraster.Geometry(masks=(mask,)), diag, "")


def _positive(value, what):
    try:
        v = float(value)
    except (TypeError, ValueError) as e:
        raise OptionsError(f"{what} must be a number") from e
    if not math.isfinite(v) or v <= 0:
        raise OptionsError(f"{what} must be finite and > 0")
    return v


def _check_raster_options(opts):
    if not isinstance(opts.dark_threshold, int) or not 1 <= opts.dark_threshold <= 255:
        raise OptionsError("dark_threshold must be an integer 1..255")
    _positive(opts.oversample, "oversample")
    _positive(opts.max_pixels, "max_pixels")
    if not math.isfinite(opts.min_thickness_m) or opts.min_thickness_m < 0:
        raise OptionsError("min_thickness_m must be finite and >= 0")
    if not 0 <= opts.margin_fraction < 0.5:
        raise OptionsError("margin_fraction must be in [0, 0.5)")
    _check_region(opts.region)


def _trim_margin(dark, fraction):
    """Clear the outer band of the page so a scanned sheet border is not read as walls."""
    rows, cols = round(dark.shape[0] * fraction), round(dark.shape[1] * fraction)
    out = dark.copy()
    if rows:
        out[:rows] = False
        out[-rows:] = False
    if cols:
        out[:, :cols] = False
        out[:, -cols:] = False
    return out


def _crop(dark, region, sx, sy, h_units):
    x0, y0, x1, y1 = (float(v) for v in region)
    keep = np.zeros_like(dark)
    c0, c1 = max(0, math.floor(x0 * sx)), min(dark.shape[1], math.ceil(x1 * sx))
    r0 = max(0, math.floor((h_units - y1) * sy))
    r1 = min(dark.shape[0], math.ceil((h_units - y0) * sy))
    if r1 > r0 and c1 > c0:
        keep[r0:r1, c0:c1] = dark[r0:r1, c0:c1]
    return keep


def _box_pass(mask, k, erode):
    """One separable (2k+1) window pass per axis (erosion keeps full windows, else any)."""
    out = mask
    for axis in (0, 1):
        n = out.shape[axis]
        pad = [(0, 0), (0, 0)]
        pad[axis] = (1, 0)
        cs = np.pad(np.cumsum(out, axis=axis, dtype=np.int32), pad)
        hi = np.minimum(np.arange(n) + k + 1, n)
        lo = np.maximum(np.arange(n) - k, 0)
        sums = np.take(cs, hi, axis=axis) - np.take(cs, lo, axis=axis)
        shape = [1, 1]
        shape[axis] = -1
        out = (sums == (hi - lo).reshape(shape)) if erode else (sums > 0)
    return out


def _open_box(mask, k):
    """Morphological opening with a (2k+1)-pixel square: drops features thinner than that."""
    if k < 1:
        return mask
    return _box_pass(_box_pass(mask, k, True), k, False)


def _mask_bbox(mask, sx, sy, h_units):
    rows, cols = np.nonzero(mask.any(axis=1))[0], np.nonzero(mask.any(axis=0))[0]
    if not len(rows) or not len(cols):
        return None
    return (float(cols[0] / sx), float(h_units - (rows[-1] + 1) / sy),
            float((cols[-1] + 1) / sx), float(h_units - rows[0] / sy))


def diagnostics_dict(extraction):
    return asdict(extraction.diagnostics)
