"""Scale and calibration math for floorplan import (numpy only).

Page space is PDF points (1/72 in at UserUnit 1) with y up; callers flip y first if their
source is y-down. Calibrations are kept in page space so re-rendering at another DPI does
not invalidate them. transforms are the triple (s, R, t): dst = s * R @ src + t.
"""

import math
import re
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

PT_PER_INCH = 72.0
INCH_M = 0.0254
INCHES_PER_FOOT = 12.0
COINCIDENT_EPS = 1e-9
SPAN_TOLERANCE = 0.01
SHEET_TOLERANCE = 0.015
LARGE_FORMAT_SHORT_IN = 16.0
MAX_METRIC_RATIO = 10000.0

HIGH = "high"
MEDIUM = "medium"
LOW = "low"
NONE = "none"

SHEETS = (
    ("ARCH A", 9.0, 12.0),
    ("ARCH B", 12.0, 18.0),
    ("ARCH C", 18.0, 24.0),
    ("ARCH D", 24.0, 36.0),
    ("ARCH E1", 30.0, 42.0),
    ("ARCH E", 36.0, 48.0),
    ("ANSI A (Letter)", 8.5, 11.0),
    ("ANSI B (Tabloid)", 11.0, 17.0),
    ("ANSI C", 17.0, 22.0),
    ("ANSI D", 22.0, 34.0),
    ("ANSI E", 34.0, 44.0),
    ("Legal", 8.5, 14.0),
    ("ISO A4", 8.27, 11.69),
    ("ISO A3", 11.69, 16.54),
    ("ISO A2", 16.54, 23.39),
    ("ISO A1", 23.39, 33.11),
    ("ISO A0", 33.11, 46.81),
)

_NUM = r"\d+\s+\d+/\d+|\d+/\d+|\d+(?:\.\d+)?"
_IMPERIAL = re.compile(
    rf"(?P<paper>{_NUM})\s*(?:\"|in(?:ch(?:es)?)?\b)\s*=\s*"
    r"(?P<feet>\d+(?:\.\d+)?)\s*(?:'|ft\b|feet\b|foot\b)"
    rf"(?:\s*-\s*(?P<inches>{_NUM})\s*(?:\"|in\b)?)?",
    re.IGNORECASE,
)
_METRIC = re.compile(
    r"(?<![\d:./])1\s*:\s*(?P<n>\d+(?:\.\d+)?)(?![\d:]|\.\d|\s*[ap]\.?m\b)", re.IGNORECASE
)
_VETO = re.compile(
    r"\bnot\s+to\s+scale\b|\bn\.?t\.?s\b|\bno\s+scale\b|\bunscaled\b", re.IGNORECASE
)
_QUOTES = {
    0x201C: '"', 0x201D: '"', 0x2033: '"',
    0x2018: "'", 0x2019: "'", 0x2032: "'",
}


class CalibrationError(ValueError):
    pass


class ScaleNote(NamedTuple):
    ratio: float
    text: str


@dataclass(frozen=True)
class ScaleProposal:
    m_per_point: float | None
    evidence: tuple
    confidence: str


@dataclass(frozen=True)
class SpanCheck:
    m_per_point: float
    per_span: tuple
    spread: float
    consistent: bool


def parse_scale_notes(text):
    """Scale notes in order of appearance; ratio = real length / paper length."""
    t = text.translate(_QUOTES).replace("''", '"')
    found = []
    for m in _IMPERIAL.finditer(t):
        paper = _fraction(m.group("paper"))
        real_in = float(m.group("feet")) * INCHES_PER_FOOT + _fraction(m.group("inches") or "0")
        if paper > 0 and real_in > 0:
            found.append((m.start(), ScaleNote(real_in / paper, " ".join(m.group(0).split()))))
    for m in _METRIC.finditer(t):
        n = float(m.group("n"))
        if 1.0 <= n <= MAX_METRIC_RATIO:
            found.append((m.start(), ScaleNote(n, " ".join(m.group(0).split()))))
    return [note for _, note in sorted(found, key=lambda item: item[0])]


def has_veto(text):
    return _VETO.search(text.translate(_QUOTES)) is not None


def identify_sheet(width_in, height_in):
    short, long_ = sorted((width_in, height_in))
    best = None
    for name, s, lg in SHEETS:
        err = max(abs(short - s) / s, abs(long_ - lg) / lg)
        if err <= SHEET_TOLERANCE and (best is None or err < best[0]):
            best = (err, name)
    return best[1] if best else None


def propose_scale(text_blobs, mediabox, userunit=1.0):
    """Scale from sheet text. A proposal for the operator to accept, never applied silently."""
    unit = _positive(userunit, "userunit")
    width_in, height_in = _sheet_inches(mediabox, unit)
    text = text_blobs if isinstance(text_blobs, str) else "\n".join(text_blobs)
    sheet = identify_sheet(width_in, height_in)
    evidence = [_sheet_line(sheet, width_in, height_in), f"UserUnit: {unit:g}"]
    veto = _VETO.search(text.translate(_QUOTES))
    if veto:
        evidence.append(f"scale vetoed by sheet text {veto.group(0)!r}")
        return ScaleProposal(None, tuple(evidence), NONE)
    notes = parse_scale_notes(text)
    if not notes:
        evidence.append("no scale note found")
        return ScaleProposal(None, tuple(evidence), NONE)
    groups = _group(notes)
    best = max(groups, key=lambda g: len(g[1]))
    evidence.extend(f"scale note {n[0].text!r} = 1:{g:g} (x{len(n)})" for g, n in groups)
    if len(groups) > 1:
        evidence.append("scale notes disagree; modal value proposed, verify with two-point")
    m_per_point = best[0] * INCH_M * unit / PT_PER_INCH
    confidence = _confidence(len(groups), sheet, min(width_in, height_in))
    return ScaleProposal(m_per_point, tuple(evidence), confidence)


def two_point(p1, p2, metres):
    a, b = _point(p1, "p1"), _point(p2, "p2")
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    if d < COINCIDENT_EPS:
        raise CalibrationError("calibration points coincide")
    return _positive(metres, "metres") / d


def map_frame(m_per_unit, rotation, origin):
    """(s, R, t) taking source points to a map frame whose +X points `rotation` rad ccw
    from the source +x axis and whose (0, 0) sits at source point `origin`."""
    s = _positive(m_per_unit, "m_per_unit")
    c, sn = math.cos(-rotation), math.sin(-rotation)
    rot = np.array([[c, -sn], [sn, c]])
    return s, rot, -s * (rot @ _point(origin, "origin"))


def similarity(src_pts, dst_pts):
    """Umeyama fit dst ~ s * R @ src + t (R a proper rotation); returns (s, R, t, rms)."""
    src, dst = _points(src_pts, "src_pts"), _points(dst_pts, "dst_pts")
    n = len(src)
    if n != len(dst):
        raise CalibrationError("src_pts and dst_pts differ in length")
    if n < 2:
        raise CalibrationError("need at least 2 correspondences")
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    xs, yd = src - mu_s, dst - mu_d
    var_s = float((xs**2).sum()) / n
    if var_s < COINCIDENT_EPS**2:
        raise CalibrationError("source points coincide")
    u, d, vt = np.linalg.svd(yd.T @ xs / n)
    flip = 1.0 if np.linalg.det(u) * np.linalg.det(vt) >= 0 else -1.0
    rot = u @ np.diag([1.0, flip]) @ vt
    s = float(d[0] + flip * d[1]) / var_s
    if not s > COINCIDENT_EPS:
        raise CalibrationError("destination points coincide")
    t = mu_d - s * (rot @ mu_s)
    resid = dst - (s * (src @ rot.T) + t)
    return s, rot, t, math.sqrt(float((resid**2).sum()) / n)


def check_spans(spans, tolerance=SPAN_TOLERANCE):
    """Each span is (p1, p2, metres) in page points; flags spans whose scales disagree."""
    spans = list(spans)
    if len(spans) < 2:
        raise CalibrationError("need at least 2 spans to cross-check the scale")
    scales = tuple(two_point(p1, p2, metres) for p1, p2, metres in spans)
    mean = sum(scales) / len(scales)
    spread = (max(scales) - min(scales)) / mean
    return SpanCheck(mean, scales, spread, spread <= tolerance)


def _fraction(text):
    total = 0.0
    for part in text.split():
        num, _, den = part.partition("/")
        total += float(num) / float(den) if den else float(num)
    return total


def _group(notes):
    """Distinct ratios with their notes, in order of first appearance."""
    order = []
    for note in notes:
        for g in order:
            if math.isclose(g[0], note.ratio, rel_tol=1e-6):
                g[1].append(note)
                break
        else:
            order.append((note.ratio, [note]))
    return order


def _confidence(distinct, sheet, short_in):
    if distinct > 1:
        return LOW
    return HIGH if sheet and short_in >= LARGE_FORMAT_SHORT_IN else MEDIUM


def _sheet_line(sheet, width_in, height_in):
    short, long_ = sorted((width_in, height_in))
    dims = f"{short:.1f} x {long_:.1f} in"
    if sheet is None:
        return f"sheet: {dims} (not a standard size)"
    if short < LARGE_FORMAT_SHORT_IN:
        return f"sheet: {sheet}, {dims} (reduced size: title-block scale may be the original's)"
    return f"sheet: {sheet}, {dims}"


def _sheet_inches(mediabox, unit):
    try:
        x0, y0, x1, y1 = (float(v) for v in mediabox)
    except (TypeError, ValueError) as e:
        raise CalibrationError("mediabox must be (x0, y0, x1, y1)") from e
    w, h = abs(x1 - x0), abs(y1 - y0)
    if not (math.isfinite(w) and math.isfinite(h)) or w <= 0 or h <= 0:
        raise CalibrationError("mediabox must have a positive finite size")
    return w * unit / PT_PER_INCH, h * unit / PT_PER_INCH


def _positive(value, what):
    try:
        v = float(value)
    except (TypeError, ValueError) as e:
        raise CalibrationError(f"{what} must be a number") from e
    if not math.isfinite(v) or v <= 0:
        raise CalibrationError(f"{what} must be finite and > 0")
    return v


def _point(p, what):
    try:
        a = np.asarray(p, dtype=float)
    except (TypeError, ValueError) as e:
        raise CalibrationError(f"{what} must be numeric") from e
    if a.shape != (2,) or not np.isfinite(a).all():
        raise CalibrationError(f"{what} must be a finite (x, y)")
    return a


def _points(pts, what):
    try:
        a = np.asarray(pts, dtype=float)
    except (TypeError, ValueError) as e:
        raise CalibrationError(f"{what} must be numeric") from e
    if a.ndim != 2 or a.shape[1] != 2 or not np.isfinite(a).all():
        raise CalibrationError(f"{what} must be a finite (N, 2) array")
    return a
