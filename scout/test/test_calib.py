"""Scale notes, two-point, similarity and span checks (docker/floorplan-importer/calib.py)."""
import importlib.util
import math
import pathlib
import sys

import numpy as np
import pytest

_SRC = pathlib.Path(__file__).resolve().parents[2] / 'docker' / 'floorplan-importer' / 'calib.py'
_spec = importlib.util.spec_from_file_location('fp_calib', _SRC)
cal = importlib.util.module_from_spec(_spec)
sys.modules['fp_calib'] = cal
_spec.loader.exec_module(cal)

ARCH_D = (0, 0, 2592, 1728)
QUARTER = 'SCALE: 1/4" = 1\'-0"'
EIGHTH = 'SCALE: 1/8" = 1\'-0"'
M_PER_PT_PAPER = 0.0254 / 72


def rot(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def ratios(text):
    return [n.ratio for n in cal.parse_scale_notes(text)]


# --- scale-note parsing ---------------------------------------------------------------

@pytest.mark.parametrize('text, ratio', [
    ('1/4" = 1\'-0"', 48.0),
    ('1/8"=1\'', 96.0),
    ('SCALE: 1:100', 100.0),
    ('3/32" = 1\'-0"', 128.0),
    ('1" = 20\'-0"', 240.0),
    ('1 1/2" = 1\'-0"', 8.0),
    ('3" = 1\'-0"', 4.0),
    ('1/4 INCH = 1 FOOT', 48.0),
    ('SCALE 1 : 50', 50.0),
    ('1:100.', 100.0),
    ('1/4” = 1’-0”', 48.0),
    ("1/4'' = 1'-0''", 48.0),
    ('SCALE:\n1/4"\n=  1\'-0"', 48.0),
    ('1/2" = 1\'-6"', 36.0),
])
def test_scale_note_forms_parse(text, ratio):
    assert ratios(text) == [pytest.approx(ratio)]


@pytest.mark.parametrize('text', [
    'DRAWN 1:30 PM', 'issued 11:30', '12:00:30', '1:20000', 'SHEET A-101', '1/4" wide', '',
    'NOT TO SCALE',
])
def test_text_that_is_not_a_scale_note_is_ignored(text):
    assert ratios(text) == []


def test_notes_come_back_in_reading_order():
    assert ratios('DETAIL 1/2" = 1\'-0"  PLAN ' + QUARTER + '  KEY 1:200') == [
        pytest.approx(24.0), pytest.approx(48.0), pytest.approx(200.0)]


@pytest.mark.parametrize('text', [
    'NOT TO SCALE', 'Not  To Scale', 'N.T.S.', 'NTS', 'n.t.s', 'NO SCALE', 'UNSCALED',
])
def test_veto_phrases(text):
    assert cal.has_veto(text)


@pytest.mark.parametrize('text', [QUARTER, 'ANTS', 'SCALED PLAN', 'CONTENTS'])
def test_non_veto_text(text):
    assert not cal.has_veto(text)


# --- propose_scale ----------------------------------------------------------------------

def test_arch_d_quarter_inch_sheet_is_a_high_confidence_proposal():
    p = cal.propose_scale([QUARTER], ARCH_D)
    assert p.m_per_point == pytest.approx(48 * M_PER_PT_PAPER, rel=1e-12)
    assert p.confidence == cal.HIGH
    joined = ' | '.join(p.evidence)
    assert 'ARCH D' in joined and '24.0 x 36.0 in' in joined
    assert '1/4"' in joined and '1:48' in joined and 'UserUnit: 1' in joined


def test_one_inch_of_paper_at_this_scale_is_four_feet_of_building():
    p = cal.propose_scale([QUARTER], ARCH_D)
    assert p.m_per_point * 72 == pytest.approx(4 * 0.3048, rel=1e-12)


def test_proposal_accepts_one_string_blobs_around_the_note_and_an_offset_mediabox():
    blobs = ['A-101', 'FLOOR PLAN', QUARTER, 'REV 3']
    a = cal.propose_scale(blobs, ARCH_D)
    b = cal.propose_scale('\n'.join(blobs), (10, 20, 2602, 1748))
    c = cal.propose_scale(blobs, (2592, 1728, 0, 0))
    assert a.m_per_point == b.m_per_point == c.m_per_point


def test_repeated_identical_notes_are_not_a_conflict():
    p = cal.propose_scale([QUARTER, QUARTER, 'ENLARGED ' + QUARTER], ARCH_D)
    assert p.confidence == cal.HIGH and 'x3' in ' '.join(p.evidence)


def test_not_to_scale_vetoes_even_with_a_scale_note():
    for veto in ('NOT TO SCALE', 'N.T.S.'):
        p = cal.propose_scale([QUARTER, 'DETAIL ' + veto], ARCH_D)
        assert p.m_per_point is None and p.confidence == cal.NONE
        assert any('vetoed' in line for line in p.evidence)


def test_conflicting_notes_are_low_confidence_with_the_modal_value():
    p = cal.propose_scale([QUARTER, EIGHTH, QUARTER], ARCH_D)
    assert p.confidence == cal.LOW
    assert p.m_per_point == pytest.approx(48 * M_PER_PT_PAPER)
    assert any('disagree' in line for line in p.evidence)
    assert sum(' = 1:' in line for line in p.evidence) == 2


def test_conflict_tie_takes_the_first_note():
    p = cal.propose_scale([EIGHTH, QUARTER], ARCH_D)
    assert p.confidence == cal.LOW and p.m_per_point == pytest.approx(96 * M_PER_PT_PAPER)


def test_no_note_means_no_proposal():
    for blobs in ([], ['A-101 FLOOR PLAN'], ['']):
        p = cal.propose_scale(blobs, ARCH_D)
        assert p.m_per_point is None and p.confidence == cal.NONE
        assert any('no scale note' in line for line in p.evidence)


def test_userunit_scales_paper_size_and_proposal():
    base = cal.propose_scale([QUARTER], ARCH_D)
    doubled = cal.propose_scale([QUARTER], ARCH_D, userunit=2.0)
    assert doubled.m_per_point == pytest.approx(2 * base.m_per_point)
    assert 'UserUnit: 2' in ' '.join(doubled.evidence)
    assert 'not a standard size' in ' '.join(doubled.evidence)
    assert doubled.confidence == cal.MEDIUM
    half = cal.propose_scale([QUARTER], (0, 0, 1296, 864), userunit=2.0)
    assert 'ARCH D' in ' '.join(half.evidence) and half.confidence == cal.HIGH
    assert half.m_per_point == pytest.approx(base.m_per_point * 2)


def test_metric_ratio_on_an_iso_sheet():
    p = cal.propose_scale(['SCALE: 1:100'], (0, 0, 1684, 2384))
    assert p.m_per_point == pytest.approx(100 * M_PER_PT_PAPER)
    assert p.confidence == cal.HIGH and 'ISO A1' in ' '.join(p.evidence)


def test_nonstandard_sheet_is_medium_confidence():
    p = cal.propose_scale([QUARTER], (0, 0, 2000, 1500))
    assert p.confidence == cal.MEDIUM and 'not a standard size' in ' '.join(p.evidence)


def test_reduced_size_sheet_is_medium_and_says_so():
    p = cal.propose_scale([QUARTER], (0, 0, 792, 1224))
    assert p.confidence == cal.MEDIUM
    assert 'Tabloid' in ' '.join(p.evidence) and 'reduced' in ' '.join(p.evidence)


@pytest.mark.parametrize('w, h, name', [
    (36, 24, 'ARCH D'), (24, 36, 'ARCH D'), (24.3, 36.1, 'ARCH D'), (17, 22, 'ANSI C'),
    (22, 34, 'ANSI D'), (8.5, 11, 'ANSI A (Letter)'), (33.11, 46.81, 'ISO A0'),
    (25, 36, None), (20, 30, None),
])
def test_identify_sheet(w, h, name):
    assert cal.identify_sheet(w, h) == name


@pytest.mark.parametrize('kwargs', [
    {'mediabox': (0, 0, 0, 100)}, {'mediabox': (0, 0, 100)}, {'mediabox': None},
    {'mediabox': (0, 0, float('nan'), 10)}, {'mediabox': ARCH_D, 'userunit': 0},
    {'mediabox': ARCH_D, 'userunit': -1}, {'mediabox': ARCH_D, 'userunit': float('nan')},
    {'mediabox': ARCH_D, 'userunit': 'x'},
])
def test_bad_sheet_geometry_is_refused(kwargs):
    with pytest.raises(cal.CalibrationError):
        cal.propose_scale([QUARTER], **kwargs)


# --- two_point --------------------------------------------------------------------------

def test_two_point_is_exact_for_a_known_distance():
    m_per_pt = 48 * M_PER_PT_PAPER
    p1 = (100.25, 200.5)
    for dx, dy in ((3 * 17.3, 4 * 17.3), (-86.5, 0.0), (0.0, 400.0), (123.4, -56.7)):
        p2 = (p1[0] + dx, p1[1] + dy)
        metres = m_per_pt * math.hypot(dx, dy)
        assert abs(cal.two_point(p1, p2, metres) - m_per_pt) < 1e-9
        assert abs(cal.two_point(p2, p1, metres) - m_per_pt) < 1e-9


def test_two_point_recovers_the_title_block_scale_from_a_dimension_string():
    assert cal.two_point((0, 0), (0, 720), 12 * 0.3048) == pytest.approx(0.3048 * 12 / 720)


@pytest.mark.parametrize('p1, p2, metres', [
    ((5, 5), (5, 5), 3.0), ((5, 5), (5 + 1e-12, 5), 3.0),
    ((0, 0), (1, 1), 0), ((0, 0), (1, 1), -2.0), ((0, 0), (1, 1), float('nan')),
    ((0, 0), (1, 1), float('inf')), ((0, 0), (1, 1), 'x'),
    ((0, 0, 0), (1, 1), 2.0), ((0, 0), None, 2.0), ((0, float('nan')), (1, 1), 2.0),
])
def test_two_point_refuses_degenerate_input(p1, p2, metres):
    with pytest.raises(cal.CalibrationError):
        cal.two_point(p1, p2, metres)


# --- map_frame --------------------------------------------------------------------------

def test_map_frame_puts_the_origin_at_zero_and_x_along_the_chosen_axis():
    origin = (300.0, 150.0)
    s, rot_m, t = cal.map_frame(0.02, math.radians(30), origin)
    to_map = lambda p: s * (rot_m @ np.asarray(p, float)) + t  # noqa: E731
    assert to_map(origin) == pytest.approx([0, 0], abs=1e-12)
    c30, s30 = math.cos(math.radians(30)), math.sin(math.radians(30))
    ahead = np.array(origin) + 100 * np.array([c30, s30])
    assert to_map(ahead) == pytest.approx([2.0, 0.0], abs=1e-9)
    left = np.array(origin) + 100 * np.array([-s30, c30])
    assert to_map(left) == pytest.approx([0.0, 2.0], abs=1e-9)
    assert np.linalg.det(rot_m) == pytest.approx(1.0) and s == 0.02


def test_map_frame_with_no_rotation_is_a_scaled_shift():
    s, rot_m, t = cal.map_frame(0.5, 0.0, (10, 20))
    assert (rot_m @ np.array([14.0, 26.0])) * s + t == pytest.approx([2.0, 3.0])


@pytest.mark.parametrize('scale', [0, -1.0, float('nan')])
def test_map_frame_refuses_a_bad_scale(scale):
    with pytest.raises(cal.CalibrationError):
        cal.map_frame(scale, 0.0, (0, 0))


# --- similarity -------------------------------------------------------------------------

def make_pairs(n, s, deg, t, sigma, seed):
    rng = np.random.default_rng(seed)
    src = rng.uniform(0, 2000, size=(n, 2))
    dst = s * (src @ rot(deg).T) + np.asarray(t) + rng.normal(0, sigma, size=(n, 2))
    return src, dst


def test_similarity_recovers_an_exact_transform():
    src, dst = make_pairs(12, 0.0169333, 37.0, (12.5, -3.2), 0.0, 1)
    s, rot_m, t, rms = cal.similarity(src, dst)
    assert s == pytest.approx(0.0169333, rel=1e-9)
    assert rot_m == pytest.approx(rot(37.0), abs=1e-9)
    assert t == pytest.approx([12.5, -3.2], abs=1e-6)
    assert rms < 1e-9


def test_similarity_with_noise_recovers_parameters_and_reports_rms():
    sigma, n = 0.01, 20
    src, dst = make_pairs(n, 0.0169333, -123.0, (-40.0, 7.0), sigma, 7)
    s, rot_m, t, rms = cal.similarity(src, dst)
    assert s == pytest.approx(0.0169333, rel=2e-3)
    assert math.atan2(rot_m[1, 0], rot_m[0, 0]) == pytest.approx(math.radians(-123.0), abs=1e-3)
    assert t == pytest.approx([-40.0, 7.0], abs=0.1)
    assert rms == pytest.approx(sigma * math.sqrt((2 * n - 4) / n), rel=0.4)
    assert rms > 0


def test_two_correspondences_are_enough_and_exact():
    src = np.array([[0.0, 0.0], [100.0, 0.0]])
    dst = 0.02 * (src @ rot(90).T) + np.array([5.0, 6.0])
    s, rot_m, t, rms = cal.similarity(src, dst)
    assert s == pytest.approx(0.02) and rms < 1e-12
    assert rot_m == pytest.approx(rot(90), abs=1e-12)
    assert t == pytest.approx([5.0, 6.0])


def test_collinear_points_beyond_two_still_fit():
    src = np.array([[0.0, 0.0], [50.0, 50.0], [100.0, 100.0]])
    dst = 0.5 * (src @ rot(20).T)
    s, rot_m, _, rms = cal.similarity(src, dst)
    assert s == pytest.approx(0.5) and rms < 1e-9
    assert np.linalg.det(rot_m) == pytest.approx(1.0)


def test_rotation_is_proper_and_a_mirrored_input_shows_up_as_rms():
    src = np.array([[0.0, 0.0], [100.0, 0.0], [100.0, 60.0], [0.0, 60.0]])
    mirrored = src * np.array([1.0, -1.0])
    s, rot_m, _, rms = cal.similarity(src, mirrored)
    assert np.linalg.det(rot_m) == pytest.approx(1.0)
    assert rms > 5.0 and s > 0


@pytest.mark.parametrize('src, dst', [
    (np.zeros((0, 2)), np.zeros((0, 2))),
    (np.array([[1.0, 2.0]]), np.array([[3.0, 4.0]])),
    (np.array([[1.0, 2.0], [3.0, 4.0]]), np.array([[1.0, 2.0]])),
    (np.array([[5.0, 5.0], [5.0, 5.0]]), np.array([[0.0, 0.0], [1.0, 1.0]])),
    (np.array([[0.0, 0.0], [1.0, 1.0]]), np.array([[7.0, 7.0], [7.0, 7.0]])),
    (np.array([[0.0, 0.0], [1.0, float('nan')]]), np.array([[0.0, 0.0], [1.0, 1.0]])),
    (np.array([0.0, 1.0, 2.0]), np.array([0.0, 1.0, 2.0])),
    (np.zeros((3, 3)), np.zeros((3, 3))),
])
def test_similarity_refuses_degenerate_input(src, dst):
    with pytest.raises(cal.CalibrationError):
        cal.similarity(src, dst)


# --- span consistency -------------------------------------------------------------------

def span(scale, length=300.0, metres_noise=1.0, axis=0):
    p1 = (50.0, 60.0)
    p2 = (50.0 + length, 60.0) if axis == 0 else (50.0, 60.0 + length)
    return (p1, p2, scale * length * metres_noise)


def test_spans_that_agree_pass_and_report_the_mean_scale():
    c = cal.check_spans([span(0.0169, 300), span(0.0169 * 1.005, 420, axis=1)])
    assert c.consistent and c.spread == pytest.approx(0.005 / 1.0025, rel=1e-6)
    assert c.m_per_point == pytest.approx(0.0169 * 1.0025)
    assert len(c.per_span) == 2


def test_spans_that_disagree_by_three_percent_are_flagged():
    c = cal.check_spans([span(0.0169), span(0.0169 * 1.03, axis=1)])
    assert not c.consistent and c.spread == pytest.approx(0.03 / 1.015, rel=1e-6)
    assert c.m_per_point == pytest.approx(0.0169 * 1.015)


def test_one_percent_boundary():
    assert cal.check_spans([span(1.0), span(1.0098)]).consistent
    assert not cal.check_spans([span(1.0), span(1.0102)]).consistent


def test_tolerance_is_adjustable_and_three_spans_use_the_extremes():
    spans = [span(0.0169), span(0.0169 * 1.02), span(0.0169 * 1.01)]
    assert not cal.check_spans(spans).consistent
    assert cal.check_spans(spans, tolerance=0.05).consistent


def test_span_check_needs_two_spans_and_valid_spans():
    with pytest.raises(cal.CalibrationError, match='2 spans'):
        cal.check_spans([span(0.0169)])
    with pytest.raises(cal.CalibrationError):
        cal.check_spans([])
    with pytest.raises(cal.CalibrationError):
        cal.check_spans([span(0.0169), ((5, 5), (5, 5), 3.0)])
