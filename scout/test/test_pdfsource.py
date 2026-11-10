"""PDF ingestion (docker/floorplan-importer/pdfsource.py): open, pages, scale, walls, raster, PNG.

Committed tests build their PDFs in-process with PyMuPDF. The sample-backed tests at the bottom
read the real A-101 sheets from $SCOUT_FLOORPLAN_SAMPLES (default: the main checkout's
docs/floorplans) and skip when they are absent; those PDFs are never committed.
"""
import importlib
import json
import math
import os
import pathlib
import sys

import numpy as np
import pytest

pymupdf = pytest.importorskip('pymupdf')

_DIR = pathlib.Path(__file__).resolve().parents[2] / 'docker' / 'floorplan-importer'
sys.path.insert(0, str(_DIR))
try:
    ps = importlib.import_module('pdfsource')
finally:
    sys.path.remove(str(_DIR))
calib = sys.modules['calib']
pr = sys.modules['planraster']

BLACK = (0, 0, 0)
M_PT = 48 * 0.0254 / 72
SCALE_NOTE = 'SCALE: 1/4" = 1\'-0"'
W, H = 2592, 1728
ROOM = (600.0, 500.0, 600.0 + 10 / M_PT, 500.0 + 6 / M_PT)
GAP = (800.0, 800.0 + 0.9 / M_PT)
WALL_PT = 6.0


def m2pt(metres):
    return metres / M_PT


# --- synthetic sheets ------------------------------------------------------------------------

def draw_frame_and_title(page, note=SCALE_NOTE):
    page.draw_rect(pymupdf.Rect(36, 36, W - 36, H - 36), color=BLACK, width=2.2)
    page.draw_rect(pymupdf.Rect(40, 40, W - 40, H - 40), color=BLACK, width=0.5)
    page.draw_line((2300, 40), (2300, H - 40), color=BLACK, width=0.6)
    page.draw_rect(pymupdf.Rect(2320, 800, 2400, 820), color=BLACK, fill=BLACK, width=0.5)
    page.insert_text((2320, 100), 'PROJECT TITLE', fontsize=24)
    if note:
        page.insert_text((2320, 1650), note, fontsize=12)


def draw_noise(page):
    page.draw_line((ROOM[0], 440), (ROOM[2], 440), color=BLACK, width=0.4)
    page.draw_line((ROOM[0], 430), (ROOM[0], 450), color=BLACK, width=0.4)
    page.draw_line((ROOM[2], 430), (ROOM[2], 450), color=BLACK, width=0.4)
    page.insert_text((900, 430), '32\'-10"', fontsize=10)
    page.draw_line((ROOM[0], 1000), (ROOM[2], 1000), color=(0.1, 0.2, 0.6), width=2.0)
    page.draw_line((1000, 100), (1000, 1600), color=(0.5, 0.5, 0.5), width=2.0, dashes='[6 3] 0')
    for i in range(20):
        page.draw_line((1300 + 10 * i, 600), (1340 + 10 * i, 560), color=BLACK, width=0.25)
    page.insert_text((800, 700), 'ROOM 101', fontsize=20)


def draw_stroke_walls(page, door=True, width=WALL_PT):
    x0, y0, x1, y1 = ROOM
    if door:
        pts = [(GAP[1], y1), (x1, y1), (x1, y0), (x0, y0), (x0, y1), (GAP[0], y1)]
    else:
        pts = [(x0, y1), (x1, y1), (x1, y0), (x0, y0), (x0, y1)]
    page.draw_polyline(pts, color=BLACK, width=width, closePath=False)


def draw_poche_walls(page, door=True):
    x0, y0, x1, y1 = ROOM
    t = m2pt(0.2)
    strips = [(x0 - t / 2, y0 - t / 2, x1 + t / 2, y0 + t / 2),
              (x0 - t / 2, y0 - t / 2, x0 + t / 2, y1 + t / 2),
              (x1 - t / 2, y0 - t / 2, x1 + t / 2, y1 + t / 2)]
    if door:
        strips += [(x0 - t / 2, y1 - t / 2, GAP[0], y1 + t / 2),
                   (GAP[1], y1 - t / 2, x1 + t / 2, y1 + t / 2)]
    else:
        strips.append((x0 - t / 2, y1 - t / 2, x1 + t / 2, y1 + t / 2))
    for s in strips:
        page.draw_rect(pymupdf.Rect(*s), color=BLACK, fill=(0.29, 0.29, 0.29), width=0.6)


def draw_poche_noise(page):
    rng = np.random.default_rng(5)
    for x, y in rng.uniform(100, 2200, size=(150, 2)):
        page.draw_rect(pymupdf.Rect(x, y, x + 0.8, y + 0.8), fill=(0.85, 0.87, 0.89), width=0)
    page.draw_rect(pymupdf.Rect(1500, 900, 1501, 901), fill=BLACK, width=0)
    page.draw_rect(pymupdf.Rect(1600, 900, 1700, 960), fill=(1, 1, 1), color=(0.5, 0.5, 0.5))
    page.insert_text((1610, 940), 'LABEL', fontsize=14)
    page.draw_rect(pymupdf.Rect(1800, 905, 1803, 912), fill=BLACK, width=0)
    page.insert_text((1795, 912), 'WORD', fontsize=14)


def make_pdf(builder, rotation=0, userunit=None, width=W, height=H):
    doc = pymupdf.open()
    page = doc.new_page(width=width, height=height)
    builder(page)
    page.set_rotation(rotation)
    if userunit is not None:
        doc.xref_set_key(page.xref, 'UserUnit', str(userunit))
    return doc.tobytes()


def stroke_sheet(page, door=True):
    draw_frame_and_title(page)
    draw_noise(page)
    draw_stroke_walls(page, door=door)


def poche_sheet(page, door=True):
    draw_frame_and_title(page)
    draw_noise(page)
    draw_poche_noise(page)
    draw_poche_walls(page, door=door)


@pytest.fixture(scope='module')
def stroke_pdf():
    pdf = ps.open_pdf(make_pdf(stroke_sheet))
    yield pdf
    ps.close_pdf(pdf)


@pytest.fixture(scope='module')
def poche_pdf():
    pdf = ps.open_pdf(make_pdf(poche_sheet))
    yield pdf
    ps.close_pdf(pdf)


def image_only_pdf(vector_bytes, dpi=100, note=None):
    src = pymupdf.open(stream=vector_bytes, filetype='pdf')
    pix = src[0].get_pixmap(dpi=dpi, colorspace=pymupdf.csGRAY, alpha=False)
    out = pymupdf.open()
    page = out.new_page(width=src[0].rect.width, height=src[0].rect.height)
    page.insert_image(page.rect, pixmap=pix)
    if note:
        page.insert_text((2320, 1650), note, fontsize=12)
    return out.tobytes()


def decode_png(png):
    pix = pymupdf.Pixmap(png)
    arr = np.frombuffer(pix.samples, dtype=np.uint8).reshape(pix.height, pix.width, pix.n)
    return arr[:, :, 0].astype(int)


# --- open / validate ------------------------------------------------------------------------

def test_garbage_empty_and_non_bytes_are_not_pdfs():
    for blob in (b'hello world', b'', b'\x00' * 100, b'<html></html>'):
        with pytest.raises(ps.NotPdfError):
            ps.open_pdf(blob)
    for blob in ('text', None, 12, ['a']):
        with pytest.raises(ps.NotPdfError):
            ps.open_pdf(blob)


def test_header_without_a_document_is_corrupt_not_a_raw_exception():
    for blob in (b'%PDF-1.7\ngarbage garbage', b'%PDF-1.4\n' + bytes(range(256)) * 4):
        with pytest.raises(ps.CorruptPdfError):
            ps.open_pdf(blob)


def test_truncated_pdf_never_leaks_a_foreign_exception():
    data = make_pdf(stroke_sheet)
    for cut in (len(data) // 2, len(data) - 40, 200):
        try:
            pdf = ps.open_pdf(data[:cut])
        except ps.PdfSourceError:
            continue
        ps.list_pages(pdf)
        ps.close_pdf(pdf)


def test_password_protected_pdf_is_refused_but_owner_only_opens():
    doc = pymupdf.open()
    doc.new_page()
    locked = doc.tobytes(encryption=pymupdf.PDF_ENCRYPT_AES_256, owner_pw='o', user_pw='u')
    with pytest.raises(ps.EncryptedPdfError):
        ps.open_pdf(locked)
    doc = pymupdf.open()
    doc.new_page()
    owner_only = doc.tobytes(encryption=pymupdf.PDF_ENCRYPT_AES_256, owner_pw='o', user_pw='')
    pdf = ps.open_pdf(owner_only)
    assert pdf.n_pages == 1
    ps.close_pdf(pdf)


def test_zero_page_pdf_is_refused():
    blob = (b'%PDF-1.4\n1 0 obj<</Type/Catalog/Pages 2 0 R>>endobj\n'
            b'2 0 obj<</Type/Pages/Kids[]/Count 0>>endobj\n'
            b'trailer<</Root 1 0 R/Size 3>>\n%%EOF')
    with pytest.raises(ps.PdfSourceError) as info:
        ps.open_pdf(blob)
    assert isinstance(info.value, (ps.EmptyPdfError, ps.CorruptPdfError))


def test_size_cap_is_the_named_constant_and_inclusive(monkeypatch):
    assert ps.MAX_PDF_BYTES == 64 * 1024 * 1024
    data = make_pdf(stroke_sheet)
    monkeypatch.setattr(ps, 'MAX_PDF_BYTES', len(data))
    ps.close_pdf(ps.open_pdf(data))
    monkeypatch.setattr(ps, 'MAX_PDF_BYTES', len(data) - 1)
    with pytest.raises(ps.PdfTooLargeError):
        ps.open_pdf(data)


def test_page_cap_is_enforced_at_open(monkeypatch):
    doc = pymupdf.open()
    for _ in range(3):
        doc.new_page()
    data = doc.tobytes()
    assert ps.MAX_PAGES == 500
    monkeypatch.setattr(ps, 'MAX_PAGES', 3)
    ps.close_pdf(ps.open_pdf(data))
    monkeypatch.setattr(ps, 'MAX_PAGES', 2)
    with pytest.raises(ps.TooManyPagesError):
        ps.open_pdf(data)


def test_bytearray_and_memoryview_are_accepted():
    data = make_pdf(stroke_sheet)
    for blob in (bytearray(data), memoryview(data)):
        pdf = ps.open_pdf(blob)
        assert pdf.n_pages == 1 and pdf.n_bytes == len(data)
        ps.close_pdf(pdf)


def test_every_error_is_a_value_error_with_its_own_code():
    classes = [c for c in vars(ps).values()
               if isinstance(c, type) and issubclass(c, ps.PdfSourceError)]
    codes = [c.code for c in classes]
    assert len(classes) >= 9 and len(set(codes)) == len(codes)
    assert all(issubclass(c, ValueError) for c in classes)


# --- pages ----------------------------------------------------------------------------------

def test_list_pages_describes_vector_raster_and_rotated_pages():
    doc = pymupdf.open(stream=make_pdf(stroke_sheet), filetype='pdf')
    raster = pymupdf.open(stream=image_only_pdf(make_pdf(stroke_sheet)), filetype='pdf')
    rotated = pymupdf.open(stream=make_pdf(stroke_sheet, rotation=90), filetype='pdf')
    doc.insert_pdf(raster)
    doc.insert_pdf(rotated)
    pdf = ps.open_pdf(doc.tobytes())
    vec, ras, rot = ps.list_pages(pdf)
    assert (vec.index, vec.mediabox, vec.width, vec.height) == (0, (0, 0, W, H), W, H)
    assert vec.has_vectors is True and vec.n_paths >= 30 and vec.rotation == 0
    assert vec.userunit == 1.0 and vec.n_layers == 0 and vec.n_images == 0
    assert ras.has_vectors is False and ras.n_images == 1 and ras.n_paths < ps.MIN_VECTOR_PATHS
    assert (rot.width, rot.height, rot.rotation) == (H, W, 90)
    assert rot.mediabox == (0, 0, W, H)
    ps.close_pdf(pdf)


def test_stats_are_only_computed_for_the_first_pages(monkeypatch):
    doc = pymupdf.open()
    for _ in range(4):
        doc.new_page()
    pdf = ps.open_pdf(doc.tobytes())
    pages = ps.list_pages(pdf, stats_limit=2)
    assert [p.n_paths is None for p in pages] == [False, False, True, True]
    assert pages[3].has_vectors is None and pages[3].width > 0
    assert ps.MAX_STATS_PAGES == 50
    ps.close_pdf(pdf)


@pytest.mark.parametrize('index', [-1, 1, 99, 'x', None, 0.0, True])
def test_bad_page_index_is_refused(stroke_pdf, index):
    for call in (ps.page_info, ps.propose_page_scale, ps.render_page_png, ps.extract_walls):
        with pytest.raises(ps.PageRangeError):
            call(stroke_pdf, index)


def test_userunit_is_reported_and_page_units_are_user_space():
    pdf = ps.open_pdf(make_pdf(stroke_sheet, userunit=2))
    info = ps.page_info(pdf, 0)
    assert info.userunit == 2.0
    assert (info.width, info.height) == (W, H)
    ps.close_pdf(pdf)


def test_layer_count_comes_from_the_document():
    doc = pymupdf.open()
    page = doc.new_page()
    doc.add_ocg('A-WALL')
    doc.add_ocg('A-ANNO', on=False)
    page.draw_line((10, 10), (100, 10))
    pdf = ps.open_pdf(doc.tobytes())
    assert ps.page_info(pdf, 0).n_layers == 2 and pdf.layer_names == ('A-ANNO', 'A-WALL')
    assert pdf.hidden_layers == ('A-ANNO',)
    ps.close_pdf(pdf)


# --- scale proposal -----------------------------------------------------------------------

def test_arch_d_sheet_proposes_the_title_block_scale_with_evidence(stroke_pdf):
    rep = ps.propose_page_scale(stroke_pdf, 0)
    assert rep.proposal.m_per_point == pytest.approx(M_PT, rel=1e-12)
    assert rep.proposal.confidence == calib.HIGH
    assert not rep.calibration_required and not rep.raster
    text = ' | '.join(rep.proposal.evidence)
    assert 'ARCH D' in text and '1/4"' in text and 'UserUnit: 1' in text


def test_vector_sheet_without_a_scale_note_needs_calibration():
    def sheet(page):
        draw_frame_and_title(page, note=None)
        draw_stroke_walls(page)

    pdf = ps.open_pdf(make_pdf(sheet))
    rep = ps.propose_page_scale(pdf, 0)
    assert rep.proposal.m_per_point is None and rep.calibration_required and not rep.raster
    ps.close_pdf(pdf)


def test_not_to_scale_vetoes_the_proposal():
    def sheet(page):
        draw_frame_and_title(page, note=SCALE_NOTE + '   NOT TO SCALE')
        draw_stroke_walls(page)

    pdf = ps.open_pdf(make_pdf(sheet))
    rep = ps.propose_page_scale(pdf, 0)
    assert rep.proposal.m_per_point is None and rep.proposal.confidence == calib.NONE
    assert rep.calibration_required
    ps.close_pdf(pdf)


def test_raster_page_has_no_proposal_even_with_a_scale_note():
    pdf = ps.open_pdf(image_only_pdf(make_pdf(stroke_sheet), note=SCALE_NOTE))
    rep = ps.propose_page_scale(pdf, 0)
    assert rep.raster and rep.calibration_required
    assert rep.proposal.m_per_point is None and rep.proposal.confidence == calib.NONE
    assert 'raster' in ' '.join(rep.proposal.evidence)
    ps.close_pdf(pdf)


def half_sheet(page):
    page.draw_rect(pymupdf.Rect(18, 18, W // 2 - 18, H // 2 - 18), color=BLACK, width=1.1)
    page.draw_line((300, 300), (700, 300), color=BLACK, width=3)
    page.insert_text((1000, 830), SCALE_NOTE, fontsize=8)


def test_userunit_scales_the_proposal_and_sheet_size():
    pdf = ps.open_pdf(make_pdf(half_sheet, userunit=2, width=W // 2, height=H // 2))
    info = ps.page_info(pdf, 0)
    assert (info.width, info.height, info.userunit) == (W // 2, H // 2, 2.0)
    rep = ps.propose_page_scale(pdf, 0)
    assert 'ARCH D' in ' '.join(rep.proposal.evidence)
    assert rep.proposal.m_per_point == pytest.approx(2 * M_PT, rel=1e-12)
    ps.close_pdf(pdf)


def test_rotated_page_gets_the_same_proposal():
    pdf = ps.open_pdf(make_pdf(stroke_sheet, rotation=90))
    rep = ps.propose_page_scale(pdf, 0)
    assert rep.proposal.m_per_point == pytest.approx(M_PT)
    assert rep.proposal.confidence == calib.HIGH
    ps.close_pdf(pdf)


# --- vector walls: strokes ---------------------------------------------------------------

def bbox_of(geometry):
    pts = [p.points for p in geometry.polylines] + [r for f in geometry.fills for r in f.rings]
    allp = np.vstack(pts)
    return (*allp.min(axis=0), *allp.max(axis=0))


def test_heavy_strokes_are_walls_and_everything_else_is_dropped(stroke_pdf):
    ext = ps.extract_walls(stroke_pdf, 0)
    d = ext.diagnostics
    assert ext.kind == 'vector' and d.mode == 'strokes' and d.kept_strokes == 1
    assert d.kept_fills == 0 and d.width_threshold == pytest.approx(0.6 * WALL_PT)
    assert [c['width'] for c in d.stroke_classes if c['kept']] == [WALL_PT]
    assert {'frame', 'outside_region', 'colored', 'dashed', 'thin'} <= set(d.dropped)
    assert d.frame == pytest.approx((40, 40, W - 40, H - 40))
    assert d.region[2] == pytest.approx(2300, abs=2)
    x0, y0, x1, y1 = ROOM
    assert bbox_of(ext.geometry) == pytest.approx((x0, H - y1, x1, H - y0), abs=0.5)
    assert ext.geometry.polylines[0].width == pytest.approx(WALL_PT)


def test_page_frame_is_y_up_in_page_units():
    pdf = ps.open_pdf(make_pdf(stroke_sheet))
    box = bbox_of(ps.extract_walls(pdf, 0).geometry)
    assert box[1] > H / 2 - 400 and box[3] < H - 400
    assert box[3] == pytest.approx(H - ROOM[1], abs=0.5)
    ps.close_pdf(pdf)


def test_extracted_room_rasterizes_with_a_sealed_door(stroke_pdf):
    ext = ps.extract_walls(stroke_pdf, 0)
    rep = ps.propose_page_scale(stroke_pdf, 0)
    x0, y0, x1, y1 = ext.diagnostics.bbox
    tf = calib.map_frame(rep.proposal.m_per_point, 0.0, (x0, y0))
    grid, meta = pr.rasterize(ext.geometry, tf, 0.05)
    res, (ox, oy) = 0.05, meta['origin'][:2]
    rows, cols = np.nonzero(grid == pr.OCCUPIED)
    assert (cols.max() - cols.min() + 1) * res == pytest.approx(10.1, abs=0.2)
    assert (rows.max() - rows.min() + 1) * res == pytest.approx(6.1, abs=0.2)

    def at(x, y):
        return int(grid[grid.shape[0] - 1 - int(math.floor((y - oy) / res)),
                        int(math.floor((x - ox) / res))])

    assert at(5.0, 3.0) == pr.FREE and at(-0.5, 3.0) == pr.UNKNOWN
    gap_x = (GAP[0] - ROOM[0] + (GAP[1] - GAP[0]) / 2) * M_PT
    assert at(gap_x, 0.0) == pr.FREE and at(gap_x, -0.6) == pr.UNKNOWN


def test_mode_override_and_both(stroke_pdf):
    assert ps.extract_walls(stroke_pdf, 0, ps.WallOptions(mode='fills')).kind == 'none'
    both = ps.extract_walls(stroke_pdf, 0, ps.WallOptions(mode='both'))
    assert both.kind == 'vector' and both.diagnostics.mode == 'both'
    assert both.diagnostics.mode_reason == 'requested'


def test_min_stroke_width_overrides_the_automatic_class(stroke_pdf):
    ext = ps.extract_walls(stroke_pdf, 0, ps.WallOptions(min_stroke_width=0.1))
    assert ext.diagnostics.width_threshold == 0.1
    assert ext.diagnostics.kept_strokes > 1
    none = ps.extract_walls(stroke_pdf, 0, ps.WallOptions(min_stroke_width=50.0))
    assert none.kind == 'none' and none.reason


def test_frame_and_title_block_knobs(stroke_pdf):
    keep_frame = ps.extract_walls(stroke_pdf, 0, ps.WallOptions(drop_frame=False,
                                                                min_stroke_width=2.0))
    assert keep_frame.diagnostics.kept_strokes == 2
    assert 'frame' not in keep_frame.diagnostics.dropped
    no_title = ps.extract_walls(stroke_pdf, 0, ps.WallOptions(detect_title_block=False))
    assert no_title.diagnostics.region[2] == pytest.approx(W - 40)


def test_region_option_crops_to_the_operator_rectangle(stroke_pdf):
    x0, y0, x1, y1 = ROOM
    away = ps.extract_walls(stroke_pdf, 0, ps.WallOptions(region=(0, 0, 400, 400)))
    assert away.kind == 'none'
    around = ps.extract_walls(stroke_pdf, 0, ps.WallOptions(
        region=(x0 - 50, H - y1 - 50, x1 + 50, H - y0 + 50)))
    assert around.kind == 'vector' and around.diagnostics.region[0] == pytest.approx(x0 - 50)


def test_blank_and_all_thin_pages_have_no_walls():
    def thin(page):
        draw_frame_and_title(page)
        draw_noise(page)

    for builder in (lambda page: None, thin):
        pdf = ps.open_pdf(make_pdf(builder))
        ext = ps.extract_walls(pdf, 0)
        assert ext.kind == 'none' and ext.reason
        assert not ext.geometry.polylines and not ext.geometry.fills
        ps.close_pdf(pdf)


def test_curved_walls_are_flattened_into_polylines():
    def sheet(page):
        draw_frame_and_title(page)
        page.draw_circle((1200, 900), 200, color=BLACK, width=WALL_PT)

    pdf = ps.open_pdf(make_pdf(sheet))
    ext = ps.extract_walls(pdf, 0)
    pts = ext.geometry.polylines[0].points
    assert len(pts) >= 4 * 8 and np.allclose(pts[0], pts[-1], atol=0.5)
    assert np.hypot(pts[:, 0] - 1200, pts[:, 1] - (H - 900)).max() == pytest.approx(200, abs=1.5)
    ps.close_pdf(pdf)


def test_diagnostics_are_json_ready(stroke_pdf, poche_pdf):
    for pdf in (stroke_pdf, poche_pdf):
        out = json.loads(json.dumps(ps.diagnostics_dict(ps.extract_walls(pdf, 0))))
        assert out['mode'] in ('strokes', 'fills') and out['total_paths'] >= 20
        assert isinstance(out['stroke_classes'], list) and isinstance(out['dropped'], dict)


# --- vector walls: filled poche ----------------------------------------------------------

def test_dark_fills_are_walls_and_dots_light_fills_labels_are_not(poche_pdf):
    ext = ps.extract_walls(poche_pdf, 0)
    d = ext.diagnostics
    assert ext.kind == 'vector' and d.mode == 'fills' and d.kept_fills == 5
    assert d.ink_fill > d.ink_stroke and d.kept_strokes == 0
    assert d.dropped['light'] >= 100 and d.dropped['speck'] >= 1
    x0, y0, x1, y1 = ROOM
    t = m2pt(0.2)
    assert bbox_of(ext.geometry) == pytest.approx(
        (x0 - t / 2, H - y1 - t / 2, x1 + t / 2, H - y0 + t / 2), abs=0.5)


def test_poche_room_seals_its_door_and_matches_the_stroke_room(poche_pdf, stroke_pdf):
    results = []
    for pdf in (poche_pdf, stroke_pdf):
        ext = ps.extract_walls(pdf, 0)
        tf = calib.map_frame(M_PT, 0.0, (ROOM[0], H - ROOM[3]))
        grid, _ = pr.rasterize(ext.geometry, tf, 0.05)
        results.append((int((grid == pr.FREE).sum()), int((grid == pr.OCCUPIED).sum())))
    (free_p, occ_p), (free_s, occ_s) = results
    assert free_p == pytest.approx(free_s, rel=0.05) and free_p > 20000
    assert occ_p == pytest.approx(2 * occ_s, rel=0.15)


def test_a_label_sitting_on_a_wall_does_not_remove_the_wall():
    def sheet(page):
        draw_frame_and_title(page)
        page.draw_rect(pymupdf.Rect(1000, 600, 1009, 900), color=BLACK, fill=(0.29,) * 3,
                       width=0.6)
        page.draw_rect(pymupdf.Rect(960, 740, 1050, 760), fill=(1, 1, 1))
        page.insert_text((970, 755), 'SEATING', fontsize=14)

    pdf = ps.open_pdf(make_pdf(sheet))
    ext = ps.extract_walls(pdf, 0)
    assert ext.kind == 'vector' and ext.diagnostics.kept_fills == 1
    assert bbox_of(ext.geometry) == pytest.approx((1000, H - 900, 1009, H - 600), abs=0.5)
    ps.close_pdf(pdf)


def test_text_mask_drops_marks_that_lie_under_a_word_and_can_be_turned_off(poche_pdf):
    on = ps.extract_walls(poche_pdf, 0)
    off = ps.extract_walls(poche_pdf, 0, ps.WallOptions(mask_text=False, min_fill_area=8))
    assert off.diagnostics.kept_fills == on.diagnostics.kept_fills + 1
    assert on.diagnostics.dropped.get('text', 0) == 1


def test_transparent_and_cmyk_fills_are_read_sensibly():
    def sheet(page):
        draw_frame_and_title(page)
        page.draw_rect(pymupdf.Rect(700, 600, 760, 900), fill=BLACK, fill_opacity=0.1, width=0)
        page.draw_rect(pymupdf.Rect(900, 600, 960, 900), fill=BLACK, width=0)

    pdf = ps.open_pdf(make_pdf(sheet))
    ext = ps.extract_walls(pdf, 0)
    assert ext.diagnostics.kept_fills == 1
    ps.close_pdf(pdf)


# --- layers ---------------------------------------------------------------------------------

def layered_pdf(names, hidden=()):
    doc = pymupdf.open()
    page = doc.new_page(width=W, height=H)
    xrefs = {n: doc.add_ocg(n, on=n not in hidden) for n in names}
    draw_frame_and_title(page)
    for i, xref in enumerate(xrefs.values()):
        y = 500 + 150 * i
        page.draw_line((600, y), (1400, y), color=BLACK, width=WALL_PT, oc=xref)
    page.draw_line((600, 1500), (1400, 1500), color=BLACK, width=WALL_PT)
    return ps.open_pdf(doc.tobytes())


def test_wall_layers_are_selected_by_name_and_the_rest_ignored():
    pdf = layered_pdf(['A-WALL', 'A-ANNO-DIMS', 'A-WALL-DIMS', 'FURNITURE'])
    ext = ps.extract_walls(pdf, 0)
    d = ext.diagnostics
    assert d.layer_mode == 'allowlist' and d.kept_strokes == 1
    assert 'A-WALL' in d.layers and d.dropped['layer'] >= 4
    ps.close_pdf(pdf)


def test_wall_layer_allowlist_is_configurable():
    pdf = layered_pdf(['A-WALL', 'XYZ-STUFF'])
    ext = ps.extract_walls(pdf, 0, ps.WallOptions(wall_layers=(r'xyz',)))
    assert ext.diagnostics.layer_mode == 'allowlist' and ext.diagnostics.kept_strokes == 1
    assert ext.geometry.polylines[0].points[0][1] == pytest.approx(H - 650)
    ps.close_pdf(pdf)


def test_hidden_layers_are_not_drawn():
    pdf = layered_pdf(['A-WALL', 'A-WALL-2'], hidden=('A-WALL-2',))
    assert ps.extract_walls(pdf, 0).diagnostics.kept_strokes == 1
    ps.close_pdf(pdf)


def test_layers_without_wall_names_fall_back_to_the_classifier_minus_skip_layers():
    pdf = layered_pdf(['LAYER-1', 'DIMS'])
    d = ps.extract_walls(pdf, 0).diagnostics
    assert d.layer_mode == 'fallback' and d.dropped['layer'] == 1
    assert d.kept_strokes == 2
    ps.close_pdf(pdf)


# --- coordinate conventions: geometry vs the PNG the UI shows ------------------------------

def asymmetric_sheet(page):
    page.draw_line((300, 200), (300, 700), color=BLACK, width=10)
    page.draw_line((300, 700), (900, 700), color=BLACK, width=10)
    page.draw_line((1100, 400), (1100, 460), color=BLACK, width=10)


def seg_distance(p, a, b):
    ab = b - a
    t = np.clip(((p - a) @ ab) / max(ab @ ab, 1e-12), 0, 1)
    return np.hypot(*(p - (a + t * ab)))


@pytest.mark.parametrize('rotation', [0, 90, 180, 270])
@pytest.mark.parametrize('userunit', [None, 2])
def test_geometry_and_png_agree_under_the_documented_mapping(rotation, userunit):
    pdf = ps.open_pdf(make_pdf(asymmetric_sheet, rotation=rotation, userunit=userunit,
                               width=1200, height=900))
    ext = ps.extract_walls(pdf, 0, ps.WallOptions(width_floor=1.0))
    png = ps.render_page_png(pdf, 0)
    img = decode_png(png.png)
    assert img.shape == (png.height_px, png.width_px)
    assert png.scale_x == pytest.approx(png.width_px / png.width)

    def to_px(pt):
        return pt[0] * png.scale_x, (png.height - pt[1]) * png.scale_y

    for pl in ext.geometry.polylines:
        for a, b in zip(pl.points[:-1], pl.points[1:], strict=True):
            for t in np.linspace(0.05, 0.95, 12):
                px, py = to_px(a + t * (b - a))
                assert img[int(py), int(px)] < 90, (rotation, userunit, a, b, t)
    rows, cols = np.nonzero(img < 90)
    segs = [(a, b) for pl in ext.geometry.polylines
            for a, b in zip(pl.points[:-1], pl.points[1:], strict=True)]
    rng = np.random.default_rng(3)
    for k in rng.choice(len(rows), size=120, replace=False):
        p = np.array([(cols[k] + 0.5) / png.scale_x, png.height - (rows[k] + 0.5) / png.scale_y])
        assert min(seg_distance(p, a, b) for a, b in segs) < 5.0 + 3 / png.scale_x
    ps.close_pdf(pdf)


# --- PNG ------------------------------------------------------------------------------------

def test_png_is_a_png_with_the_returned_scale(stroke_pdf):
    png = ps.render_page_png(stroke_pdf, 0)
    assert png.png.startswith(b'\x89PNG')
    assert (png.width, png.height) == (W, H)
    assert png.width_px == pytest.approx(W * ps.DEFAULT_PNG_PX_PER_PT, abs=1)
    assert png.scale_x == pytest.approx(1.5, abs=0.01)
    assert png.scale_y == pytest.approx(1.5, abs=0.01)


def test_png_pixel_cap_clamps_the_scale_not_the_page(stroke_pdf):
    assert ps.MAX_PNG_PIXELS == 25_000_000
    png = ps.render_page_png(stroke_pdf, 0, px_per_pt=10.0, max_pixels=1_000_000)
    assert png.width_px * png.height_px <= 1_000_000 * 1.01
    assert png.scale_x == pytest.approx(png.width_px / W)
    assert png.scale_x < 1.0


@pytest.mark.parametrize('bad', [0, -1, float('nan'), float('inf'), 'x', None])
def test_bad_png_scale_is_refused(stroke_pdf, bad):
    with pytest.raises(ps.OptionsError):
        ps.render_page_png(stroke_pdf, 0, px_per_pt=bad)


# --- options ---------------------------------------------------------------------------------

@pytest.mark.parametrize('kwargs', [
    {'mode': 'walls'}, {'max_luminance': -1}, {'heavy_ratio': float('nan')},
    {'min_stroke_width': -1}, {'min_stroke_width': float('inf')}, {'curve_segments': 0},
    {'curve_segments': 1000}, {'region': (0, 0, 0, 10)}, {'region': (0, 0, 10)},
    {'region': (0, 0, float('nan'), 10)}, {'text_pad': 'x'},
])
def test_bad_wall_options_are_refused(stroke_pdf, kwargs):
    with pytest.raises(ps.OptionsError):
        ps.extract_walls(stroke_pdf, 0, ps.WallOptions(**kwargs))


def test_extraction_path_cap(monkeypatch, stroke_pdf):
    monkeypatch.setattr(ps, 'MAX_DRAWINGS', 5)
    with pytest.raises(ps.ExtractionError):
        ps.extract_walls(stroke_pdf, 0)


# --- raster fallback -------------------------------------------------------------------------

@pytest.fixture(scope='module')
def raster_pdf():
    pdf = ps.open_pdf(image_only_pdf(make_pdf(stroke_sheet), dpi=100))
    yield pdf
    ps.close_pdf(pdf)


def test_raster_page_walls_come_from_dark_pixels(raster_pdf):
    assert ps.extract_walls(raster_pdf, 0).kind == 'none'
    ext = ps.extract_raster_walls(
        raster_pdf, 0, M_PT, 0.05,
        ps.RasterOptions(region=(500, 800, 1300, 1260), dark_threshold=100))
    d = ext.diagnostics
    assert ext.kind == 'raster' and len(ext.geometry.masks) == 1
    assert d.oversample == pytest.approx(4.0, rel=0.05) and not d.clamped
    assert d.pixel_m == pytest.approx(0.0125, rel=0.05)
    x0, y0, x1, y1 = d.bbox
    assert (x1 - x0) * M_PT == pytest.approx(10.1, abs=0.3)
    assert (y1 - y0) * M_PT == pytest.approx(6.1, abs=0.3)


def test_raster_room_rasterizes_like_the_vector_room(raster_pdf, stroke_pdf):
    vec = ps.extract_walls(stroke_pdf, 0)
    ras = ps.extract_raster_walls(
        raster_pdf, 0, M_PT, 0.05,
        ps.RasterOptions(region=(500, 800, 1300, 1260), min_thickness_m=0.06))
    out = []
    for ext in (vec, ras):
        x0, y0, _, _ = ext.diagnostics.bbox
        grid, _ = pr.rasterize(ext.geometry, calib.map_frame(M_PT, 0.0, (x0, y0)), 0.05)
        out.append((int((grid == pr.FREE).sum()), int((grid == pr.OCCUPIED).sum())))
    assert out[1][0] == pytest.approx(out[0][0], rel=0.05)
    assert out[1][1] == pytest.approx(out[0][1], rel=0.35)


def test_raster_without_a_region_includes_the_clutter_but_trims_the_page_edge(raster_pdf):
    ext = ps.extract_raster_walls(raster_pdf, 0, M_PT, 0.05)
    x0, y0, x1, y1 = ext.diagnostics.bbox
    assert x0 > 0.03 * W - 2 and y0 > 0.03 * H - 2 and x1 < W * 0.97 + 2 and y1 < H * 0.97 + 2
    assert x1 > 2300
    edge = ps.extract_raster_walls(raster_pdf, 0, M_PT, 0.05, ps.RasterOptions(margin_fraction=0))
    assert edge.diagnostics.bbox[0] < 0.03 * W


def test_min_thickness_opening_removes_hairlines_and_keeps_walls(raster_pdf):
    opts = dict(region=(500, 800, 1300, 1260))
    plain = ps.extract_raster_walls(raster_pdf, 0, M_PT, 0.05, ps.RasterOptions(**opts))
    opened = ps.extract_raster_walls(raster_pdf, 0, M_PT, 0.05,
                                     ps.RasterOptions(min_thickness_m=0.06, **opts))
    assert opened.kind == 'raster'
    assert opened.diagnostics.dark_pixels < plain.diagnostics.dark_pixels
    assert opened.diagnostics.opened_px > 0
    too_thick = ps.extract_raster_walls(raster_pdf, 0, M_PT, 0.05,
                                        ps.RasterOptions(min_thickness_m=2.0, **opts))
    assert too_thick.kind == 'none'


def test_pixel_budget_clamps_the_render_and_reports_it(raster_pdf):
    ext = ps.extract_raster_walls(raster_pdf, 0, M_PT, 0.05, ps.RasterOptions(max_pixels=400_000))
    d = ext.diagnostics
    assert d.clamped and d.width_px * d.height_px <= 410_000
    assert ext.diagnostics.oversample < 4.0
    assert ps.MAX_RASTER_PIXELS == 36_000_000 and ps.RASTER_OVERSAMPLE == 4.0


def test_blank_raster_page_has_no_walls():
    doc = pymupdf.open()
    page = doc.new_page(width=600, height=400)
    pix = pymupdf.Pixmap(pymupdf.csGRAY, pymupdf.IRect(0, 0, 60, 40), False)
    pix.clear_with(255)
    page.insert_image(page.rect, pixmap=pix)
    pdf = ps.open_pdf(doc.tobytes())
    ext = ps.extract_raster_walls(pdf, 0, 0.02, 0.05)
    assert ext.kind == 'none' and ext.reason
    ps.close_pdf(pdf)


@pytest.mark.parametrize('args, opts', [
    ((0, 0.05), {}), ((-1, 0.05), {}), ((float('nan'), 0.05), {}), ((M_PT, 0), {}),
    ((M_PT, float('inf')), {}), ((M_PT, 0.05), {'dark_threshold': 0}),
    ((M_PT, 0.05), {'dark_threshold': 300}), ((M_PT, 0.05), {'oversample': 0}),
    ((M_PT, 0.05), {'min_thickness_m': -1}), ((M_PT, 0.05), {'margin_fraction': 0.6}),
    ((M_PT, 0.05), {'region': (5, 5, 1, 1)}), ((M_PT, 0.05), {'max_pixels': 0}),
])
def test_bad_raster_arguments_are_refused(raster_pdf, args, opts):
    with pytest.raises(ps.OptionsError):
        ps.extract_raster_walls(raster_pdf, 0, *args, ps.RasterOptions(**opts))


# --- sample-backed (real A-101 sheets; skipped when absent) --------------------------------

SAMPLES = pathlib.Path(os.environ.get(
    'SCOUT_FLOORPLAN_SAMPLES', '/Users/cdrew/Documents/GitHub/Scout/docs/floorplans'))
SAMPLE_FILES = sorted(SAMPLES.glob('*.pdf')) if SAMPLES.is_dir() else []
EXISTING = SAMPLES / 'the_zone_A-101_existing_conditions.pdf'
BAR_GRAY = 74
OTHER_REVS = ['the_zone_A-101_existing_conditions.pdf', 'the_zone_A-101_rev1.pdf',
              'the_zone_A-101_rev2.pdf', 'the_zone_A-101_rev3.pdf']


def need(path):
    if not path.is_file():
        pytest.skip(f'sample sheet not present: {path}')
    return ps.open_pdf(path.read_bytes())


@pytest.mark.parametrize('path', SAMPLE_FILES or [None], ids=lambda p: p.name if p else 'none')
def test_every_sample_opens_and_yields_walls_or_demands_calibration(path):
    if path is None:
        pytest.skip(f'no sample sheets in {SAMPLES}')
    pdf = ps.open_pdf(path.read_bytes())
    for info in ps.list_pages(pdf):
        rep = ps.propose_page_scale(pdf, info.index)
        ext = ps.extract_walls(pdf, info.index)
        assert ext.kind == 'vector' or rep.calibration_required or ext.kind == 'none'
        if ext.kind == 'vector':
            assert ext.geometry.polylines or ext.geometry.fills
            assert ext.diagnostics.bbox is not None
        else:
            assert rep.calibration_required and ext.reason
    ps.close_pdf(pdf)


def test_a101_proposes_quarter_inch_scale_on_arch_d():
    pdf = need(EXISTING)
    rep = ps.propose_page_scale(pdf, 0)
    text = ' | '.join(rep.proposal.evidence)
    assert rep.proposal.m_per_point == pytest.approx(M_PT, rel=1e-9)
    assert '1/4"' in text and 'ARCH D' in text and rep.proposal.confidence == calib.HIGH
    ps.close_pdf(pdf)


def bar_pixel_truth(pdf, png):
    """Wall-bar interiors found by their flat fill colour in the rendered sheet (independent of
    the drawing classifier; a flat 3x3 neighbourhood rejects anti-aliased text edges that also
    pass through that gray); the legend swatch in the title block (right 10 %) is excluded."""
    img = decode_png(png.png)
    flat = np.abs(img - BAR_GRAY) <= 4
    truth = flat.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            truth &= np.roll(np.roll(flat, dy, axis=0), dx, axis=1)
    truth[:, int(0.9 * png.width_px):] = False
    return truth


def grid_of(pdf, origin=(0.0, 0.0)):
    rep = ps.propose_page_scale(pdf, 0)
    ext = ps.extract_walls(pdf, 0)
    tf = calib.map_frame(rep.proposal.m_per_point, 0.0, origin)
    grid, meta = pr.rasterize(ext.geometry, tf, 0.05)
    return rep, ext, grid, meta


def truth_cells(png, truth, rep, meta, shape, origin):
    """Cells under the truth pixels, through the same map frame the grid was built with."""
    rows, cols = np.nonzero(truth)
    m = rep.proposal.m_per_point
    x = ((cols + 0.5) / png.scale_x - origin[0]) * m
    y = ((png.height - (rows + 0.5) / png.scale_y) - origin[1]) * m
    c = np.floor((x - meta['origin'][0]) / 0.05).astype(int)
    r = shape[0] - 1 - np.floor((y - meta['origin'][1]) / 0.05).astype(int)
    ok = (c >= 0) & (c < shape[1]) & (r >= 0) & (r < shape[0])
    cells = np.zeros(shape, bool)
    cells[r[ok], c[ok]] = True
    return cells


def grow(mask, n=1):
    out = mask.copy()
    for dy in range(-n, n + 1):
        for dx in range(-n, n + 1):
            out |= np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
    return out


def test_a101_grid_extent_matches_the_wall_bars_footprint_within_5_percent():
    """Footprint = bounding box of the pixels carrying the wall-bar fill colour in the rendered
    page (not the drawing classifier), converted with the proposed scale. This checks the
    geometry -> grid chain (flip, scale, origin), not whether the classifier chose the right
    paths; the pixel-overlap test below covers that."""
    pdf = need(EXISTING)
    png = ps.render_page_png(pdf, 0)
    truth = bar_pixel_truth(pdf, png)
    rows, cols = np.nonzero(truth)
    m = ps.propose_page_scale(pdf, 0).proposal.m_per_point
    foot_w = (cols.max() - cols.min() + 1) / png.scale_x * m
    foot_h = (rows.max() - rows.min() + 1) / png.scale_y * m
    _, ext, grid, meta = grid_of(pdf, origin=(ext_origin(pdf)))
    occ_rows, occ_cols = np.nonzero(grid == pr.OCCUPIED)
    grid_w = (occ_cols.max() - occ_cols.min() + 1) * 0.05
    grid_h = (occ_rows.max() - occ_rows.min() + 1) * 0.05
    assert grid_w == pytest.approx(foot_w, rel=0.05) and grid_h == pytest.approx(foot_h, rel=0.05)
    ps.close_pdf(pdf)


def ext_origin(pdf):
    x0, y0, _, _ = ps.extract_walls(pdf, 0).diagnostics.bbox
    return (x0, y0)


@pytest.mark.parametrize('name', OTHER_REVS)
def test_a101_occupied_cells_overlap_the_wall_bar_pixels(name):
    pdf = need(SAMPLES / name)
    png = ps.render_page_png(pdf, 0)
    truth = bar_pixel_truth(pdf, png)
    origin = ext_origin(pdf)
    _, _, grid, meta = grid_of(pdf, origin=origin)
    rep = ps.propose_page_scale(pdf, 0)
    cells = truth_cells(png, truth, rep, meta, grid.shape, origin)
    occupied = grid == pr.OCCUPIED
    recall = (cells & grow(occupied)).sum() / cells.sum()
    precision = (occupied & grow(cells)).sum() / occupied.sum()
    assert recall >= 0.97 and precision >= 0.95, (name, recall, precision)
    ps.close_pdf(pdf)


def test_a101_stroke_sheet_picks_the_heavy_wall_class():
    pdf = need(SAMPLES / 'the_zone_floorplan.pdf')
    d = ps.extract_walls(pdf, 0).diagnostics
    assert d.mode == 'strokes' and d.kept_strokes == 20
    assert [c['width'] for c in d.stroke_classes if c['kept']] == [1.4]
    ps.close_pdf(pdf)
