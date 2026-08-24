"""scout.core.cliff — negative-obstacle math (1:1 with core/cliff.py, ADR-0012).

Geometry used throughout: camera level at CAM_H above the floor, identity
rotation camera->base except where a synthetic optical rotation is exercised,
so test clouds can be authored directly in base-frame coordinates.
"""

import numpy as np

from scout.core.cliff import CliffMemory, find_cliff_cells, parse_xyz, stop_gate

CAM_H = 0.205
IDENTITY = np.eye(3, dtype=np.float32)
CAM_T = np.array([0.14, 0.0, CAM_H], dtype=np.float32)

PARAMS = dict(min_range=0.4, max_range=2.0, drop_base=0.05, drop_slope=0.02,
              max_drop=1.5, cell_size=0.05, min_points=3)


def _to_cam(points_base):
    """Base-frame points -> camera frame for the identity-rotation camera."""
    return (np.asarray(points_base, dtype=np.float32) - CAM_T)


def _cloud_bytes(xyz, point_step=16):
    """Serialize Nx3 float32 into a PointCloud2-style buffer with padding."""
    xyz = np.asarray(xyz, dtype=np.float32)
    raw = np.zeros((len(xyz), point_step), dtype=np.uint8)
    raw[:, :12] = xyz.astype('<f4').view(np.uint8).reshape(len(xyz), 12)
    return raw.tobytes()


# --- parse_xyz ---------------------------------------------------------------

def test_parse_xyz_roundtrip_with_padding():
    pts = np.array([[1.0, 2.0, 3.0], [-0.5, 0.25, -1.5]], dtype=np.float32)
    out = parse_xyz(_cloud_bytes(pts), point_step=16)
    assert np.allclose(out, pts)


def test_parse_xyz_drops_non_finite():
    pts = np.array([[1.0, 2.0, 3.0], [np.nan, 0.0, 0.0], [0.0, np.inf, 1.0]],
                   dtype=np.float32)
    out = parse_xyz(_cloud_bytes(pts), point_step=16)
    assert len(out) == 1 and np.allclose(out, pts[:1])


def test_parse_xyz_empty():
    assert parse_xyz(b'', point_step=16).shape == (0, 3)


# --- find_cliff_cells --------------------------------------------------------

def test_flat_floor_yields_nothing():
    xs = np.linspace(0.45, 1.9, 200)
    floor = np.column_stack([xs, np.zeros_like(xs), np.zeros_like(xs)])
    cells = find_cliff_cells(_to_cam(floor), IDENTITY, CAM_T, **PARAMS)
    assert len(cells) == 0


def test_step_down_marks_the_lip_not_the_tread():
    # A 0.17 m step: floor to x=1.0, tread at z=-0.17 beyond it. Returns on
    # the tread land at x >= ~1.0 + a bit; the lip projection must pull the
    # mark back to ~the ray's floor-plane crossing, short of the tread hits.
    tread_x = np.linspace(1.1, 1.4, 30)
    tread = np.column_stack([tread_x, np.zeros_like(tread_x),
                             np.full_like(tread_x, -0.17)])
    cells = find_cliff_cells(_to_cam(tread), IDENTITY, CAM_T, **PARAMS)
    assert len(cells) > 0
    # Every mark sits at the plane crossing (between camera and tread),
    # strictly short of the tread returns themselves.
    assert cells[:, 0].max() < 1.1
    assert cells[:, 0].min() > 0.4
    # Analytic check for the nearest tread point: s = c_z/(c_z - p_z).
    s = CAM_H / (CAM_H + 0.17)
    expected_x = CAM_T[0] + s * (1.1 - CAM_T[0])
    assert abs(cells[:, 0].min() - expected_x) < PARAMS['cell_size']


def test_shallow_dip_below_threshold_ignored():
    # 3 cm dip at 1 m: drop(1.0) = 0.05 + 0.02 = 0.07 -> not a cliff.
    xs = np.linspace(0.9, 1.1, 40)
    dip = np.column_stack([xs, np.zeros_like(xs), np.full_like(xs, -0.03)])
    cells = find_cliff_cells(_to_cam(dip), IDENTITY, CAM_T, **PARAMS)
    assert len(cells) == 0


def test_drop_threshold_scales_with_range():
    # z=-0.06 clears drop(0.45)=0.059 up close but not drop(1.8)=0.086 far out.
    near = np.array([[0.5, 0.0, -0.062]] * 5)
    far = np.array([[1.8, 0.0, -0.062]] * 5)
    assert len(find_cliff_cells(_to_cam(near), IDENTITY, CAM_T, **PARAMS)) > 0
    assert len(find_cliff_cells(_to_cam(far), IDENTITY, CAM_T, **PARAMS)) == 0


def test_below_max_drop_rejected():
    # Returns 2 m below the floor are not a stair — reflections/glass noise.
    deep = np.array([[1.0, 0.0, -2.0]] * 5)
    cells = find_cliff_cells(_to_cam(deep), IDENTITY, CAM_T, **PARAMS)
    assert len(cells) == 0


def test_min_points_per_cell_rejects_speckle():
    lone = np.array([[1.0, 0.0, -0.3]])          # single below-floor speckle
    cells = find_cliff_cells(_to_cam(lone), IDENTITY, CAM_T, **PARAMS)
    assert len(cells) == 0


def test_range_gate():
    too_close = np.array([[0.2, 0.0, -0.3]] * 5)
    too_far = np.array([[3.0, 0.0, -0.3]] * 5)
    assert len(find_cliff_cells(_to_cam(too_close), IDENTITY, CAM_T,
                                **PARAMS)) == 0
    assert len(find_cliff_cells(_to_cam(too_far), IDENTITY, CAM_T,
                                **PARAMS)) == 0


def test_optical_frame_rotation_handled():
    # Same tread, but expressed in a camera OPTICAL frame (z forward, x right,
    # y down) with the matching rotation matrix — results must agree with the
    # identity-frame run.
    tread_x = np.linspace(1.1, 1.4, 30)
    tread = np.column_stack([tread_x, np.zeros_like(tread_x),
                             np.full_like(tread_x, -0.17)])
    # base <- optical basis columns: optical z -> base x, -x -> base y, -y -> base z
    rot = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32)
    cam_optical = (tread - CAM_T) @ rot            # inverse = transpose
    ref = find_cliff_cells(_to_cam(tread), IDENTITY, CAM_T, **PARAMS)
    out = find_cliff_cells(cam_optical.astype(np.float32), rot, CAM_T, **PARAMS)
    assert np.allclose(np.sort(ref, axis=0), np.sort(out, axis=0), atol=1e-3)


# --- CliffMemory -------------------------------------------------------------

def test_memory_persists_and_expires():
    mem = CliffMemory(cell_size=0.05, ttl_s=10.0, max_cells=100)
    mem.add(np.array([[1.0, 0.5]]), now=0.0)
    assert len(mem.cells()) == 1
    mem.add(np.empty((0, 2)), now=9.0)             # still inside TTL
    assert len(mem.cells()) == 1
    mem.add(np.empty((0, 2)), now=10.5)            # expired
    assert len(mem.cells()) == 0


def test_memory_refresh_extends_ttl():
    mem = CliffMemory(cell_size=0.05, ttl_s=10.0, max_cells=100)
    mem.add(np.array([[1.0, 0.5]]), now=0.0)
    mem.add(np.array([[1.0, 0.5]]), now=8.0)       # re-seen: clock restarts
    mem.add(np.empty((0, 2)), now=15.0)
    assert len(mem.cells()) == 1


def test_memory_cap_drops_oldest():
    mem = CliffMemory(cell_size=0.05, ttl_s=1000.0, max_cells=3)
    for i in range(5):
        mem.add(np.array([[float(i), 0.0]]), now=float(i))
    cells = mem.cells()
    assert len(cells) == 3
    assert cells[:, 0].min() > 1.5                 # cells 0 and 1 dropped


def test_memory_dedupes_within_a_cell():
    mem = CliffMemory(cell_size=0.05, ttl_s=10.0, max_cells=100)
    mem.add(np.array([[1.001, 0.5], [1.002, 0.5], [1.004, 0.5]]), now=0.0)
    assert len(mem.cells()) == 1


# --- stop_gate ---------------------------------------------------------------

def test_stop_gate_corridor():
    assert stop_gate(np.array([[0.4, 0.1]]), stop_x=0.6, half_width=0.25)
    assert not stop_gate(np.array([[0.8, 0.0]]), stop_x=0.6, half_width=0.25)  # too far
    assert not stop_gate(np.array([[0.4, 0.4]]), stop_x=0.6, half_width=0.25)  # off side
    assert not stop_gate(np.array([[-0.2, 0.0]]), stop_x=0.6, half_width=0.25)  # behind
    assert not stop_gate(np.empty((0, 2)), stop_x=0.6, half_width=0.25)
