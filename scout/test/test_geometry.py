import math

import numpy as np

from scout.core import geometry as g


def test_wrap_angle_range_and_periodicity():
    assert math.isclose(g.wrap_angle(0.0), 0.0)
    assert math.isclose(g.wrap_angle(1.5 * math.pi), -0.5 * math.pi)
    # +/- 2pi is a no-op (to floating tolerance).
    for a in (0.3, -2.0, 3.0, -3.1):
        assert math.isclose(g.wrap_angle(a + 2 * math.pi), g.wrap_angle(a), abs_tol=1e-9)
    # Result always in [-pi, pi).
    for a in np.linspace(-20, 20, 200):
        w = g.wrap_angle(a)
        assert -math.pi - 1e-9 <= w < math.pi + 1e-9


def test_planar_yaw_roundtrip():
    for yaw in (0.0, 0.5, -0.5, 1.2, -3.0, 3.0):
        z, w = g.yaw_to_quat_zw(yaw)
        assert math.isclose(g.planar_yaw(z, w), yaw, abs_tol=1e-9)


def test_quat_to_matrix_identity_and_orthonormal():
    r = g.quat_to_matrix(0.0, 0.0, 0.0, 1.0)
    assert np.allclose(r, np.eye(3), atol=1e-6)
    # 90 deg about +z maps x -> y.
    z, w = g.yaw_to_quat_zw(math.pi / 2)
    r = g.quat_to_matrix(0.0, 0.0, z, w)
    assert np.allclose(r @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-6)
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-6)
    assert r.dtype == np.float32


def test_base_anchor_roundtrip_scalar_and_vector():
    pose = (1.0, -2.0, 0.7)
    ax, ay = g.base_to_anchor(pose, 0.5, 0.3)
    bx, by = g.anchor_to_base(pose, ax, ay)
    assert math.isclose(bx, 0.5, abs_tol=1e-9) and math.isclose(by, 0.3, abs_tol=1e-9)
    # Known 90 deg pose: base +x -> anchor +y.
    ax, ay = g.base_to_anchor((0.0, 0.0, math.pi / 2), 1.0, 0.0)
    assert math.isclose(ax, 0.0, abs_tol=1e-9) and math.isclose(ay, 1.0, abs_tol=1e-9)
    # Vectorized.
    xs, ys = np.array([1.0, 0.0]), np.array([0.0, 1.0])
    axs, ays = g.base_to_anchor((0.0, 0.0, math.pi / 2), xs, ys)
    assert np.allclose(axs, [0.0, -1.0], atol=1e-9)
    assert np.allclose(ays, [1.0, 0.0], atol=1e-9)
