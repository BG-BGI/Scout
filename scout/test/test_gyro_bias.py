"""GyroBias startup/refresh behaviour (golden equivalence lives in test_imu_cdr)."""

from scout.core.gyro_bias import GyroBias


def test_no_publish_during_startup_window():
    est = GyroBias(5.0, 0.02, 10.0, 0.25)
    ns = [est.update(0.1, 0.1, 0.1, i * 5_000_000) for i in range(999)]
    assert all(v is None for v in ns)


def test_bias_then_correction():
    est = GyroBias(1.0, 0.02, 10.0, 0.25)
    out = None
    for i in range(300):
        out = est.update(0.01, 0.0, -0.01, i * 5_000_000)
    assert all(abs(a - b) < 1e-12 for a, b in zip(est.bias, (0.01, 0.0, -0.01), strict=True))
    assert all(abs(c) < 1e-12 for c in out)
