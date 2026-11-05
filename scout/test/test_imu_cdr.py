"""gyro_calibrator hot path: golden equivalence of every /imu/data field.

_Old is the pre-optimisation node logic (object path: deserialize, mutate fields,
publish) verbatim; the new path is GyroBias + ImuPatcher on raw CDR bytes. A
synthetic 10 s 200 Hz stream (still, then a pivot, then still) is run through
both and every field of every published message is compared with exact equality.
"""

import math
import random
import struct

from scout.core.gyro_bias import GyroBias
from scout.core.imu_cdr import ImuPatcher

V = 7.0e-6
FIELDS = ('sec', 'nanosec', 'frame_id', 'orient', 'orient_cov', 'ang', 'ang_cov',
          'lin', 'lin_cov')


def encode(m):
    """Independent CDR-LE encoder for the Imu dict (alignment relative to byte 4)."""
    fid = m['frame_id'].encode() + b'\0'
    b = bytearray(b'\x00\x01\x00\x00')
    b += struct.pack('<iII', m['sec'], m['nanosec'], len(fid)) + fid
    while (len(b) - 4) % 8:
        b.append(0)
    for k, n in (('orient', 4), ('orient_cov', 9), ('ang', 3), ('ang_cov', 9),
                 ('lin', 3), ('lin_cov', 9)):
        b += struct.pack('<%dd' % n, *m[k])
    return bytes(b)


def decode(raw):
    sec, nsec, n = struct.unpack_from('<iII', raw, 4)
    fid = raw[16:16 + n - 1].decode()
    off = 4 + ((16 + n - 4 + 7) & ~7)
    m = {'sec': sec, 'nanosec': nsec, 'frame_id': fid}
    for k, c in (('orient', 4), ('orient_cov', 9), ('ang', 3), ('ang_cov', 9),
                 ('lin', 3), ('lin_cov', 9)):
        m[k] = struct.unpack_from('<%dd' % c, raw, off)
        off += 8 * c
    assert off == len(raw)
    return m


class _Old:
    def __init__(self, cal=5.0, still=0.02, refresh=10.0, weight=0.25):
        self.cal, self.still, self.refresh, self.weight = cal, still, refresh, weight
        self._bias = None
        self._sum = [0.0, 0.0, 0.0]
        self._count = 0
        self._t0 = None

    def _acc(self, g, ns):
        if self._t0 is None:
            self._t0 = ns
        for i in range(3):
            self._sum[i] += g[i]
        self._count += 1
        return (ns - self._t0) * 1e-9

    def _reset(self):
        self._sum = [0.0, 0.0, 0.0]
        self._count = 0
        self._t0 = None

    def on_imu(self, m, ns):
        g = m['ang']
        if self._bias is None:
            if self._acc(g, ns) >= self.cal:
                self._bias = [s / self._count for s in self._sum]
                self._reset()
            return None
        c = (g[0] - self._bias[0], g[1] - self._bias[1], g[2] - self._bias[2])
        if max(abs(v) for v in c) < self.still:
            if self._acc(g, ns) >= self.refresh:
                w = [s / self._count for s in self._sum]
                self._bias = [b + self.weight * (x - b)
                              for b, x in zip(self._bias, w, strict=True)]
                self._reset()
        elif self._count:
            self._reset()
        out = dict(m)
        out['ang'] = c
        out['ang_cov'] = (V, 0.0, 0.0, 0.0, V, 0.0, 0.0, 0.0, V)
        oc = list(m['orient_cov'])
        oc[0] = -1.0
        out['orient_cov'] = tuple(oc)
        return out


def _stream(frame_id='camera_imu_optical_frame', n=2000, seed=1):
    rng = random.Random(seed)
    for i in range(n):
        t = i / 200.0
        yaw = 0.0
        if 6.0 <= t < 8.0:
            yaw = 1.5 * math.sin(math.pi * (t - 6.0) / 2.0)
        m = {'sec': 100 + int(t), 'nanosec': int((t % 1) * 1e9),
             'frame_id': frame_id,
             'orient': (0.0, 0.0, 0.0, 0.0),
             'orient_cov': (0.0,) * 9,
             'ang': (-0.0044 + rng.gauss(0, 0.002),
                     -0.0036 - yaw + rng.gauss(0, 0.002),
                     -0.0022 + rng.gauss(0, 0.002)),
             'ang_cov': (0.0,) * 9,
             'lin': (rng.gauss(0, .05), -9.64 + rng.gauss(0, .05), rng.gauss(0, .05)),
             'lin_cov': (0.0,) * 9}
        yield m, int(t * 1e9) + 5_000_000


def _run_new(stream):
    est = GyroBias(5.0, 0.02, 10.0, 0.25)
    pt = ImuPatcher(V)
    outs = []
    for m, ns in stream:
        raw = encode(m)
        lay = pt.layout(raw)
        assert lay is not None
        c = est.update(*pt.angular_velocity(raw, lay), ns)
        outs.append(None if c is None else decode(pt.patch(raw, lay, *c)))
    return outs, est


def test_golden_all_fields_exact():
    for fid in ('camera_imu_optical_frame', 'imu', 'a', 'camera_gyro_optical_frame_x'):
        old = _Old()
        want = [old.on_imu(m, ns) for m, ns in _stream(fid)]
        got, est = _run_new(_stream(fid))
        assert [w is None for w in want] == [g is None for g in got]
        assert sum(g is None for g in got) > 900   # 5 s startup window at 200 Hz
        n = 0
        for w, g in zip(want, got, strict=True):
            if w is None:
                continue
            for k in FIELDS:
                assert g[k] == w[k], (fid, k)
            n += 1
        assert n > 900
        assert list(est.bias) == old._bias


def test_unrecognised_layouts_fall_back():
    pt = ImuPatcher(V)
    good = encode(next(_stream())[0])
    assert pt.layout(b'\x00\x00' + good[2:]) is None     # big-endian CDR
    assert pt.layout(good[:100]) is None                 # truncated
    assert pt.layout(b'') is None
    assert pt.layout(good) is not None
