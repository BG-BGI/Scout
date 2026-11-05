"""Byte-level patching of a serialized sensor_msgs/Imu (CDR little-endian).

gyro_calibrator changes only three doubles (angular_velocity), the 9-double
angular_velocity_covariance and orientation_covariance[0]. Everything else is
forwarded verbatim, so the node takes the message raw and patches those bytes
instead of deserializing, rebuilding and re-serializing an Imu per sample
(profiled on the Pi: that was ~40% of the node's GIL time at 200 Hz).

Layout, offsets from the start of the buffer (4-byte encapsulation header, then
CDR alignment is relative to buffer offset 4):
  4 stamp.sec i32 | 8 stamp.nanosec u32 | 12 frame_id len u32 | 16 frame_id+NUL
  then pad to 8-alignment, orientation 4d, orientation_cov 9d,
  angular_velocity 3d, angular_velocity_cov 9d, linear_acceleration 3d, cov 9d.
"""

import struct

_LEN = struct.Struct('<I')
_ANG = struct.Struct('<3d')
_NEG1 = struct.pack('<d', -1.0)
_ENCAP_LE = b'\x00\x01'
_TAIL = (4 + 9 + 3 + 9 + 3 + 9) * 8  # orientation .. linear_acceleration_cov


class ImuLayout:
    __slots__ = ('orient_cov', 'ang', 'ang_cov', 'size')

    def __init__(self, frame_len):
        pos = 16 + frame_len
        orient = 4 + ((pos - 4 + 7) & ~7)
        self.orient_cov = orient + 32
        self.ang = self.orient_cov + 72
        self.ang_cov = self.ang + 24
        self.size = orient + _TAIL


class ImuPatcher:
    """patch(raw, corrected, variance) -> bytes, or None if raw is not a
    plain little-endian Imu the layout maths covers (caller falls back)."""

    def __init__(self, variance):
        v = variance
        self._cov = struct.pack('<9d', v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, v)
        self._layouts = {}

    def layout(self, raw):
        if len(raw) < 20 or raw[:2] != _ENCAP_LE:
            return None
        n = _LEN.unpack_from(raw, 12)[0]
        lay = self._layouts.get(n)
        if lay is None:
            if n > 256:
                return None
            lay = self._layouts[n] = ImuLayout(n)
        return lay if len(raw) >= lay.size else None

    def angular_velocity(self, raw, lay):
        return _ANG.unpack_from(raw, lay.ang)

    def patch(self, raw, lay, x, y, z):
        buf = bytearray(raw)
        _ANG.pack_into(buf, lay.ang, x, y, z)
        buf[lay.ang_cov:lay.ang_cov + 72] = self._cov
        buf[lay.orient_cov:lay.orient_cov + 8] = _NEG1
        return bytes(buf)
