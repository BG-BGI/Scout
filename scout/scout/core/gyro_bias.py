"""Gyro bias estimator (pure, no ROS) — gyro_calibrator's per-sample math.

Startup window of `calibration_s` seconds sets the initial bias; afterwards the
bias is blended toward each uninterrupted stationary window of `refresh_s`
seconds by `weight`. update() is the hot path (100-200 Hz): scalars in, scalars
out, no allocation beyond the returned tuple. Time is integer nanoseconds.
"""

import math

# Only announce a refreshed bias when it moved by ~0.03 deg/s.
LOG_DELTA = 0.0005


class GyroBias:
    def __init__(self, calibration_s, stillness, refresh_s, weight, log=None):
        self.calibration_s = calibration_s
        self.stillness = stillness
        self.refresh_s = refresh_s
        self.weight = weight
        self._log = log or (lambda level, msg: None)
        self.bias = None      # (x, y, z) once the startup window closes
        self.refreshes = 0
        self._sx = self._sy = self._sz = 0.0
        self._count = 0
        self._t0 = None

    def _reset(self):
        self._sx = self._sy = self._sz = 0.0
        self._count = 0
        self._t0 = None

    def update(self, x, y, z, t_ns):
        """Return corrected (x, y, z), or None while the startup window is open."""
        bias = self.bias
        if bias is None:
            if self._t0 is None:
                self._t0 = t_ns
            self._sx += x
            self._sy += y
            self._sz += z
            self._count += 1
            if (t_ns - self._t0) * 1e-9 >= self.calibration_s:
                n = self._count
                self.bias = b = (self._sx / n, self._sy / n, self._sz / n)
                self._log('info',
                          'Gyro bias rad/s: x=%+.5f y=%+.5f z=%+.5f (%d samples) — '
                          'yaw axis will be refreshed whenever the robot is still'
                          % (*b, n))
                if max(abs(v) for v in b) > self.stillness:
                    self._log('warn',
                              'Bias exceeds the stillness threshold (%.3f rad/s) — '
                              'was the robot actually stationary during calibration?'
                              % self.stillness)
                self._reset()
            return None

        cx = x - bias[0]
        cy = y - bias[1]
        cz = z - bias[2]
        s = self.stillness
        if abs(cx) < s and abs(cy) < s and abs(cz) < s:
            if self._t0 is None:
                self._t0 = t_ns
            self._sx += x
            self._sy += y
            self._sz += z
            self._count += 1
            if (t_ns - self._t0) * 1e-9 >= self.refresh_s:
                self._refresh()
        elif self._count:
            self._reset()
        return (cx, cy, cz)

    def _refresh(self):
        n = self._count
        w = self.weight
        old = self.bias
        new = tuple(b + w * (m / n - b)
                    for b, m in zip(old, (self._sx, self._sy, self._sz), strict=True))
        self.bias = new
        self.refreshes += 1
        drift = max(abs(a - b) for a, b in zip(new, old, strict=True))
        message = ('Gyro bias refresh %d: x=%+.5f y=%+.5f z=%+.5f rad/s '
                   '(moved %.4f deg/s)' % (self.refreshes, *new, math.degrees(drift)))
        self._log('info' if drift > LOG_DELTA else 'debug', message)
        self._reset()
