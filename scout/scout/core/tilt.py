"""Tilt-from-gravity decision logic (pure, no ROS) — tilt_monitor's brain.

The node feeds raw IMU samples; this owns everything that decides: the
gravity-direction tilt angle against the mounted level axis, the per-sample
LPF, the stillness gate (gyro rates during a pivot corrupt the accel vector,
so spinning samples must not accumulate toward an abort), and the
entry-dwelled abort latch (scout.core.latch). Constants stay in the node's
parameters; this takes them injected, like core.health.
"""

import math

from scout.core.latch import Latch

WARN = 'warn'
ABORT = 'abort'


class TiltTracker:
    """update(accel, gyro, now) -> None | WARN | ABORT.

    WARN fires once per excursion over warn_deg (re-arms below it);
    ABORT fires once, when the filtered tilt has held over abort_deg for
    hold_s seconds of non-spinning samples — after that the tracker is
    latched and returns None forever (the caller owns any un-latching).
    The filtered angle is readable as .tilt_deg (None until the first
    valid sample).
    """

    def __init__(self, level_axis, warn_deg, abort_deg, stillness_gyro,
                 hold_s, lpf_alpha):
        self.level_axis = level_axis
        self.warn_deg = float(warn_deg)
        self.abort_deg = float(abort_deg)
        self.stillness_gyro = float(stillness_gyro)
        self.lpf_alpha = float(lpf_alpha)
        self.tilt_deg = None
        self.latched = False
        self._warned = False
        self._abort = Latch(on_dwell=float(hold_s))

    def update(self, accel, gyro, now):
        if self.latched:
            return None

        spinning = max(abs(gyro[0]), abs(gyro[1]), abs(gyro[2])) \
            >= self.stillness_gyro

        ax, ay, az = accel
        mag = math.sqrt(ax * ax + ay * ay + az * az)
        if mag < 1.0:
            return None  # free-fall / garbage sample: no gravity direction
        ux, uy, uz = ax / mag, ay / mag, az / mag
        lx, ly, lz = self.level_axis
        dot = max(-1.0, min(1.0, ux * lx + uy * ly + uz * lz))
        tilt = math.degrees(math.acos(dot))

        if self.tilt_deg is None:
            self.tilt_deg = tilt
        else:
            self.tilt_deg += self.lpf_alpha * (tilt - self.tilt_deg)

        if spinning:
            # Corrupted gravity vector: reset the abort dwell, decide nothing.
            self._abort.update(False, False, now)
            return None

        event = None
        if self.tilt_deg >= self.warn_deg and not self._warned:
            self._warned = True
            event = WARN
        if self.tilt_deg < self.warn_deg:
            self._warned = False

        if self._abort.update(self.tilt_deg >= self.abort_deg, False, now):
            self.latched = True
            return ABORT
        return event
