"""One latch with asymmetric entry/exit conditions and dwells (pure, no ROS).

Four nodes hand-rolled this same state machine and each got a different
subset right: collision_polygon_manager (turn + reverse zone hysteresis with
exit dwell), led_status (battery warn/critical voltage hysteresis),
tilt_monitor (abort threshold with entry dwell + spin-gate reset). This is
the one implementation; the caller expresses its thresholds as the two
boolean conditions, which is what lets value hysteresis (enter at or below
the critical volts, leave only hysteresis_volts above), rate hysteresis
(enter above 0.8 rad/s, leave below 0.4) and dwell gating all be the same
object.

Semantics, per update(enter, leave, now):
  * state False -> True once `enter` has held continuously for on_dwell
    seconds (0.0 = immediately). Any update with `enter` false resets the
    accumulation — a spin-gated tilt sample resets the abort dwell exactly by
    passing enter=False.
  * state True -> False once `leave` has held continuously for off_dwell
    seconds. Any update with `leave` false resets that accumulation.
  * `now` is any monotonic seconds source (injected — tests use plain floats).
"""


class Latch:
    """See module docstring. State is readable as `.state` (bool)."""

    def __init__(self, on_dwell=0.0, off_dwell=0.0, state=False):
        self.on_dwell = float(on_dwell)
        self.off_dwell = float(off_dwell)
        self.state = bool(state)
        self._on_since = None
        self._off_since = None

    def update(self, enter, leave, now=0.0):
        """Feed one sample; returns the (possibly new) state."""
        if not self.state:
            self._off_since = None
            if not enter:
                self._on_since = None
            else:
                if self._on_since is None:
                    self._on_since = now
                if now - self._on_since >= self.on_dwell:
                    self.state = True
                    self._on_since = None
        else:
            self._on_since = None
            if not leave:
                self._off_since = None
            else:
                if self._off_since is None:
                    self._off_since = now
                if now - self._off_since >= self.off_dwell:
                    self.state = False
                    self._off_since = None
        return self.state
