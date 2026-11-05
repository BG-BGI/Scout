"""Continuous landmark correction decision logic (pure, no ROS).

tag_relocalizer feeds each solved tag pose (map x, y, yaw) plus the current
amcl pose and robot speed; this decides whether to re-seed /initialpose.
Median window rejects single-frame noise, a spread test demands agreement,
a speed gate avoids motion/latency error, thresholds avoid chasing noise,
and a cooldown rate-limits corrections.
"""

import math
from collections import deque

from scout.core.geometry import wrap_angle


def median(vals):
    s = sorted(vals)
    n = len(s)
    if n == 0:
        raise ValueError('median of empty sequence')
    mid = n // 2
    return s[mid] if n % 2 else 0.5 * (s[mid - 1] + s[mid])


def circular_median(angles):
    """Median of angles (rad) robust to the +-pi wrap: median of the
    deviations from the circular mean, re-anchored on that mean."""
    ref = math.atan2(sum(math.sin(a) for a in angles),
                     sum(math.cos(a) for a in angles))
    return wrap_angle(ref + median([wrap_angle(a - ref) for a in angles]))


def circular_spread(angles, center):
    """Max absolute deviation (rad) of angles from center."""
    return max(abs(wrap_angle(a - center)) for a in angles)


class LandmarkCorrector:
    """add(...) -> None | (dx, dy, dyaw_rad, dist, (x, y, yaw)).

    dx/dy/dyaw are tag-window median minus current amcl pose; the tuple's
    pose is the median to publish. Returns a correction only when the window
    is full and consistent, the robot is slow, the error exceeds a threshold
    and the cooldown has elapsed. Fast / far samples are not windowed.
    """

    def __init__(self, window=5, spread_xy_m=0.03, spread_yaw_deg=2.0,
                 xy_m=0.10, yaw_deg=3.0, cooldown_s=5.0, max_dist_m=1.5,
                 max_v=0.10, max_w=0.20):
        self.window = int(window)
        self.spread_xy = float(spread_xy_m)
        self.spread_yaw = math.radians(spread_yaw_deg)
        self.xy = float(xy_m)
        self.yaw = math.radians(yaw_deg)
        self.cooldown = float(cooldown_s)
        self.max_dist = float(max_dist_m)
        self.max_v = float(max_v)
        self.max_w = float(max_w)
        self.count = 0
        self._buf = deque(maxlen=self.window)
        self._last_t = None

    def reset(self):
        self._buf.clear()

    def add(self, solve, amcl, v, w, dist, now):
        if dist > self.max_dist:
            return None
        if abs(v) >= self.max_v or abs(w) >= self.max_w:
            self._buf.clear()  # samples straddling motion are suspect
            return None
        self._buf.append(solve)
        if len(self._buf) < self.window:
            return None
        xs = [s[0] for s in self._buf]
        ys = [s[1] for s in self._buf]
        yaws = [s[2] for s in self._buf]
        mx, my = median(xs), median(ys)
        myaw = circular_median(yaws)
        if (max(abs(x - mx) for x in xs) > self.spread_xy
                or max(abs(y - my) for y in ys) > self.spread_xy
                or circular_spread(yaws, myaw) > self.spread_yaw):
            return None
        dx, dy = mx - amcl[0], my - amcl[1]
        dyaw = wrap_angle(myaw - amcl[2])
        if math.hypot(dx, dy) <= self.xy and abs(dyaw) <= self.yaw:
            return None
        if self._last_t is not None and now - self._last_t < self.cooldown:
            return None
        self._last_t = now
        self.count += 1
        self._buf.clear()  # amcl jumps; old solves still valid but re-verify
        return dx, dy, dyaw, dist, (mx, my, myaw)
