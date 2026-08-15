"""Trick choreography tables + validation (extracted from trick_player).

TRICKS map a name to a list of (duration_s, vx, wz[, '#RRGGBB' LED override])
segments; TRICK_LED gives each trick a default (color, led_node mode). The
velocity caps and the pivot floor are passed in by the node (from the robot
profile / a param) so this module stays pure.
"""

# name: [(duration_s, vx m/s, wz rad/s[, '#RRGGBB' led override]), ...]
TRICKS = {
    'spin':      [(4.2, 0.0, 3.0)],
    'wiggle':    [(0.3, 0.0, 2.5), (0.3, 0.0, -2.5)] * 6,
    'figure8':   [(3.5, 0.4, 1.5), (3.5, 0.4, -1.5)],
    'wheelie':   [(0.4, -1.0, 0.0), (0.6, 1.0, 0.0)],
    'fakeout':   [(0.3, 1.0, 0.0), (0.4, 0.0, 0.0), (0.5, 0.0, 0.0),
                  (0.3, 1.0, 0.0), (0.6, -0.6, 0.0)],
    'shiver':    [(0.15, 0.0, 2.5), (0.15, 0.0, -2.5)] * 8,
    'whiplash':  [(2.1, 0.0, 3.0), (2.1, 0.0, -3.0)],
    'orbit':     [(4.2, 0.4, 1.5)],
    'slalom':    [(0.5, 0.6, 1.2), (0.5, 0.6, -1.2)] * 3,
    'moonwalk':  [(0.7, -0.25, 0.8), (0.7, -0.25, -0.8)] * 4,
    'wag':       [(0.2, 0.15, 2.5), (0.2, 0.15, -2.5)] * 6,
    'countdown': [(1.0, 0.0, 0.0, '#FF0000'), (1.0, 0.0, 0.0, '#FF8000'),
                  (1.0, 0.0, 0.0, '#00FF00'), (1.0, 1.0, 0.0, '#00FF00'),
                  (0.4, 0.0, 0.0, '#00FF00')],
    'disco':     [(0.25, 0.0, 2.5), (0.25, 0.0, -2.5)] * 8,
    'burnout':   [(1.2, 1.0, 0.0), (0.5, 0.0, 0.0)],
}

# Default LED (color, led_node mode) per trick; 'rainbow' ignores color.
TRICK_LED = {
    'spin':      ('#0080FF', 'chase'),
    'wiggle':    ('#B040FF', 'chase'),
    'figure8':   ('#00FFFF', 'chase'),
    'wheelie':   ('#FF2000', 'chase'),
    'fakeout':   ('#FF2000', 'chase'),
    'shiver':    ('#4060FF', 'breathe'),
    'whiplash':  ('#FFFFFF', 'blink'),
    'orbit':     ('#00FFFF', 'chase'),
    'slalom':    ('#00FF40', 'chase'),
    'moonwalk':  ('', 'rainbow'),
    'wag':       ('#B040FF', 'chase'),
    'countdown': ('#FF0000', 'solid'),
    'disco':     ('', 'rainbow'),
    'burnout':   ('#FF6000', 'chase'),
}


def validate_tricks(tricks, trick_led, *, max_linear, max_angular, min_pivot_rate):
    """Raise ValueError if any segment violates the caps or the pivot floor, or
    if a trick has no TRICK_LED entry."""
    for name, segments in tricks.items():
        for i, seg in enumerate(segments):
            dur, vx, wz = seg[0], seg[1], seg[2]
            if dur <= 0.0:
                raise ValueError('%s[%d]: duration %.2f <= 0' % (name, i, dur))
            if abs(vx) > max_linear:
                raise ValueError('%s[%d]: |vx| %.2f > %.2f' % (name, i, vx, max_linear))
            if abs(wz) > max_angular:
                raise ValueError('%s[%d]: |wz| %.2f > %.2f' % (name, i, wz, max_angular))
            if vx == 0.0 and wz != 0.0 and abs(wz) < min_pivot_rate:
                raise ValueError(
                    '%s[%d]: pivot at %.2f rad/s is under the %.2f tracking floor'
                    % (name, i, wz, min_pivot_rate))
        if name not in trick_led:
            raise ValueError('%s: missing TRICK_LED entry' % name)
