"""Hex color parsing for the LED strip (extracted from led_node)."""


def parse_hex_color(text):
    """Parse a hex color into (r, g, b). Empty -> black. Raises ValueError.

    Accepts '#RRGGBB', 'RRGGBB', '0xRRGGBB', and 3-digit shorthand '#RGB'.
    """
    s = (text or '').strip().lower()
    if not s:
        return (0, 0, 0)
    if s.startswith('0x'):
        s = s[2:]
    elif s.startswith('#'):
        s = s[1:]
    if len(s) == 3:                     # shorthand: "abc" -> "aabbcc"
        s = ''.join(c * 2 for c in s)
    if len(s) != 6 or any(c not in '0123456789abcdef' for c in s):
        raise ValueError("expected hex like '#RRGGBB', got '%s'" % text)
    return (int(s[0:2], 16), int(s[2:4], 16), int(s[4:6], 16))
