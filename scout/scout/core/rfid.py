"""Flipper Zero CLI output parsing — pure functions, no ROS/serial imports.

The Flipper's USB CDC shell is a human-facing terminal: it echoes what was
typed, prompts with `>:`, and `rfid read` prints free-text progress until a
card is found. These helpers turn that stream into events for flipper_node.

This module is FIRMWARE-coupled only; the /rfid/reads wire format lives with
the other wire formats in scout.core.status (format_rfid_read) — different
change triggers (a firmware update vs a protocol change), different modules.

⚠ The success-line format is FIRMWARE-DEPENDENT. parse_read_output matches
"<known protocol> <hex bytes>" tolerantly (optional separators, optional
data:/Hex: prefixes) against the protocol names the official CLI documents;
test_rfid.py pins the accepted shapes. After a firmware update, recapture real
transcripts on the bench (miniterm /dev/ttyACM0 230400) and extend the
fixtures before trusting the parser — do not guess.
"""

import re

PROMPT = '>:'

# Protocol names from the official CLI docs (rfid write/emulate key types).
# Longest-first so "EM4100/32" wins over "EM4100".
PROTOCOLS = sorted((
    'EM4100/32', 'EM4100/16', 'EM4100', 'Electra', 'H10301', 'Idteck',
    'Indala26', 'IoProxXSF', 'AWID', 'FDX-A', 'FDX-B', 'HIDProx', 'HIDExt',
    'Pyramid', 'Viking', 'Jablotron', 'Paradox', 'PAC/Stanley', 'Keri',
    'Gallagher', 'Nexwatch', 'Noralsy', 'Securakey', 'GProxII', 'Radio Key',
), key=len, reverse=True)

_HEX_RUN = re.compile(r'^(?:(?:data|hex)\s*[:=]\s*)?((?:[0-9A-Fa-f]{2}[ :]?)+)\s*$',
                      re.IGNORECASE)


def strip_echo(buf, command):
    """Remove the shell's echo of `command` (first occurrence) from `buf`."""
    idx = buf.find(command)
    if idx < 0:
        return buf
    return buf[:idx] + buf[idx + len(command):]


def has_prompt(buf):
    """True when the shell is idle again (a `>:` prompt is present)."""
    return PROMPT in buf


def parse_read_output(buf):
    """Incremental: the accumulated `rfid read` output so far -> None until a
    complete card line appears, then {'protocol': str, 'data_hex': 'AABB…'}
    (uppercase, no separators)."""
    for line in buf.splitlines():
        line = line.strip()
        for proto in PROTOCOLS:
            if not line.startswith(proto):
                continue
            rest = line[len(proto):].strip()
            m = _HEX_RUN.match(rest)
            if m:
                data = re.sub(r'[^0-9A-Fa-f]', '', m.group(1)).upper()
                if len(data) >= 4 and len(data) % 2 == 0:
                    return {'protocol': proto, 'data_hex': data}
    return None
