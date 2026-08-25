"""Flipper Zero NFC CLI output parsing — pure functions, no ROS/serial imports.

Sibling of core/rfid.py for the 13.56 MHz HF radio. The one structural
difference from RFID: the Flipper's `nfc` command opens a SUB-SHELL — you type
`nfc`, then `scanner` inside it (which lists a presented tag's protocols and
UID), and `exit` to return to the top-level `>:` prompt. There is no one-line
`nfc scanner`. flipper_node drives that sequence; these helpers turn the
`scanner` output stream into a read event.

Like core/rfid.py this module is FIRMWARE-coupled only; the /nfc/reads wire
format lives with the other wire formats in scout.core.status (format_nfc_read).

⚠ The scanner-output format is FIRMWARE-DEPENDENT. parse_scan_output matches
either "<known tech> <hex>" or a "UID: <hex>" line (tolerant of separators and
prefixes) against the HF tech names the official CLI documents; test_nfc.py
pins the accepted shapes. After a firmware update, recapture real transcripts
on the bench (miniterm /dev/ttyACM0 230400, `nfc` then `scanner`, present a
card) and extend the fixtures before trusting the parser — do not guess.

⚠ The sub-shell prompt token is unverified. NFC_PROMPT defaults to the same
`>:` as the top level; if a bench capture shows `nfc>` instead, fix it here and
in flipper_node's shell-exit path. The exit path drains to the TOP-LEVEL `>:`
after `exit`, so it does not depend on NFC_PROMPT being right.
"""

import re

# Sub-shell command sequence (see module header).
NFC_ENTER = 'nfc'
NFC_SCAN = 'scanner'
NFC_EXIT = 'exit'

# Bench-verify: the nfc sub-shell prompt. Only used if a future caller needs to
# gate on the sub-shell prompt specifically; the node's exit path uses the
# top-level '>:' from core.rfid instead.
NFC_PROMPT = '>:'

# HF tech names the Flipper NFC app reports. Longest-first so "MIFARE Classic
# 1K" wins over "MIFARE Classic", "NTAG215" over "NTAG", "ISO14443-4A" over
# "ISO14443". Unlike RFID's LF list, MIFARE/NTAG/ISO14443 are VALID here.
NFC_TECHS = sorted((
    'MIFARE Classic Mini', 'MIFARE Classic 4K', 'MIFARE Classic 1K',
    'MIFARE Classic', 'MIFARE Ultralight C', 'MIFARE Ultralight',
    'MIFARE Plus', 'MIFARE DESFire', 'MIFARE',
    'NTAG213', 'NTAG215', 'NTAG216', 'NTAG',
    'FeliCa', 'ST25TB', 'SLIX2', 'SLIX-L', 'SLIX-S', 'SLIX',  # profile-exempt: protocol names
    'ISO14443-4A', 'ISO14443-4B', 'ISO14443-3A', 'ISO14443-3B',
    'ISO14443A', 'ISO14443B', 'ISO15693-3', 'ISO15693',
), key=len, reverse=True)

_HEX_RUN = re.compile(r'^(?:(?:uid|data|hex)\s*[:=]\s*)?((?:[0-9A-Fa-f]{2}[ :]?)+)\s*$',
                      re.IGNORECASE)
_UID_LINE = re.compile(r'^UID\s*[:=]?\s*((?:[0-9A-Fa-f]{2}[ :]?)+)\s*$',
                       re.IGNORECASE)


def _clean_hex(raw):
    """Strip separators, uppercase; None unless >=2 whole bytes."""
    data = re.sub(r'[^0-9A-Fa-f]', '', raw).upper()
    if len(data) >= 4 and len(data) % 2 == 0:
        return data
    return None


def parse_scan_output(buf):
    """Incremental: the accumulated `scanner` output so far -> None until a
    complete tag line appears, then {'protocol': str, 'data_hex': 'AABB…'}
    (UID uppercase, no separators). Matches either "<tech> <hex>" on one line,
    or a "UID: <hex>" line — carrying the most recently seen tech name as the
    protocol (falling back to 'NFC' when the UID appears before any tech)."""
    proto = None
    for line in buf.splitlines():
        line = line.strip()
        matched_tech = False
        for tech in NFC_TECHS:
            if not line.startswith(tech):
                continue
            matched_tech = True
            proto = tech
            rest = line[len(tech):].strip()
            m = _HEX_RUN.match(rest)
            if m:
                data = _clean_hex(m.group(1))
                if data:
                    return {'protocol': proto, 'data_hex': data}
            break
        if matched_tech:
            continue
        m = _UID_LINE.match(line)
        if m:
            data = _clean_hex(m.group(1))
            if data:
                return {'protocol': proto or 'NFC', 'data_hex': data}
    return None
