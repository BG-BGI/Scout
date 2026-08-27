"""Flipper Zero NFC CLI parsing — pure functions, no ROS/serial imports.

Sibling of core/rfid.py for the 13.56 MHz HF radio, but the read MECHANISM is
fundamentally different and this is load-bearing (verified against firmware
source `applications/main/nfc/cli/...`, dev branch, 2026-08-27):

  * The Flipper `nfc` command opens a SUB-SHELL. Inside it, `scanner` lists only
    the PROTOCOL NAMES of a presented tag ("Protocols detected: Mifare Classic,
    Iso14443-3a") — it emits NO UID. There is no CLI verb that prints a tag UID
    to stdout. So, unlike RFID's `rfid read` (which prints protocol + data hex),
    NFC cannot yield a per-tag identifier from a single command's output.

  * To recover a UID we DUMP the card to a .nfc file and read it back:
      (sub-shell)  nfc                       enter
                   dump -f <path>            auto-detects protocol, reads the
                                             card, writes <path> (a Flipper NFC
                                             file); prints "Dump saved to
                                             '<path>'" on success, or an
                                             ANSI-red "Error: <reason>" line
                                             (e.g. "Error: timeout" = no card).
      (top level)  exit                      leave the sub-shell (storage is not
                                             available inside it)
                   storage read <path>       prints "Size: N" then the file
                                             VERBATIM, then a trailing newline
                   storage remove <path>     clean up
    The .nfc file carries the authoritative `Device type: <name>` and
    `UID: XX XX XX XX` (space-separated uppercase hex) lines — parse_nfc_file
    reads exactly those two, no protocol whitelist needed.

flipper_node drives that sequence (dump inside the shell, storage read/remove
at the top level). This module is FIRMWARE-coupled only; the /nfc/reads wire
format lives with the other wire formats in scout.core.status (format_nfc_read).

⚠ NOT yet bench-captured on real firmware — the shapes below are from firmware
SOURCE, which is authoritative for the format strings but does not prove the
end-to-end shell dance on this unit. Recapture on the bench (miniterm
/dev/ttyACM0 230400: `nfc`, `dump -f /ext/nfc/t.nfc`, present a card, `exit`,
`storage read /ext/nfc/t.nfc`) and confirm the fixtures before trusting reads.

⚠ `dump` with no `-p` uses a 5 s internal timeout (firmware default) for BOTH
protocol auto-detect and the card read, so a no-card cycle blocks ~5 s on the
Flipper; the node's poll loop is non-blocking (it accumulates whatever arrived
each tick), so this only sets the effective re-scan cadence.
"""

import re

# Sub-shell command sequence (see module header). `dump` and its file live
# inside the `nfc` sub-shell; `storage read`/`remove` run at the TOP level.
NFC_ENTER = 'nfc'
NFC_EXIT = 'exit'

# Fixed temp dump target. Removed after every read and once at scan start, so
# `dump`'s "File already exists" guard never trips; a no-card cycle writes no
# file, so nothing to clean on those.
NFC_TMP_DIR = '/ext/nfc'
NFC_TMP_PATH = '/ext/nfc/_scout_scan.nfc'


def nfc_dump_cmd(path=NFC_TMP_PATH):
    """The in-sub-shell dump command (auto-detect protocol -> write <path>)."""
    return 'dump -f %s' % path


def nfc_read_file_cmd(path=NFC_TMP_PATH):
    """Top-level: stream the dumped .nfc file back over the CLI."""
    return 'storage read %s' % path


def nfc_remove_cmd(path=NFC_TMP_PATH):
    """Top-level: delete the temp dump file."""
    return 'storage remove %s' % path


def nfc_mkdir_cmd(path=NFC_TMP_DIR):
    """Top-level: ensure the dump directory exists (dump errors if it doesn't).
    Harmless if it already does (prints a storage error we drain)."""
    return 'storage mkdir %s' % path


def parse_dump_output(buf):
    """Incremental: accumulated `dump` output so far -> a terminal signal or
    None while still in progress:
      * 'saved' once "Dump saved to '<path>'" appears (card read + written),
      * 'error' once an "Error: <reason>" line appears (no card / read fail /
        auth) — the ANSI colour codes around it do not break the substring,
      * None otherwise (the "Protocols detected:"/"Dumping as" progress lines).
    """
    if 'Dump saved to ' in buf:
        return 'saved'
    if 'Error:' in buf:
        return 'error'
    return None


def parse_nfc_file(buf):
    """The `storage read` output of a dumped .nfc file -> None until both a
    UID and (ideally) a Device type line are present, then
    {'protocol': str, 'data_hex': 'AABB…'} (UID uppercase, no separators).

    Reads the file's own `Device type: <name>` and `UID: <hex>` key lines
    verbatim (the authoritative protocol string and UID). The `# Device type
    can be …` / `# UID is common …` comment lines start with '#', so the
    prefix match skips them."""
    proto = None
    data = None
    for line in buf.splitlines():
        line = line.strip()
        if line.startswith('Device type:'):
            proto = line[len('Device type:'):].strip() or None
        elif line.startswith('UID:'):
            d = re.sub(r'[^0-9A-Fa-f]', '', line[len('UID:'):]).upper()
            if len(d) >= 4 and len(d) % 2 == 0:
                data = d
    if data:
        return {'protocol': proto or 'NFC', 'data_hex': data}
    return None
