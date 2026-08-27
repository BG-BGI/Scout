"""scout.core.nfc — Flipper NFC dump->file read parsing (1:1 with core/nfc.py).

The output shapes below are from firmware SOURCE (applications/main/nfc/cli/…,
dev branch): `dump` prints "Dumping as \"<name>\"" / "Dump saved to '<path>'"
or an ANSI-red "Error: <reason>"; `storage read` prints "Size: N" then the
dumped .nfc file verbatim, whose "Device type:" and "UID:" key lines carry the
authoritative protocol string and UID. NOT yet bench-confirmed on this unit —
recapture (miniterm /dev/ttyACM0 230400) and reconcile before trusting reads.
Unlike RFID, MIFARE/NTAG/ISO14443 are VALID Device-type values here.
"""

from scout.core.nfc import (
    NFC_ENTER,
    NFC_EXIT,
    NFC_TMP_PATH,
    nfc_dump_cmd,
    nfc_mkdir_cmd,
    nfc_read_file_cmd,
    nfc_remove_cmd,
    parse_dump_output,
    parse_nfc_file,
)

ANSI_RED = '\x1b[31m'
ANSI_RESET = '\x1b[0m'

# A real Flipper NFC file streamed by `storage read` (Size header + verbatim
# file + trailing newline). Comment lines start with '# '.
STORAGE_READ_MFC = (
    'Size: 1163\r\n'
    'Filetype: Flipper NFC device\r\n'
    'Version: 4\r\n'
    '# Device type can be ISO14443-3A, ISO14443-3B, ISO14443-4A, ISO14443-4B, '
    'ISO15693-3, FeliCa, NTAG/Ultralight, Mifare Classic, Mifare Plus, '
    'Mifare DESFire, SLIX, ST25TB\r\n'
    'Device type: Mifare Classic\r\n'
    '# UID is common for all formats\r\n'
    'UID: 04 A2 2B 5C\r\n'
    'ATQA: 00 04\r\n'
    'SAK: 08\r\n'
    '\r\n'
)


# --- command constants / builders (the sequence flipper_node drives) ---------

def test_subshell_command_constants():
    assert (NFC_ENTER, NFC_EXIT) == ('nfc', 'exit')
    assert nfc_dump_cmd() == 'dump -f /ext/nfc/_scout_scan.nfc'
    assert nfc_read_file_cmd() == 'storage read /ext/nfc/_scout_scan.nfc'
    assert nfc_remove_cmd() == 'storage remove /ext/nfc/_scout_scan.nfc'
    assert nfc_mkdir_cmd() == 'storage mkdir /ext/nfc'
    assert nfc_dump_cmd('/ext/nfc/x.nfc') == 'dump -f /ext/nfc/x.nfc'
    assert NFC_TMP_PATH == '/ext/nfc/_scout_scan.nfc'


# --- parse_dump_output -------------------------------------------------------

def test_dump_in_progress_returns_none():
    assert parse_dump_output('') is None
    assert parse_dump_output('Press Ctrl+C to abort\r\n\n') is None
    assert parse_dump_output(
        'Protocols detected: Mifare Classic\r\n'
        'Dumping as "Mifare Classic"\r\n') is None


def test_dump_saved_is_terminal_success():
    buf = ('Protocols detected: Mifare Classic\r\n'
           'Dumping as "Mifare Classic"\r\n'
           "Dump saved to '/ext/nfc/_scout_scan.nfc'\r\n")
    assert parse_dump_output(buf) == 'saved'


def test_dump_error_is_terminal_even_with_ansi():
    # No card within the timeout: ANSI-red "Error: timeout".
    buf = 'Press Ctrl+C to abort\r\n\n' + ANSI_RED + 'Error: timeout\r\n' + ANSI_RESET
    assert parse_dump_output(buf) == 'error'
    assert parse_dump_output(ANSI_RED + 'Error: failed to read\r\n' + ANSI_RESET) == 'error'


def test_dump_multi_protocol_still_saves():
    buf = ('Protocols detected: Iso14443-3a, Mifare Classic\r\n'
           'Dumping as "Iso14443-3a"\r\n'
           "Use '-p' key to specify another protocol\r\n"
           "Dump saved to '/ext/nfc/_scout_scan.nfc'\r\n")
    assert parse_dump_output(buf) == 'saved'


# --- parse_nfc_file ----------------------------------------------------------

def test_file_read_yields_protocol_and_uid():
    assert parse_nfc_file(STORAGE_READ_MFC) == {
        'protocol': 'Mifare Classic', 'data_hex': '04A22B5C'}


def test_file_read_7byte_ntag_uid():
    buf = ('Size: 900\r\n'
           'Filetype: Flipper NFC device\r\n'
           'Version: 4\r\n'
           'Device type: NTAG215\r\n'
           'UID: 04 A2 2B 5C 6D 7E 8F\r\n'
           '\r\n')
    assert parse_nfc_file(buf) == {
        'protocol': 'NTAG215', 'data_hex': '04A22B5C6D7E8F'}


def test_file_read_comment_lines_are_not_the_keys():
    # The '# Device type can be …' / '# UID is common …' comments must not be
    # mistaken for the real key lines.
    buf = ('# Device type can be Mifare Classic, NTAG/Ultralight\r\n'
           '# UID is common for all formats\r\n'
           'Device type: FeliCa\r\n'
           'UID: 01 02 03 04 05 06 07 08\r\n')
    assert parse_nfc_file(buf) == {
        'protocol': 'FeliCa', 'data_hex': '0102030405060708'}


def test_file_read_incomplete_returns_none():
    # Header only, UID not streamed yet.
    assert parse_nfc_file('Size: 1163\r\n'
                          'Filetype: Flipper NFC device\r\n'
                          'Device type: Mifare Classic\r\n') is None
    assert parse_nfc_file('') is None


def test_file_read_uid_without_device_type_falls_back_to_nfc():
    assert parse_nfc_file('UID: AA BB CC DD\r\n') == {
        'protocol': 'NFC', 'data_hex': 'AABBCCDD'}


def test_file_read_odd_nibble_uid_rejected():
    assert parse_nfc_file('Device type: Mifare Classic\r\nUID: 04 A2 2\r\n') is None


# The /nfc/reads wire format lives in scout.core.status (format_nfc_read); its
# freeze is in test_status.py with the other wire formats.
