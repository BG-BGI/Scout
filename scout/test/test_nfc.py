"""scout.core.nfc — Flipper NFC `scanner` output parsing (1:1 with core/nfc.py).

⚠ FIXTURES ARE BEST-EFFORT until bench-captured (plan step F0): the transcript
shapes below follow the official CLI docs (the `nfc` sub-shell `scanner` verb)
and common firmware output, but the real success line must be recorded from the
attached Flipper (miniterm /dev/ttyACM0 230400, `nfc` then `scanner`, present a
card) and pasted here verbatim. The parser is deliberately tolerant of
separators/prefixes so a format drift shows up as a test edit, not a field
failure. Unlike RFID, MIFARE/NTAG/ISO14443 are VALID techs here.
"""

from scout.core.nfc import NFC_ENTER, NFC_EXIT, NFC_SCAN, parse_scan_output

SCAN_IN_PROGRESS = (
    'scanner\r\n'
    'Scanning for NFC tags...\r\n'
    'Press Ctrl+C to abort\r\n'
)


# --- command constants (the sub-shell sequence flipper_node drives) ----------

def test_subshell_command_constants():
    assert (NFC_ENTER, NFC_SCAN, NFC_EXIT) == ('nfc', 'scanner', 'exit')


# --- parse_scan_output -------------------------------------------------------

def test_no_tag_yet_returns_none():
    assert parse_scan_output(SCAN_IN_PROGRESS) is None
    assert parse_scan_output('') is None


def test_tech_with_inline_uid():
    buf = SCAN_IN_PROGRESS + 'MIFARE Classic 1K 04 A2 2B 5C\r\n'
    assert parse_scan_output(buf) == {'protocol': 'MIFARE Classic 1K',
                                      'data_hex': '04A22B5C'}


def test_uid_line_carries_preceding_tech():
    buf = SCAN_IN_PROGRESS + 'ISO14443-3A\r\nUID: 04 A2 2B 5C 6D 7E 8F\r\n'
    assert parse_scan_output(buf) == {'protocol': 'ISO14443-3A',
                                      'data_hex': '04A22B5C6D7E8F'}


def test_uid_before_any_tech_falls_back_to_nfc():
    assert parse_scan_output('UID: AABBCCDD\r\n') == {'protocol': 'NFC',
                                                      'data_hex': 'AABBCCDD'}


def test_subprotocol_wins_longest_match():
    buf = 'MIFARE Classic 4K 11 22 33 44\r\n'
    assert parse_scan_output(buf)['protocol'] == 'MIFARE Classic 4K'
    assert parse_scan_output('NTAG215 01 02 03 04\r\n')['protocol'] == 'NTAG215'


def test_mifare_is_valid_here_unlike_rfid():
    # core/rfid.py rejects 'MIFARE' (LF-only list); NFC accepts it.
    assert parse_scan_output('MIFARE 1A 2B 3C 4D\r\n') == {
        'protocol': 'MIFARE', 'data_hex': '1A2B3C4D'}


def test_prose_and_bare_tech_not_a_read():
    assert parse_scan_output('Present a MIFARE card to the back\r\n') is None
    assert parse_scan_output('NTAG215\r\n') is None


def test_odd_nibble_count_rejected():
    assert parse_scan_output('MIFARE Classic 1K 04 A2 2\r\n') is None
    assert parse_scan_output('UID: 04 A2 2\r\n') is None


# The /nfc/reads wire format lives in scout.core.status (format_nfc_read); its
# freeze is in test_status.py with the other wire formats.
