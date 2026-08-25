"""scout.core.rfid — Flipper CLI output parsing (1:1 with core/rfid.py).

⚠ FIXTURES ARE BEST-EFFORT until bench-captured (plan step F0): the transcript
shapes below follow the official CLI docs and common firmware output, but the
real success line must be recorded from the attached Flipper
(miniterm /dev/ttyACM0 230400, `rfid read`, present a card) and pasted here
verbatim. The parser is deliberately tolerant of separators/prefixes so a
format drift shows up as a test edit, not a field failure.
"""

from scout.core.rfid import (
    has_prompt,
    parse_read_output,
    strip_echo,
)

BANNER = (
    'Flipper Zero Command Line Interface!\r\n'
    'Read the manual: https://docs.flipper.net/development/cli\r\n'
    '\r\n>: '
)

READ_IN_PROGRESS = (
    'rfid read\r\n'
    'Reading RFID...\r\n'
    'Press Ctrl+C to abort\r\n'
)


# --- prompt / echo -----------------------------------------------------------

def test_banner_has_prompt():
    assert has_prompt(BANNER)


def test_in_progress_read_has_no_prompt():
    assert not has_prompt(READ_IN_PROGRESS)


def test_strip_echo_removes_command_once():
    out = strip_echo(READ_IN_PROGRESS, 'rfid read')
    assert 'rfid read' not in out
    assert 'Reading RFID' in out


def test_strip_echo_no_match_passthrough():
    assert strip_echo('abc', 'rfid read') == 'abc'


# --- parse_read_output -------------------------------------------------------

def test_no_card_yet_returns_none():
    assert parse_read_output(READ_IN_PROGRESS) is None
    assert parse_read_output('') is None


def test_em4100_spaced_hex():
    buf = READ_IN_PROGRESS + 'EM4100 1A 2B 3C 4D 5E\r\n'
    assert parse_read_output(buf) == {'protocol': 'EM4100',
                                      'data_hex': '1A2B3C4D5E'}


def test_em4100_packed_hex():
    buf = READ_IN_PROGRESS + 'EM4100 1a2b3c4d5e\r\n'
    assert parse_read_output(buf) == {'protocol': 'EM4100',
                                      'data_hex': '1A2B3C4D5E'}


def test_data_prefix_variant():
    buf = READ_IN_PROGRESS + 'H10301 data: 01 23 45\r\n'
    assert parse_read_output(buf) == {'protocol': 'H10301',
                                      'data_hex': '012345'}


def test_subprotocol_wins_longest_match():
    buf = 'EM4100/32 AA BB CC DD EE\r\n'
    assert parse_read_output(buf)['protocol'] == 'EM4100/32'


def test_unknown_protocol_ignored():
    assert parse_read_output('MIFARE 1A 2B 3C 4D\r\n') is None


def test_prose_mentioning_protocol_not_a_read():
    # A hint line naming a protocol without a hex payload must not parse.
    assert parse_read_output('Try EM4100 cards near the back\r\n') is None
    assert parse_read_output('EM4100\r\n') is None


def test_odd_nibble_count_rejected():
    assert parse_read_output('EM4100 1A 2B 3\r\n') is None


# The /rfid/reads wire format moved to scout.core.status (format_rfid_read);
# its freeze lives in test_status.py with the other wire formats.
