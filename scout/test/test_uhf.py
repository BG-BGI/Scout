"""Tests for scout.core.uhf — ThingMagic Mercury framing for the M7E Hecto.

Command fixtures are exact expected bytes: cmd_version() is pinned against the
canonical ThingMagic version probe FF 00 03 1D 0C (the CRC known-vector), the
rest against this implementation's output frozen at ADR-0032 time. Response
fixtures are synthetic frames built with the same CRC (self-consistent
round-trips pinning the documented byte offsets, parseResponse L698-720 of the
SparkFun library). Extend with raw bench captures from scripts/uhf_bench.py
after hardware bring-up — especially any frame with embedded tag data.
"""

from scout.core import uhf


def _frame(len_op_status_payload):
    """Wrap body (len, opcode, status, payload) with header + CRC."""
    crc = uhf.crc16(len_op_status_payload)
    return (bytes((uhf.HEADER,)) + len_op_status_payload
            + bytes((crc >> 8, crc & 0xFF)))


def _tag_frame(rssi=0xC4, phase=0x002A, embedded_bits=0, epc=None):
    if epc is None:
        epc = bytes.fromhex('E28011606000020000000001')
    rfu = bytes((0x10, 0x00, 0x1B, 0x01, 0xFF, 0x01, 0x01))
    m = (embedded_bits + 7) // 8
    epc_bits = (len(epc) + 4) * 8  # PC (2) + EPC + EPC CRC (2)
    rec = (bytes((rssi, 0x11, 0x0E, 0x16, 0x40, 0x00, 0x00, 0x01, 0x27,
                  phase >> 8, phase & 0xFF, 0x05,
                  embedded_bits >> 8, embedded_bits & 0xFF))
           + b'\xda' * m                      # embedded data
           + bytes((0x0F, epc_bits >> 8, epc_bits & 0xFF, 0x30, 0x00))
           + epc + bytes((0x45, 0xE9)))       # PC + EPC + EPC CRC
    payload = rfu + rec
    return _frame(bytes((len(payload), 0x22, 0x00, 0x00)) + payload)


KEEPALIVE = _frame(bytes((0x00, 0x22, 0x04, 0x00)))
TEMP_THROTTLE = _frame(bytes((0x00, 0x22, 0x05, 0x04)))
HIGH_RETURN_LOSS = _frame(bytes((0x00, 0x22, 0x05, 0x05)))
VERSION_RESPONSE = _frame(bytes((0x02, 0x03, 0x00, 0x00, 0x14, 0x12)))


# --- CRC + command builders ---------------------------------------------------

def test_crc_known_vector_version_probe():
    # FF 00 03 1D 0C is the ThingMagic boot/version probe from the vendor docs.
    assert uhf.cmd_version() == bytes.fromhex('FF00031D0C')


def test_command_builders_exact_bytes():
    assert uhf.cmd_set_region() == bytes.fromhex('FF0197014BBC')
    assert uhf.cmd_set_tag_protocol() == uhf.build_command(0x93, b'\x00\x05')
    assert uhf.cmd_set_antenna_port() == uhf.build_command(0x91, b'\x01\x01')
    assert uhf.cmd_disable_read_filter() == uhf.build_command(
        0x9A, b'\x01\x0c\x00')
    assert uhf.cmd_start_continuous() == bytes.fromhex(
        'FF102F00000122000005072210001B03E801FFDD2B')
    assert uhf.cmd_stop_continuous() == uhf.build_command(0x2F, b'\x00\x00\x02')


def test_read_power_clamped_in_builder():
    # ADR-0032 USB budget: no caller can exceed 2000 cdBm (20.00 dBm).
    assert uhf.cmd_set_read_power(2700) == uhf.cmd_set_read_power(2000)
    assert uhf.cmd_set_read_power(500) == uhf.build_command(
        0x92, bytes((0x01, 0xF4)))
    assert uhf.cmd_set_read_power(-5) == uhf.build_command(
        0x92, bytes((0x00, 0x00)))


# --- framer -------------------------------------------------------------------

def test_framer_reassembles_split_frames():
    frame = _tag_frame()
    acc = uhf.FrameAccumulator()
    out = []
    for i in range(len(frame)):  # worst case: one byte per serial read
        out += acc.feed(frame[i:i + 1])
    assert out == [frame]


def test_framer_skips_leading_garbage_and_streams_multiple():
    acc = uhf.FrameAccumulator()
    out = acc.feed(b'\x00\x12garbage' + KEEPALIVE + _tag_frame() + KEEPALIVE)
    assert [uhf.classify_frame(f) for f in out] == [
        'keepalive', 'tag', 'keepalive']


def test_framer_resyncs_after_corrupt_frame():
    good = _tag_frame()
    corrupt = bytearray(good)
    corrupt[12] ^= 0xFF  # flip RSSI byte -> CRC fails
    acc = uhf.FrameAccumulator()
    out = acc.feed(bytes(corrupt) + good)
    assert out == [good]  # corrupted record dropped, stream recovers


# --- classification -----------------------------------------------------------

def test_classify_keepalive_family_and_responses():
    assert uhf.classify_frame(KEEPALIVE) == 'keepalive'
    assert uhf.classify_frame(TEMP_THROTTLE) == 'temp_throttle'
    assert uhf.classify_frame(HIGH_RETURN_LOSS) == 'high_return_loss'
    assert uhf.classify_frame(VERSION_RESPONSE) == 'response'
    assert uhf.classify_frame(_tag_frame()) == 'tag'


# --- tag record parsing ---------------------------------------------------------

def test_parse_tag_record_documented_offsets():
    rec = uhf.parse_tag_record(_tag_frame())
    assert rec == {
        'epc': 'E28011606000020000000001',
        'rssi_dbm': -60,          # 0xC4 - 256
        'antenna': 0x11,
        'freq_khz': 923200,       # 0x0E1640
        'timestamp_ms': 295,      # 0x00000127
        'phase': 42,              # raw wire units, stored untouched (stage 2)
        'protocol': uhf.TAG_PROTOCOL_GEN2,
    }


def test_parse_tag_record_embedded_data_shifts_epc():
    rec = uhf.parse_tag_record(_tag_frame(embedded_bits=16,
                                          epc=bytes.fromhex('AABBCCDD')))
    assert rec['epc'] == 'AABBCCDD'
    assert rec['rssi_dbm'] == -60


def test_parse_tag_record_epc_length_variation():
    rec = uhf.parse_tag_record(_tag_frame(epc=bytes.fromhex('0102030405060708')))
    assert rec['epc'] == '0102030405060708'


def test_rssi_two_complement_low_values():
    rec = uhf.parse_tag_record(_tag_frame(rssi=0xEA))
    assert rec['rssi_dbm'] == -22
