"""ThingMagic Mercury serial protocol for the M7E Hecto UHF reader — pure
functions, no ROS/serial imports (ADR-0032).

The SparkFun M7E Hecto board is a JADAK/ThingMagic module behind a CH340C
USB-UART at 115200. Frames are binary, CRC-tailed, and ASYMMETRIC:

  command  = FF len opcode payload…             crc16   (len + 5 bytes total)
  response = FF len opcode status(2) payload…   crc16   (len + 7 bytes total)

`len` counts only the payload (command) / post-status payload (response).
The CRC is the ThingMagic nibble-table variant ("notably not CCITT",
serial_reader_l3.c) over everything between the FF header and the CRC.

Ported from SparkFun_Simultaneous_RFID_Tag_Reader_Library
SparkFun_UHF_RFID_Reader.cpp — command payloads verbatim, tag-record byte
offsets from the commented capture at parseResponse() L698-720. The
continuous-read start blob is a URA transport-log capture upstream never
fully deciphered; treat it as opaque.

The /uhf/reads wire format lives with the other wire formats in
scout.core.status (format_uhf_read_batch) — different change triggers (a
module firmware update vs a protocol change), different modules.

⚠ Tag-record offsets are MODULE-FIRMWARE-coupled. test_uhf.py pins them with
the library's documented capture plus bench captures from scripts/uhf_bench.py
— after a firmware update, recapture on the bench before trusting the parser.
"""

# --- opcodes (TMR_SR_OPCODE_*) ------------------------------------------------
OPCODE_VERSION = 0x03
OPCODE_SET_BAUD_RATE = 0x06
OPCODE_READ_TAG_ID_MULTIPLE = 0x22
OPCODE_MULTI_PROTOCOL_TAG_OP = 0x2F
OPCODE_SET_ANTENNA_PORT = 0x91
OPCODE_SET_READ_TX_POWER = 0x92
OPCODE_SET_TAG_PROTOCOL = 0x93
OPCODE_SET_REGION = 0x97
OPCODE_SET_READER_OPTIONAL_PARAMS = 0x9A

REGION_NORTHAMERICA = 0x01
TAG_PROTOCOL_GEN2 = 0x05

# ADR-0032: the reader runs off a Pi 5 USB port shared with camera + lidar;
# full 27 dBm draws >720 mA @ 5 V. Cap at 20.00 dBm until it gets its own 5 V.
READ_POWER_MAX_CDBM = 2000

# Continuous-read keepalive/fault status words (response [3:5], opcode 0x22).
STATUS_KEEPALIVE = 0x0400
STATUS_TEMP_THROTTLE = 0x0504
STATUS_HIGH_RETURN_LOSS = 0x0505

HEADER = 0xFF

# ThingMagic-mutated CRC table (serial_reader_l3.c via the SparkFun library).
_CRC_TABLE = (
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
)

# URA transport-log capture: continuous GEN2 inventory via opcode 0x2F
# (timeout 0, TM option 1, sub-opcode 0x22, search flags, GEN2 …).
_START_CONTINUOUS_BLOB = bytes((
    0x00, 0x00, 0x01, 0x22, 0x00, 0x00, 0x05, 0x07,
    0x22, 0x10, 0x00, 0x1B, 0x03, 0xE8, 0x01, 0xFF,
))
_STOP_CONTINUOUS_BLOB = bytes((0x00, 0x00, 0x02))


def crc16(data):
    """ThingMagic CRC over `data` (bytes), init 0xFFFF, nibble-table."""
    crc = 0xFFFF
    for b in data:
        crc = (((crc << 4) & 0xFFFF) | (b >> 4)) ^ _CRC_TABLE[crc >> 12]
        crc = (((crc << 4) & 0xFFFF) | (b & 0x0F)) ^ _CRC_TABLE[crc >> 12]
    return crc


def build_command(opcode, payload=b''):
    """Complete command frame: FF len opcode payload crc16."""
    body = bytes((len(payload), opcode)) + bytes(payload)
    crc = crc16(body)
    return bytes((HEADER,)) + body + bytes((crc >> 8, crc & 0xFF))


def cmd_version():
    return build_command(OPCODE_VERSION)


def cmd_set_region(region=REGION_NORTHAMERICA):
    return build_command(OPCODE_SET_REGION, bytes((region,)))


def cmd_set_tag_protocol(protocol=TAG_PROTOCOL_GEN2):
    # Opcode expects 16 bits; high byte is always 0.
    return build_command(OPCODE_SET_TAG_PROTOCOL, bytes((0x00, protocol)))


def cmd_set_antenna_port():
    # TX port 1, RX port 1 — the board's single U.FL antenna path.
    return build_command(OPCODE_SET_ANTENNA_PORT, bytes((0x01, 0x01)))


def cmd_set_read_power(centi_dbm):
    """Read TX power in hundredths of a dBm, clamped to READ_POWER_MAX_CDBM
    here in pure code so no caller can exceed the ADR-0032 USB power budget."""
    centi_dbm = max(0, min(int(centi_dbm), READ_POWER_MAX_CDBM))
    return build_command(OPCODE_SET_READ_TX_POWER,
                         bytes((centi_dbm >> 8, centi_dbm & 0xFF)))


def cmd_disable_read_filter():
    # setReaderConfiguration(0x0C, 0x00): key-value form, "read filter" off —
    # required before continuous read so repeat sightings keep streaming.
    return build_command(OPCODE_SET_READER_OPTIONAL_PARAMS,
                         bytes((0x01, 0x0C, 0x00)))


def cmd_start_continuous():
    return build_command(OPCODE_MULTI_PROTOCOL_TAG_OP, _START_CONTINUOUS_BLOB)


def cmd_stop_continuous():
    return build_command(OPCODE_MULTI_PROTOCOL_TAG_OP, _STOP_CONTINUOUS_BLOB)


class FrameAccumulator:
    """Incremental response framer: feed() raw serial bytes, get back complete
    CRC-checked response frames. Frames routinely split across serial reads at
    150 tags/s; a bad CRC drops one byte and rescans to the next 0xFF, so a
    corrupted stream loses one record, not the session."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data):
        """-> list of complete frames (bytes, FF..crc inclusive), in order."""
        self._buf.extend(data)
        frames = []
        while True:
            # Discard garbage before the next possible header.
            start = self._buf.find(HEADER)
            if start < 0:
                self._buf.clear()
                return frames
            if start:
                del self._buf[:start]
            if len(self._buf) < 2:
                return frames
            total = self._buf[1] + 7  # FF len op status(2) payload crc(2)
            if len(self._buf) < total:
                return frames
            frame = bytes(self._buf[:total])
            crc = crc16(frame[1:total - 2])
            if (frame[-2], frame[-1]) == (crc >> 8, crc & 0xFF):
                frames.append(frame)
                del self._buf[:total]
            else:
                del self._buf[0]  # false header — resync one byte on


def frame_status(frame):
    """16-bit status word from response bytes [3:5]."""
    return (frame[3] << 8) | frame[4]


def classify_frame(frame):
    """One of 'tag', 'keepalive', 'temp_throttle', 'high_return_loss',
    'temperature', 'response' (a plain command reply), 'unknown'.
    Mirrors the library's parseResponse() dispatch."""
    if frame[2] != OPCODE_READ_TAG_ID_MULTIPLE:
        return 'response'
    if frame[1] == 0x00:  # keepalive family — status word says which
        return {
            STATUS_KEEPALIVE: 'keepalive',
            STATUS_TEMP_THROTTLE: 'temp_throttle',
            STATUS_HIGH_RETURN_LOSS: 'high_return_loss',
        }.get(frame_status(frame), 'unknown')
    if frame[1] == 0x08:
        return 'unknown'
    if frame[1] == 0x0A:
        return 'temperature'
    return 'tag'


def parse_tag_record(frame):
    """Continuous-read tag record -> dict. Offsets from the library's
    documented capture (parseResponse L698-720); embedded-data length at
    [24:26] shifts the EPC fields when non-zero.

    Returns {'epc', 'rssi_dbm', 'antenna', 'freq_khz', 'timestamp_ms',
    'phase', 'protocol'}. `phase` is the RAW wire value (0-180-ish units,
    calibration unresolved) — stored untouched as stage-2 synthetic-aperture
    fuel, do not normalize (ADR-0032)."""
    rssi = frame[12] - 256 if frame[12] >= 128 else frame[12]
    data_bits = (frame[24] << 8) | frame[25]  # profile-exempt: wire offsets
    m = (data_bits + 7) // 8
    epc_bits = (frame[27 + m] << 8) | frame[28 + m]
    epc_bytes = epc_bits // 8 - 4  # strip PC (2) and EPC CRC (2)
    epc = frame[31 + m:31 + m + epc_bytes]
    return {
        'epc': epc.hex().upper(),
        'rssi_dbm': rssi,
        'antenna': frame[13],
        'freq_khz': (frame[14] << 16) | (frame[15] << 8) | frame[16],
        'timestamp_ms': ((frame[17] << 24) | (frame[18] << 16)  # profile-exempt: wire offsets
                         | (frame[19] << 8) | frame[20]),
        'phase': (frame[21] << 8) | frame[22],
        'protocol': frame[23],
    }
