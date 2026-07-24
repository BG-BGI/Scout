#!/usr/bin/env python3
"""Standalone RoboClaw 2x30A bench test — no ROS, no pyserial.

Reads the firmware version and battery voltage to prove the packet-serial link,
then gently spins each motor forward and back. Runs via the docker-compose
`test` service (privileged + host net expose the UART):

    docker compose --profile test run --rm test                        # comms + motors
    docker compose --profile test run --rm -e ROBOCLAW_COMMS_ONLY=1 test   # comms only

Wiring: RoboClaw S1 <- Pi TXD (GPIO14, pin 8), S2 -> Pi RXD (GPIO15, pin 10),
common ground. The Pi 5 header UART is /dev/ttyAMA0 and needs `dtparam=uart0=on`
in config.txt (ttyAMA10 is the separate debug/console UART — do not use it).

Overridable via env: ROBOCLAW_PORT, ROBOCLAW_BAUD, ROBOCLAW_ADDR.
"""

import os
import select
import struct
import sys
import termios
import time

PORT = os.environ.get('ROBOCLAW_PORT', '/dev/ttyAMA0')
BAUD = int(os.environ.get('ROBOCLAW_BAUD', '115200'))
ADDRESS = int(os.environ.get('ROBOCLAW_ADDR', '128'))

TEST_DUTY = 8192        # ~25% of full PWM (signed duty range is -32767..32767)
DRIVE_SECONDS = 1.5
PAUSE_SECONDS = 0.7

GETVERSION = 21
GETMBATT = 24
M1DUTY = 32             # signed duty, 2-byte payload
M2DUTY = 33

_BAUDS = {9600: termios.B9600, 38400: termios.B38400,
          115200: termios.B115200, 230400: termios.B230400}


class RoboClaw:
    """Minimal RoboClaw packet-serial client over a raw termios UART."""

    def __init__(self, port, baud, address, timeout=0.1):
        self.address = address
        self.timeout = timeout
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = attrs[1] = attrs[3] = 0                 # raw: no in/out/line processing
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = _BAUDS[baud]                 # in/out baud
        attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def close(self):
        os.close(self.fd)

    @staticmethod
    def _crc16(data):
        crc = 0
        for b in data:
            crc ^= b << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        return crc

    def _read(self, n):
        """Read up to n bytes, stopping at the timeout."""
        deadline = time.monotonic() + self.timeout
        buf = bytearray()
        while len(buf) < n and (remaining := deadline - time.monotonic()) > 0:
            if select.select([self.fd], [], [], remaining)[0]:
                buf.extend(os.read(self.fd, n - len(buf)))
        return bytes(buf)

    def _read_value(self, cmd, nbytes):
        """Send a read command; return CRC-verified payload bytes, or None."""
        termios.tcflush(self.fd, termios.TCIFLUSH)
        sent = bytes([self.address, cmd])
        os.write(self.fd, sent)
        resp = self._read(nbytes + 2)
        if len(resp) != nbytes + 2:
            return None
        data, crc = resp[:nbytes], (resp[nbytes] << 8) | resp[nbytes + 1]
        return data if self._crc16(sent + data) == crc else None

    def read_version(self):
        termios.tcflush(self.fd, termios.TCIFLUSH)
        sent = bytes([self.address, GETVERSION])
        os.write(self.fd, sent)
        resp = self._read(50)                              # NUL-terminated string + 2 CRC bytes
        nul = resp.find(0)
        if nul < 0 or len(resp) < nul + 3:
            return None
        string, crc = resp[:nul + 1], (resp[nul + 1] << 8) | resp[nul + 2]
        if self._crc16(sent + string) != crc:
            return None
        return string.rstrip(b'\x00').decode('ascii', 'replace').strip()

    def read_main_battery(self):
        data = self._read_value(GETMBATT, 2)
        return None if data is None else struct.unpack('>H', data)[0] / 10.0

    def drive(self, cmd, duty):
        """Set signed duty (-32767..32767) on M1DUTY or M2DUTY; return True on ack."""
        termios.tcflush(self.fd, termios.TCIFLUSH)
        packet = bytes([self.address, cmd]) + struct.pack('>h', max(-32767, min(32767, duty)))
        crc = self._crc16(packet)
        os.write(self.fd, packet + bytes([crc >> 8, crc & 0xFF]))
        return self._read(1) == b'\xff'

    def stop(self):
        self.drive(M1DUTY, 0)
        self.drive(M2DUTY, 0)


def check_comms(rc):
    version = rc.read_version()
    if version is None:
        print('FAIL: no valid reply — check baud/address/wiring.')
        return False
    print(f'OK  firmware: {version}')
    mbatt = rc.read_main_battery()
    if mbatt is not None:
        print(f'OK  main battery: {mbatt:.1f} V')
    return True


def test_motor(rc, name, cmd):
    print(f'== {name} ==')
    for label, duty in (('forward', TEST_DUTY), ('reverse', -TEST_DUTY)):
        print(f'  {label} {DRIVE_SECONDS}s @ duty {duty}')
        rc.drive(cmd, duty)
        time.sleep(DRIVE_SECONDS)
        rc.drive(cmd, 0)
        time.sleep(PAUSE_SECONDS)


def main():
    comms_only = os.environ.get('ROBOCLAW_COMMS_ONLY', '') not in ('', '0', 'false')
    print(f'RoboClaw on {PORT} @ {BAUD} baud, address {ADDRESS}')
    rc = RoboClaw(PORT, BAUD, ADDRESS)
    try:
        if not check_comms(rc):
            sys.exit(1)
        if comms_only:
            return
        print('\n*** MOTORS WILL SPIN — wheels off the ground! ***')
        for n in range(3, 0, -1):
            print(f'  starting in {n}...')
            time.sleep(1)
        test_motor(rc, 'M1 (right)', M1DUTY)
        test_motor(rc, 'M2 (left)', M2DUTY)
        print('\nDone.')
    finally:
        rc.stop()
        rc.close()


if __name__ == '__main__':
    main()
