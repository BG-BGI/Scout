"""Flipper Zero USB-CDC serial driver (plain module, no ROS — apa102 pattern).

The Flipper enumerates as CDC-ACM (/dev/ttyACM0 in the robot container; the
/dev/serial/by-id symlink farm does NOT exist in-container — see rplidar.yaml).
CDC ignores the baud, 230400 is the documented convention. The shell is a
human terminal: it echoes input, prompts `>:` when idle, and a long-running
command (rfid read) holds the line until Ctrl+C (0x03).

flipper_node owns exactly one instance and is the sole caller (single-timer
I/O, led_node pattern). serial.SerialException/OSError propagate — the node
maps them to its DISCONNECTED state.
"""

import time

import serial

from scout.core.rfid import has_prompt

CTRL_C = b'\x03'


class FlipperCli:
    """One exclusive handle on the Flipper's CLI serial port."""

    def __init__(self, port, baud):
        self._port = port
        self._baud = baud
        self._ser = None

    @property
    def connected(self):
        return self._ser is not None

    def open(self, settle_s=2.0):
        """Open the port and return it to an idle prompt: Ctrl+C + CR recovers
        a Flipper left mid-`rfid read` by a node crash, then the output is
        drained until the prompt appears (bounded by settle_s). Returns True
        when the prompt was seen — False means something answered the port
        but not like a Flipper shell."""
        self._ser = serial.Serial(self._port, self._baud, timeout=0)
        self._ser.write(CTRL_C + b'\r')
        return self.drain_to_prompt(settle_s)

    def drain_to_prompt(self, timeout_s):
        """Discard output until a `>:` prompt arrives (True) or timeout_s
        elapses (False). Blocking, bounded — callers keep timeout_s short."""
        deadline = time.monotonic() + timeout_s
        buf = ''
        while time.monotonic() < deadline:
            buf += self.read_available()
            if has_prompt(buf):
                return True
            time.sleep(0.02)
        return False

    def read_available(self):
        """Everything currently buffered, decoded leniently (the banner has
        box-drawing bytes on some firmware)."""
        n = self._ser.in_waiting
        if not n:
            return ''
        return self._ser.read(n).decode('utf-8', errors='replace')

    def send_line(self, command):
        self._ser.write(command.encode('utf-8') + b'\r')

    def send_ctrl_c(self):
        self._ser.write(CTRL_C)

    def close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None
