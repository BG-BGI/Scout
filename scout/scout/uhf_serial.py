"""M7E Hecto USB-serial driver (plain module, no ROS — flipper_cli pattern).

The SparkFun board's CH340C enumerates as /dev/ttyUSB*, racing the RPLIDAR's
CP2102 for probe order, so the port is pinned via the /dev/serial/by-id path
(the robot service bind-mounts /dev/serial from the host — see rplidar.yaml
and docker-compose.yaml). The wire is binary ThingMagic Mercury frames
(scout.core.uhf), NOT a text shell — bytes in, bytes out, no decoding.

uhf_node owns exactly one instance and is the sole caller (single-timer I/O,
flipper_cli pattern). serial.SerialException/OSError propagate — the node maps
them to its DISCONNECTED state.
"""

import serial


class UhfSerial:
    """One exclusive handle on the M7E's serial port."""

    def __init__(self, port, baud):
        self._port = port
        self._baud = baud
        self._ser = None

    @property
    def connected(self):
        return self._ser is not None

    def open(self):
        """Open the port; the module answers commands ~200 ms after power-on.
        Whether an M7E is really on the other end is the version handshake's
        job (uhf_node) — a lidar on a mispinned port just never answers."""
        self._ser = serial.Serial(self._port, self._baud, timeout=0)
        self._ser.reset_input_buffer()

    def read_available(self):
        """Everything currently buffered, raw bytes (b'' when idle)."""
        n = self._ser.in_waiting
        if not n:
            return b''
        return self._ser.read(n)

    def write(self, frame):
        self._ser.write(frame)

    def close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None
