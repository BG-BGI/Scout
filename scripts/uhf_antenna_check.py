#!/usr/bin/env python3
"""M7E antenna-path diagnostic — measures return loss instead of guessing.

    python3 scripts/uhf_antenna_check.py [--port /dev/tty.usbserial-XXXX]

Sends Mercury opcode 0x61 option 5 (antenna detect) and option 6 (return
loss, dB per port) — TMR_SR_cmdAntennaDetect / cmdGetAntennaReturnLoss in
mercuryapi serial_reader_l3.c. Interpreting the number (M7E User Guide §
performance spec + mercuryapi antenna detection):

    >= 17 dB  spec-grade match (VSWR <= 1.33) — full sensitivity
    >= 10 dB  detected as "antenna present"; usable, reduced sensitivity
    <  10 dB  the module's high-return-loss protection (FAULT 0x0505)
              territory — open port, bad solder joint, or bad cable
    ~0-2 dB   open circuit: nothing electrically connected

Run it after any antenna/pigtail/mount change (same class of instrument as
camera_health.py). The robot's uhf_node must not hold the port (stop the
robot service first when running on the Pi).
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scout"))
from scout.core import uhf

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: pip3 install pyserial")

OPCODE_GET_ANTENNA_PORT = 0x61


def send(ser, frame, name, timeout=2.0):
    ser.reset_input_buffer()
    ser.write(frame)
    acc = uhf.FrameAccumulator()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for resp in acc.feed(ser.read(256)):
            if resp[2] == frame[2]:
                return resp
    sys.exit(f"{name}: no response (port/switch/power?)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    args = ap.parse_args()
    port = args.port or next(
        (str(p) for pat in ("tty.usbserial-*", "ttyUSB*")
         for p in sorted(Path("/dev").glob(pat))), None)
    if not port:
        sys.exit("no serial port found — pass --port")

    ser = serial.Serial(port, 115200, timeout=0.05)
    time.sleep(0.3)
    ser.write(uhf.cmd_stop_continuous())  # in case a session died mid-scan
    time.sleep(0.2)

    v = send(ser, uhf.cmd_version(), "version")
    print(f"module answered, fw payload {v[5:-2].hex().upper()}")
    # Region must be set before RF measurements or the module errors.
    send(ser, uhf.cmd_set_region(), "set region")

    det = send(ser, uhf.build_command(OPCODE_GET_ANTENNA_PORT, b"\x05"),
               "antenna detect")
    print(f"antenna detect  (status 0x{uhf.frame_status(det):04X}): "
          f"payload {det[5:-2].hex().upper() or '-'}")

    rl = send(ser, uhf.build_command(OPCODE_GET_ANTENNA_PORT, b"\x06"),
              "return loss")
    payload = rl[5:-2]
    print(f"return loss     (status 0x{uhf.frame_status(rl):04X}): "
          f"payload {payload.hex().upper() or '-'}")
    # Payload after the echoed option byte: (port, dB) pairs.
    pairs = payload[1:] if payload and payload[0] == 0x06 else payload
    for i in range(0, len(pairs) - 1, 2):
        db = pairs[i + 1]
        verdict = ("spec-grade" if db >= 17 else
                   "usable" if db >= 10 else
                   "BAD — open/mismatched path" if db >= 3 else
                   "OPEN CIRCUIT")
        print(f"  port {pairs[i]}: {db} dB  -> {verdict}")
    ser.close()


if __name__ == "__main__":
    main()
