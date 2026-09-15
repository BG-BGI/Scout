#!/usr/bin/env python3
"""M7E Hecto bench validation — run on a Mac (or the Pi) with the SparkFun
board on USB-C. Proves framing, config handshake, continuous read, RSSI and
phase BEFORE any ROS work, and captures raw hex fixtures for test_uhf.py.

    python3 scripts/uhf_bench.py [--port /dev/tty.usbserial-XXXX]
                                 [--power 2000] [--seconds 0]

Board prep: UART slide switch in the USB position. macOS 13+ has a built-in
CH340 driver; the port shows up as /dev/tty.usbserial-* (on Linux /dev/ttyUSB*).

Prints one line per tag read (epc / rssi / freq / phase / dt), keepalives as
dots, and warnings on temp-throttle (0x0504) / high-return-loss (0x0505).
--capture FILE appends the raw hex of every frame — paste interesting ones
(varied EPC lengths, embedded data, throttle) into scout/test/test_uhf.py.

Ctrl+C sends the stop command and prints a summary (reads/s, per-EPC counts,
RSSI range) — the step-9 bench acceptance numbers come from here.
"""

import argparse
import collections
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "scout"))
from scout.core import uhf

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: pip3 install pyserial")


def find_port():
    for pattern in ("tty.usbserial-*", "ttyUSB*"):
        hits = sorted(Path("/dev").glob(pattern))
        if hits:
            return str(hits[0])
    sys.exit("no CH340 port found — pass --port (is the UART switch on USB?)")


def send_expect(ser, frame, name):
    """Send a command, read until its response frame arrives (2 s timeout)."""
    ser.reset_input_buffer()
    ser.write(frame)
    acc = uhf.FrameAccumulator()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        for resp in acc.feed(ser.read(256)):
            if resp[2] == frame[2]:
                status = uhf.frame_status(resp)
                print(f'{name}: status=0x{status:04X} '
                      f'payload={resp[5:-2].hex().upper() or "-"}')
                if status != 0:
                    sys.exit(f"{name} failed with status 0x{status:04X}")
                return resp
    sys.exit(f"{name}: no response — wrong port, wrong switch position, "
             f"or not an M7E (a lidar answers nothing here)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--power", type=int, default=2000,
                    help="read power, centi-dBm (clamped to 2000 in core.uhf)")
    ap.add_argument("--seconds", type=float, default=0,
                    help="stop after N seconds (0 = until Ctrl+C)")
    ap.add_argument("--capture", default=None,
                    help="append raw frame hex to this file (fixture source)")
    args = ap.parse_args()

    port = args.port or find_port()
    print(f"opening {port} @ 115200")
    ser = serial.Serial(port, 115200, timeout=0.05)
    time.sleep(0.3)  # module spits its boot version ~200 ms after power-on

    send_expect(ser, uhf.cmd_version(), "version")
    send_expect(ser, uhf.cmd_set_region(), "set region NA")
    send_expect(ser, uhf.cmd_set_tag_protocol(), "set protocol GEN2")
    send_expect(ser, uhf.cmd_set_antenna_port(), "set antenna port")
    send_expect(ser, uhf.cmd_disable_read_filter(), "disable read filter")
    send_expect(ser, uhf.cmd_set_read_power(args.power),
                f"set read power {min(args.power, uhf.READ_POWER_MAX_CDBM)}")

    print("starting continuous read — Ctrl+C to stop\n")
    ser.write(uhf.cmd_start_continuous())

    cap = open(args.capture, "a") if args.capture else None
    acc = uhf.FrameAccumulator()
    counts = collections.Counter()
    rssi_min, rssi_max, n_reads, n_keepalive = 0, -999, 0, 0
    t0 = time.monotonic()
    try:
        while not args.seconds or time.monotonic() - t0 < args.seconds:
            for frame in acc.feed(ser.read(4096)):
                if cap:
                    cap.write(frame.hex().upper() + "\n")
                kind = uhf.classify_frame(frame)
                if kind == "tag":
                    r = uhf.parse_tag_record(frame)
                    n_reads += 1
                    counts[r["epc"]] += 1
                    rssi_min = min(rssi_min, r["rssi_dbm"])
                    rssi_max = max(rssi_max, r["rssi_dbm"])
                    print(f"epc[{r['epc']}] rssi[{r['rssi_dbm']:4d}] "
                          f"freq[{r['freq_khz']}] phase[{r['phase']:5d}] "
                          f"dt[{r['timestamp_ms']}]")
                elif kind == "keepalive":
                    n_keepalive += 1
                    print(".", end="", flush=True)
                elif kind in ("temp_throttle", "high_return_loss"):
                    print(f"\n⚠ {kind} (frame {frame.hex().upper()})")
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(uhf.cmd_stop_continuous())
        time.sleep(0.2)
        ser.close()
        if cap:
            cap.close()

    dt = time.monotonic() - t0
    print(f"\n--- {dt:.1f} s: {n_reads} reads ({n_reads / dt:.1f}/s), "
          f"{n_keepalive} keepalives, {len(counts)} distinct EPCs, "
          f"RSSI {rssi_min}..{rssi_max} dBm")
    for epc, n in counts.most_common():
        print(f"  {epc}  x{n}")


if __name__ == "__main__":
    main()
