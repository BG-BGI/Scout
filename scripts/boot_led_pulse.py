#!/usr/bin/env python3
"""Pre-container boot pulse — the dev-mode arm window (docs/deploy.md).

Breathes the APA102 strip blue, PULSES cycles over WINDOW_S seconds, ending
dark. Runs from scout-bootpulse.service BEFORE docker.service: a power cut
mid-pulse leaves bootmode.sh's `interrupted` flag on disk, and the NEXT
boot enters dev mode (webui + fleet_status only).

Stdlib only — the robot container's spidev is not available on the host, so
this opens /dev/spidev0.0 and pokes it with raw ioctls. Wire format mirrors
scout/scout/apa102.py exactly (start frame, [0xE0|brightness, B, G, R] per
LED, ceil(n/16) 0xFF tail). If the SPI device never appears the script
still holds the 5 s window — the arm-window semantics must not depend on
the strip being wired.
"""
import fcntl
import math
import os
import struct
import time

SPI_DEV = "/dev/spidev0.0"
NUM_LEDS = 131          # apa102.py NUM_LEDS — the physical strip length
SPI_HZ = 1_000_000      # apa102.py SPI_HZ
BRIGHTNESS = 8          # apa102.py DEFAULT_BRIGHTNESS (5-bit global)
PULSES = 5
WINDOW_S = 5.0
FPS = 40
OPEN_RETRY_S = 10.0

# linux/spi/spidev.h ioctl numbers (stable ABI).
SPI_IOC_WR_MODE = 0x40016B01
SPI_IOC_WR_MAX_SPEED_HZ = 0x40046B04

_START = 4
_END = (NUM_LEDS + 15) // 16


def _open_spi():
    """fd on the strip's SPI device, or None if it never appears."""
    deadline = time.monotonic() + OPEN_RETRY_S
    while True:
        try:
            fd = os.open(SPI_DEV, os.O_WRONLY)
        except OSError:
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.5)
            continue
        fcntl.ioctl(fd, SPI_IOC_WR_MODE, b"\x00")    # CPOL=0, CPHA=0
        fcntl.ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, struct.pack("I", SPI_HZ))
        return fd


def _frame(blue):
    buf = bytearray(_START + NUM_LEDS * 4 + _END)
    for i in range(NUM_LEDS):
        o = _START + i * 4
        buf[o] = 0xE0 | BRIGHTNESS
        buf[o + 1] = blue           # B; G and R stay 0
    for i in range(len(buf) - _END, len(buf)):
        buf[i] = 0xFF
    return buf


def main():
    fd = _open_spi()
    if fd is None:
        # No strip: hold the window anyway so the gesture still arms dev.
        time.sleep(WINDOW_S)
        return
    start = time.monotonic()
    while True:
        t = time.monotonic() - start
        if t >= WINDOW_S:
            break
        # sin^2: 0 -> 255 -> 0 once per second, 5 peaks, ends dark.
        blue = round(255 * math.sin(math.pi * PULSES * t / WINDOW_S) ** 2)
        os.write(fd, _frame(blue))
        time.sleep(1.0 / FPS)
    os.write(fd, _frame(0))
    os.close(fd)


if __name__ == "__main__":
    main()
