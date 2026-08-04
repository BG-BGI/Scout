#!/usr/bin/env python3
"""Shared APA102 LED strip driver for the Raspberry Pi 5.

The APA102 is an SPI-driven addressable LED (two wires: DATA + CLOCK), unlike
the WS2812/NeoPixel family which needs precise bit-banged timing. On the Pi 5
the GPIO is behind the RP1 chip, so the rpi_ws281x / Adafruit Blinka WS2812
paths do NOT apply here. We talk straight to /dev/spidev0.0 via `spidev`, which
is a plain kernel SPI device. On the Pi 5 the header SPI0 lives on the RP1 and
enumerates as bus 0 once `dtparam=spi=on` is set (reboot required). Do NOT use
/dev/spidev10.0 — that is the BCM2712 SoC's internal SPI, not on the header.

Wiring:
    DATA  -> GPIO10 / SPI0 MOSI (physical pin 19)
    CLOCK -> GPIO11 / SPI0 SCLK (physical pin 23)
    Strip 5V/GND from an external 5V/10A buck converter (NOT the GPIO header).
    Common ground between the buck converter and the Pi.

This module is dependency-light (spidev + stdlib) so it can be imported by both
the standalone bench test (led_test.py) and the ROS node (led_node.py).
"""

import math

import spidev

# --- Configuration ----------------------------------------------------------
NUM_LEDS = 131                # actual strip length
SPI_BUS, SPI_DEVICE = 0, 0    # /dev/spidev0.0 = RP1 SPI0 on the header (GPIO10 MOSI, GPIO11 SCLK)
SPI_HZ = 1_000_000            # 1 MHz: slow clock tolerates 3.3V logic straight into the strip
DEFAULT_BRIGHTNESS = 8        # 5-bit global brightness (0-31); low = low current

# Visual white-balance (not current limiting). Equal R=G=B looks cool/blue on
# these APA102s because blue/green dies outshine red at the same PWM.
# Tuned so #FFFFFF is neutral and #FF9900 stays deep orange (not tennis-ball).
R_GAIN = 1.00
G_GAIN = 0.28
B_GAIN = 0.27
# Push chroma away from gray after gains (fights slight white-wash on mixed colors).
SATURATION = 1.15

# Rough per-LED current at full white, full (31/31) brightness. From CLAUDE.md:
# 131 LEDs * ~60 mA ~= 7.9 A at full white. Used for the node's current budget.
LED_FULL_WHITE_AMPS = 0.060

_START_LEN = 4
_LED_FRAME = 4  # brightness, B, G, R


def _wb(r, g, b):
    """Channel gains, then saturation boost; clamp to 0..255."""
    r = (r & 0xFF) * R_GAIN
    g = (g & 0xFF) * G_GAIN
    b = (b & 0xFF) * B_GAIN
    gray = (r + g + b) / 3.0
    r = gray + SATURATION * (r - gray)
    g = gray + SATURATION * (g - gray)
    b = gray + SATURATION * (b - gray)
    return (
        max(0, min(255, int(round(r)))),
        max(0, min(255, int(round(g)))),
        max(0, min(255, int(round(b)))),
    )


class APA102:
    """Minimal APA102 strip driver over a raw spidev SPI device.

    Colors are staged in a preallocated wire buffer and flushed on show().
    Each LED frame on the wire is [0xE0 | brightness, BLUE, GREEN, RED].
    """

    def __init__(self, num_leds=NUM_LEDS, bus=SPI_BUS, device=SPI_DEVICE,
                 speed_hz=SPI_HZ, brightness=DEFAULT_BRIGHTNESS):
        self.num_leds = num_leds
        self.brightness = max(0, min(31, brightness))
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0                      # APA102 is CPOL=0, CPHA=0

        end_len = math.ceil(num_leds / 16)
        self._buf = bytearray(_START_LEN + num_leds * _LED_FRAME + end_len)
        # Start frame already zero; end frame must be 0xFF.
        for i in range(len(self._buf) - end_len, len(self._buf)):
            self._buf[i] = 0xFF
        bright = 0xE0 | self.brightness
        for i in range(num_leds):
            self._buf[_START_LEN + i * _LED_FRAME] = bright

    def _led_offset(self, i):
        return _START_LEN + i * _LED_FRAME

    def set_pixel(self, i, r, g, b):
        """Stage RGB for LED `i` (stored B-G-R). Out-of-range index is ignored."""
        if 0 <= i < self.num_leds:
            r, g, b = _wb(r, g, b)
            off = self._led_offset(i)
            self._buf[off + 1] = b
            self._buf[off + 2] = g
            self._buf[off + 3] = r

    def set_all(self, r, g, b):
        """Stage the same RGB color on every LED."""
        rb, gb, bb = _wb(r, g, b)
        for i in range(self.num_leds):
            off = self._led_offset(i)
            self._buf[off + 1] = bb
            self._buf[off + 2] = gb
            self._buf[off + 3] = rb

    def set_brightness(self, brightness):
        """Set the 5-bit global brightness (0-31) on every LED."""
        self.brightness = max(0, min(31, brightness))
        byte = 0xE0 | self.brightness
        for i in range(self.num_leds):
            self._buf[self._led_offset(i)] = byte

    def show(self):
        """Clock the prebuilt APA102 frame out over SPI."""
        self.spi.xfer2(self._buf)

    def clear(self):
        """Turn every LED off and flush."""
        self.set_all(0, 0, 0)
        self.show()

    def close(self):
        """Blank the strip and release the SPI device."""
        try:
            self.clear()
        finally:
            self.spi.close()
