#!/usr/bin/env python3
"""Standalone APA102 LED strip driver for the Raspberry Pi 5 — no ROS.

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

Requires SPI enabled (dtparam=spi=on) and the `spidev` module. Run directly:

    python3 led_test.py
"""

import math
import time

import spidev

# --- Configuration ----------------------------------------------------------
NUM_LEDS = 131                # actual strip length
SPI_BUS, SPI_DEVICE = 0, 0    # /dev/spidev0.0 = RP1 SPI0 on the header (GPIO10 MOSI, GPIO11 SCLK)
SPI_HZ = 1_000_000            # 1 MHz: slow clock tolerates 3.3V logic straight into the strip
DEFAULT_BRIGHTNESS = 8        # 5-bit global brightness (0-31); low = low current


class APA102:
    """Minimal APA102 strip driver over a raw spidev SPI device.

    Colors are staged in an in-memory buffer (one 4-byte LED frame each) and
    flushed to the strip on show(). Each LED frame is stored on the wire as
    [0xE0 | brightness, BLUE, GREEN, RED] — note the B-G-R byte order.
    """

    def __init__(self, num_leds=NUM_LEDS, bus=SPI_BUS, device=SPI_DEVICE,
                 speed_hz=SPI_HZ, brightness=DEFAULT_BRIGHTNESS):
        self.num_leds = num_leds
        self.brightness = max(0, min(31, brightness))
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = speed_hz
        self.spi.mode = 0                      # APA102 is CPOL=0, CPHA=0
        # Per-LED frame buffer: [brightness_byte, blue, green, red] * num_leds.
        self._buf = [[0xE0 | self.brightness, 0, 0, 0]
                     for _ in range(num_leds)]

    def set_pixel(self, i, r, g, b):
        """Stage RGB for LED `i` (stored B-G-R). Out-of-range index is ignored."""
        if 0 <= i < self.num_leds:
            frame = self._buf[i]
            frame[1] = b & 0xFF
            frame[2] = g & 0xFF
            frame[3] = r & 0xFF

    def set_all(self, r, g, b):
        """Stage the same RGB color on every LED."""
        for i in range(self.num_leds):
            self.set_pixel(i, r, g, b)

    def set_brightness(self, brightness):
        """Set the 5-bit global brightness (0-31) on every LED."""
        self.brightness = max(0, min(31, brightness))
        byte = 0xE0 | self.brightness
        for frame in self._buf:
            frame[0] = byte

    def show(self):
        """Assemble the full APA102 protocol frame and clock it out over SPI.

        Frame layout:
            Start frame:  4 bytes of 0x00
            LED frames:   per LED -> [0xE0 | brightness, BLUE, GREEN, RED]
            End frame:    ceil(NUM_LEDS / 16) bytes of 0xFF
                          (extra clock edges to latch the last LEDs)
        """
        data = [0x00, 0x00, 0x00, 0x00]
        for frame in self._buf:
            data.extend(frame)
        end_frame_len = math.ceil(self.num_leds / 16)
        data.extend([0xFF] * end_frame_len)
        self.spi.xfer2(data)

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


def _demo(strip):
    """Solid color, a brightness sweep, then a single dot running the strip."""
    print('Solid green')
    strip.set_brightness(DEFAULT_BRIGHTNESS)
    strip.set_all(0, 255, 0)
    strip.show()
    time.sleep(2.0)

    print('Brightness sweep')
    strip.set_all(0, 0, 255)                   # solid blue to sweep
    for level in list(range(0, 32)) + list(range(30, -1, -1)):
        strip.set_brightness(level)
        strip.show()
        time.sleep(0.04)

    print('Moving dot')
    strip.set_brightness(DEFAULT_BRIGHTNESS)
    for i in range(strip.num_leds):
        strip.set_all(0, 0, 0)
        strip.set_pixel(i, 255, 0, 0)
        strip.show()
        time.sleep(0.05)


def main():
    print(f'APA102 on /dev/spidev{SPI_BUS}.{SPI_DEVICE}, '
          f'{NUM_LEDS} LEDs @ {SPI_HZ / 1e6:.1f} MHz')
    strip = APA102()
    try:
        _demo(strip)
        print('Done.')
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        strip.close()


if __name__ == '__main__':
    main()
