#!/usr/bin/env python3
"""Standalone APA102 LED strip bench test for the Raspberry Pi 5 — no ROS.

The APA102 driver itself lives in `apa102.py` (shared with the ROS led_node).
This script just exercises it: a solid color, a brightness sweep, and a moving
dot. Run directly on the Pi:

    python3 led_test.py

Requires SPI enabled (dtparam=spi=on, reboot) and the `spidev` module. See
apa102.py for the full wiring / SPI-bus notes.
"""

import time

from apa102 import APA102, DEFAULT_BRIGHTNESS, NUM_LEDS, SPI_BUS, SPI_DEVICE, SPI_HZ


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
