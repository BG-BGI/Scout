#!/usr/bin/env python3
"""ROS 2 node driving the APA102 LED strip via a SetLedMode service.

Design (see the LED integration notes in the repo):
  * The node owns the single APA102 SPI driver instance.
  * The SetLedMode service ONLY mutates the target state and returns promptly —
    it never blocks and never touches SPI directly.
  * A render timer is the SOLE caller of the driver's show(), so SPI is written
    from exactly one place (safe under rclpy's default single-threaded executor).

Safety: the strip shares a non-isolated 5 V buck with the Pi. Per CLAUDE.md,
131 LEDs at full white / brightness 31 draw ~7.9 A and can brown out the Pi.
So requested brightness is clamped to `max_brightness`, and if the estimated
worst-case current for the requested color/brightness exceeds `current_budget_amps`
the brightness is scaled down further (and the caller is told in the response).
"""

import colorsys
import math
import time

from rclpy.node import Node

from scout.apa102 import (APA102, LED_FULL_WHITE_AMPS, NUM_LEDS,
                          SPI_BUS, SPI_DEVICE, SPI_HZ)
from scout.core.colors import parse_hex_color
from scout.node_util import run_node
from scout.robot_profile import load as _load_profile
from scout_interfaces.srv import SetLedMode

VALID_MODES = tuple(_load_profile()['led_modes'])

CHASE_SEGMENT = 6          # LEDs lit in the chase "comet"
DEFAULT_SPEED = 1.0        # used when a request passes speed <= 0


class LedNode(Node):
    """Serves SetLedMode and renders the current mode on a fixed timer."""

    def __init__(self):
        super().__init__('led_node')

        # --- parameters ------------------------------------------------------
        self.declare_parameter('num_leds', NUM_LEDS)
        self.declare_parameter('spi_bus', SPI_BUS)
        self.declare_parameter('spi_device', SPI_DEVICE)
        self.declare_parameter('spi_hz', SPI_HZ)
        self.declare_parameter('render_hz', 30.0)
        # Safety caps. max_brightness bounds the 5-bit global brightness (a
        # request's brightness % maps onto 0..max_brightness); the current
        # budget is a soft ceiling on estimated strip draw (amps).
        self.declare_parameter('max_brightness', 16)
        self.declare_parameter('current_budget_amps', 5.0)
        # Initial brightness as a percentage (0-100) of max_brightness.
        self.declare_parameter('default_brightness_pct', 50.0)

        p = self.get_parameter
        self._num_leds = int(p('num_leds').value)
        self._render_hz = float(p('render_hz').value)
        self._max_brightness = max(0, min(31, int(p('max_brightness').value)))
        self._current_budget = float(p('current_budget_amps').value)

        # --- driver ----------------------------------------------------------
        init_brightness, _ = self._pct_to_brightness(
            float(p('default_brightness_pct').value), 255, 255, 255)
        self._strip = APA102(
            num_leds=self._num_leds,
            bus=int(p('spi_bus').value),
            device=int(p('spi_device').value),
            speed_hz=int(p('spi_hz').value),
            brightness=init_brightness,
        )

        # --- render state ----------------------------------------------------
        self._mode = 'off'
        self._color = (0, 0, 0)
        self._brightness = self._strip.brightness
        self._speed = DEFAULT_SPEED
        self._phase = 0.0                 # advances every tick for animations
        self._last_tick = time.monotonic()
        self._dirty = True                # force an initial blank show()

        self._srv = self.create_service(SetLedMode, 'set_led_mode',
                                         self._on_set_led_mode)
        self.create_timer(1.0 / self._render_hz, self._render)

        self.get_logger().info(
            'LED node up: %d LEDs, render %.0f Hz, max_brightness %d/31, '
            'current budget %.1f A. Call /set_led_mode to drive the strip.'
            % (self._num_leds, self._render_hz, self._max_brightness,
               self._current_budget))

    # --- safety --------------------------------------------------------------
    def _estimate_amps(self, r, g, b, brightness):
        """Conservative worst-case draw if every LED showed (r,g,b) at brightness.

        Linear model from CLAUDE.md: full white (r=g=b=255) at brightness 31
        ~= LED_FULL_WHITE_AMPS per LED. Scales with color sum and brightness.
        """
        color_frac = (r + g + b) / (3.0 * 255.0)
        bright_frac = brightness / 31.0
        return self._num_leds * LED_FULL_WHITE_AMPS * color_frac * bright_frac

    def _pct_to_brightness(self, pct, r=255, g=255, b=255):
        """Map a brightness percentage onto a safe 5-bit global brightness.

        pct (0-100) scales the node's max_brightness, then the result is backed
        off further if the estimated worst-case draw for this color exceeds the
        current budget. Returns (safe_brightness, note); note is '' if the
        current budget did not force a reduction.
        """
        pct = max(0.0, min(100.0, float(pct)))
        level = int(round(pct / 100.0 * self._max_brightness))

        # If this level would exceed the budget for this color, back it off
        # until the estimate fits (or hits 0).
        b_final = level
        while b_final > 0 and self._estimate_amps(r, g, b, b_final) > self._current_budget:
            b_final -= 1
        note = ''
        if b_final < level:
            note = ('brightness reduced %d->%d/31 for ~%.1f A current budget'
                    % (level, b_final, self._current_budget))
        return b_final, note

    # --- service -------------------------------------------------------------
    def _on_set_led_mode(self, request, response):
        mode = (request.mode or '').strip().lower()
        if mode not in VALID_MODES:
            response.success = False
            response.message = ("unknown mode '%s'; valid: %s"
                                % (request.mode, ', '.join(VALID_MODES)))
            self.get_logger().warn(response.message)
            return response

        try:
            r, g, b = parse_hex_color(request.color)
        except ValueError as exc:
            response.success = False
            response.message = 'bad color: %s' % exc
            self.get_logger().warn(response.message)
            return response

        safe_brightness, note = self._pct_to_brightness(request.brightness, r, g, b)
        speed = float(request.speed) if request.speed > 0.0 else DEFAULT_SPEED

        self._mode = mode
        self._color = (r, g, b)
        self._brightness = safe_brightness
        self._speed = speed
        self._phase = 0.0
        self._strip.set_brightness(safe_brightness)
        self._dirty = True

        msg = "mode='%s' color=#%02X%02X%02X brightness=%d%%->%d/31 speed=%.2f" % (
            mode, r, g, b, int(request.brightness), safe_brightness, speed)
        if note:
            msg += ' [' + note + ']'
        response.success = True
        response.message = msg
        self.get_logger().info(msg)
        return response

    # --- rendering (sole SPI writer) -----------------------------------------
    def _render(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        self._phase += dt * self._speed

        mode = self._mode
        animated = mode in ('blink', 'breathe', 'rainbow', 'chase')
        if not self._dirty and not animated:
            return

        if mode == 'off':
            self._strip.set_all(0, 0, 0)
        elif mode == 'solid':
            self._strip.set_all(*self._color)
        elif mode == 'blink':
            on = (self._phase % 1.0) < 0.5
            self._strip.set_all(*(self._color if on else (0, 0, 0)))
        elif mode == 'breathe':
            # Pulse global brightness with a raised cosine, 0..target.
            level = 0.5 - 0.5 * math.cos(2.0 * math.pi * self._phase)
            self._strip.set_all(*self._color)
            self._strip.set_brightness(int(round(self._brightness * level)))
        elif mode == 'rainbow':
            self._render_rainbow()
        elif mode == 'chase':
            self._render_chase()

        # A transient SPI timeout (TimeoutError/errno 110 from xfer2 under bus
        # contention or EMI) must not kill the node — drop the frame, stay
        # dirty so a static mode retries next tick.
        try:
            self._strip.show()
        except (TimeoutError, OSError) as exc:
            self.get_logger().warn('SPI show() failed, frame dropped: %s' % exc,
                                   throttle_duration_sec=5.0)
            self._dirty = True
            return
        # Static modes only need SPI once until the next SetLedMode.
        self._dirty = animated

    def _render_rainbow(self):
        n = self._num_leds
        offset = self._phase % 1.0
        for i in range(n):
            hue = (i / n + offset) % 1.0
            r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
            self._strip.set_pixel(i, int(r * 255), int(g * 255), int(b * 255))

    def _render_chase(self):
        n = self._num_leds
        self._strip.set_all(0, 0, 0)
        head = int(self._phase * n) % n
        cr, cg, cb = self._color
        for k in range(CHASE_SEGMENT):
            # Trailing comet: dim the tail linearly behind the head.
            fade = (CHASE_SEGMENT - k) / CHASE_SEGMENT
            idx = (head - k) % n
            self._strip.set_pixel(idx, int(cr * fade), int(cg * fade), int(cb * fade))

    def shutdown(self):
        """Blank the strip and release SPI (mirror of joystick_teleop.stop())."""
        try:
            self._strip.close()
        except Exception:  # noqa: BLE001 — best-effort cleanup on shutdown
            pass


def main(args=None):
    run_node(LedNode, on_shutdown=lambda n: n.shutdown(), args=args)


if __name__ == '__main__':
    main()
