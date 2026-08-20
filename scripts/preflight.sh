#!/usr/bin/env bash
# preflight.sh — non-destructive Pi host baseline asserts (docs/platform.md).
# Read-only: safe to run mid-mission. Exit 0 = all green; nonzero = failures
# listed on stderr. Run on the Pi host, not inside a container.
set -u

FAIL=0
fail() { echo "FAIL: $*" >&2; FAIL=1; }
ok()   { echo "  ok: $*"; }

# 64-bit kernel
[ "$(uname -m)" = "aarch64" ] && ok "aarch64 kernel" || fail "kernel is $(uname -m), want aarch64"

# Boot config: UART on GPIO14/15 (RoboClaw) + SPI0 (APA102)
CFG=/boot/firmware/config.txt
if [ -r "$CFG" ]; then
  grep -Eq '^\s*dtparam=uart0=on' "$CFG" && ok "uart0=on" || fail "dtparam=uart0=on missing in $CFG"
  grep -Eq '^\s*dtparam=spi=on' "$CFG" && ok "spi=on" || fail "dtparam=spi=on missing in $CFG"
  grep -Eq '^\s*usb_max_current_enable=1' "$CFG" && ok "usb_max_current_enable=1" || fail "usb_max_current_enable=1 missing in $CFG"
else
  fail "$CFG unreadable"
fi

# Devices (spec §3.1): RoboClaw UART, RPLIDAR USB-UART, D455, joystick, SPI0
[ -e /dev/ttyAMA0 ] && ok "/dev/ttyAMA0 (RoboClaw)" || fail "/dev/ttyAMA0 missing (RoboClaw UART)"
[ -e /dev/ttyUSB0 ] && ok "/dev/ttyUSB0 (RPLIDAR)" || fail "/dev/ttyUSB0 missing (RPLIDAR CP2102)"
[ -e /dev/spidev0.0 ] && ok "/dev/spidev0.0 (APA102)" || fail "/dev/spidev0.0 missing (header SPI — /dev/spidev10.0 does NOT count)"
if command -v lsusb >/dev/null; then
  lsusb | grep -qi '8086:0b5c\|RealSense' && ok "D455 on USB" || fail "D455 not on USB (lsusb)"
else
  fail "lsusb unavailable, cannot check D455"
fi
[ -e /dev/input/js0 ] && ok "/dev/input/js0 (gamepad)" || echo "warn: /dev/input/js0 missing (gamepad off is fine if not needed)" >&2

# Pin mux: SPI pins actually ALT0, not just device node present (CLAUDE.md trap)
if command -v pinctrl >/dev/null; then
  MUX=$(pinctrl get 10,11 2>/dev/null)
  echo "$MUX" | grep -q 'a0' && ok "GPIO10/11 muxed a0 (SPI)" || fail "GPIO10/11 not ALT0 — SPI not on header pins: $MUX"
fi

# Clock: no RTC, NTP must be synced before sensor stamps mean anything
timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -q yes \
  && ok "NTP synced" || fail "NTP not synced (timedatectl)"

# Docker
systemctl is-active --quiet docker && ok "docker.service active" || fail "docker.service not active"

# Thermals / throttling
if command -v vcgencmd >/dev/null; then
  T=$(vcgencmd measure_temp 2>/dev/null | grep -o '[0-9.]*')
  TH=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)
  awk "BEGIN{exit !($T < 80)}" && ok "temp ${T}C" || fail "temp ${T}C >= 80C"
  [ "$TH" = "0x0" ] && ok "no throttle flags" || fail "throttled=$TH (bit 0 undervolt now, 16 undervolt since boot)"
else
  fail "vcgencmd unavailable"
fi

[ $FAIL -eq 0 ] && echo "preflight: ALL GREEN" || echo "preflight: FAILURES ABOVE" >&2
exit $FAIL
