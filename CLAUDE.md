# Skid-Steer Robot

## Hardware

**Compute**
- Raspberry Pi 5 16 GB (https://www.adafruit.com/product/6125)

**Motors**
- Four Pololu #4693 37D gearmotors (50:1, 24V) with quadrature encoders (https://www.pololu.com/product/4693)
- 64 CPR motor shaft → 3200 counts/wheel rev. Datasheet: 200 RPM / 100 mA no-load, 3 A stall @ 24V → ~8 Ω per motor, ~4 Ω per paralleled channel
- Per-channel physics: ~0.2 A no-load, ~5 A hard-stall ceiling, ~10 A worst-case plugging transient. Nothing sustained above ~5 A/channel can be real

**Motor Controller**
- RoboClaw 2x30A, firmware 4.4.9 (https://www.basicmicro.com/RoboClaw-2x30A-Motor-Controller_p_9.html)
- Drives all four motors. Two motors paralleled per channel
- BasicMicro RoboClaw Screw Terminal Adapter (V1) for the I/O header
- Only the two rear encoders wired (weight/battery sits at the rear); front encoders left unconnected
- Controlled from the Pi over the **GPIO UART** (packet serial, GPIO14/15 → /dev/ttyAMA0). USB not used

**LED Strip**
- APA102 addressable strip, 131 LEDs. **SPI-driven (separate DATA + CLOCK)** — it is NOT a WS2812/NeoPixel, so the rpi_ws281x / bit-banged timing libraries do not apply
- Wiring: DATA → GPIO10 / SPI0 MOSI (pin 19), CLOCK → GPIO11 / SPI0 SCLK (pin 23), into the strip's **DI/CI input end** (arrows point away from it)
- Powered from its own 5V/10A buck (NOT the GPIO header). Shares ground with the Pi through the non-isolated buck; a dedicated Pi-GND→strip-GND jumper alongside DATA/CLOCK tightens the signal return
- **3.3V GPIO drives it directly — no level shifter needed** (verified; the strip also ran off a 3.3V ESP32). Confirmed working at 1 MHz SPI
- Current budget: 131 LEDs × ~60 mA ≈ **7.9 A at full white**. Fine at the default low brightness (8/31), but do not `set_all(255,255,255)` at brightness 31 on a buck shared with the Pi — it can brown out and reset the Pi

**Power**
- DEWALT 20V MAX Lithium-Ion Battery 2-Pack, 5.0 Ah — 5s li-ion: ~21.0 V full, ~18 V nominal (https://www.amazon.com/DEWALT-Lithium-Ion-Battery-Charger-DCB205-2c/dp/B0CZ9XR2Z7)
- Power Wheel Adapter (Dewalt 20V converter kit) — bare terminals, **no BMS/low-voltage cutoff**; RoboClaw Min Main Battery is the only pack protection (https://www.amazon.com/dp/B0CDGR4Y8K)
- Tobsun 24V→5V 10A buck powers the Pi 5 over USB-C (non-isolated — Pi ground is tied to battery negative) (https://www.amazon.com/dp/B01M03288J)
- Dewalt terminal split: motor driver + buck converter. *Note: USB-C without PD negotiation caps Pi peripheral current to 600 mA; current limit manually disabled*

## RoboClaw Configuration (verified on board after save + power cycle)

**General / Serial**
- Control Mode: Packet Serial. Address 128 (0x80). Baud 115200 (used — packets go over the GPIO UART)
- Timeout **0.2 s** = deadman. Motors stop if no valid packet for 200 ms. Pi must stream commands >5 Hz (use 20–50 Hz). **Do not set 0 to "fix" dropouts**
- Swap Encoder Channels / Multi-Unit / USB-TTL Relay: all off. RC/Analog panel: inert in Packet Serial

**Battery**
- Battery Cutoff: Use User Settings. Autodetect off
- Max Main: set 22.0 → regen clamp ~4.4 V/cell. **Displayed value ratchets down through readback scaling (21.9 → 21.7 → 21.4 across sessions) — periodically re-enter 22.0.** If MBH warnings appear on a full pack, it drifted low
- Min Main: 16.0 (3.2 V/cell floor). Max/Min Logic: inert (no logic battery)

**Motors (both channels)**
- Max Current 30.0 / Max Regen −30.0. Motors physically can't exceed ~10 A/channel, so limits never engage in normal operation — they only catch wiring faults, and give headroom over current-sense error (see below). 12 A / −10 A limits were the original autotune failures
- Idle: Free Wheeling, 1.0 s delay. Default Speed 100%, Default Accel/Decel 200%/200%

**Encoders / sign convention**
- Both Quadrature. **Encoder 1 Invert: checked. Encoder 2: unchecked. Reverse boxes: unchecked**
- Verified in duty (PWM) mode: +duty = robot-forward rotation = counts increase, both channels. Re-verify in duty mode after touching any Reverse/Invert box — a wrong encoder sign in closed loop = instant full-speed runaway

**I/O**
- S3/S4/S5 disabled. (S3 can become a hardware e-stop later)

## Velocity PID (autotuned at 20.2–20.3 V full pack, saved to NVM)

| | Motor1 | Motor2 |
|---|---|---|
| P | 5.76070 | 5.96193 |
| I | 0.33235 | 0.35071 |
| D | 0 | 0 |
| QPPS | 9240 | 9240 |
| Error Limit | 0 (disabled) | 0 (disabled) |

- Autotune varies ±5% run to run — normal, don't chase decimals. Channels within a few % of each other = healthy
- QPPS scales with battery voltage: 7920 @ 18.1 V, 9240 @ 20.3 V, datasheet-consistent. **Cap commanded speed ≈ 7,000 counts/s in Pi code** so the loop never saturates as the pack sags toward 16 V
- **Measured ground top speed ≈ 1.30 m/s (8510–8521 counts/s, ~92% of QPPS) at a full 20.4 V pack** (155 mm wheels; commanded 1.6 m/s, so it saturated below the command). No-load QPPS ceiling 9240 = 1.41 m/s; the ~8% gap is on-ground load. Expect the ceiling to fall to ~1.0 m/s as the pack sags to 16 V — a `max_linear_velocity` above ~1.0 will clip late in discharge
- **Measured rotation (in-place spin, ~20.1 V pack):** true yaw ≈ **0.71 × commanded** (skid-steer scrub — physically 2.75 rev vs 3.89 odom rev over 12 s @ 2.0 rad/s cmd, so **odom over-reports yaw ~41%**). Spinning is high-load: wheels saturate at only ~5600–6740 counts/s spinning (vs 8510 straight-line), giving a max true spin ≈ **4.6 rad/s** (~1.4 s/rev). Config `max_angular_velocity: 4.0` → ~2.8 rad/s true. **Driver does NOT normalize the wheel pair**, so at full `max_linear_velocity` (1.2 m/s / 7885 counts/s) the outer wheel is near its ceiling and hard turns-while-driving-fast clip (in-place pivots unaffected). The 0.71 scrub factor is a starting point for `wheel_separation` calibration
- Position PID: untouched, all zeros — not used. Only tune (cascaded position autotune) if sub-crawl-speed motion is ever needed

## Operating limits & known behaviors

- Breakaway ≈ 7% duty (off-ground, both channels)
- Velocity-loop floor ≈ 300–500 counts/s: at the ~300 Hz control loop the encoder delivers ~1 count/tick, so tracking below this is quantization-limited. Enforce a minimum-speed floor in Pi code; use position cascade if smooth crawl is ever required
- **Current telemetry is unreliable below ~15–20% duty.** Verified from CSV: phantom smooth readings to ±30 A during breakaway at 5–12% duty, frozen fake plateaus (4.71 A) during no-load cruise — while battery voltage never moved. BasicMicro blanks current limiting at low duty for this reason. Judge health by speed tracking, temperature, and battery sag, not the current channel at low duty. Readings above ~20% duty / a few amps are meaningful
- Original symptom decoded: "low-speed stall + current spike" = velocity loop running unsaved/default gains with 12 A/−10 A limits clamping the thrash (motor humming = armature limit-cycling through gear backlash). Fixed by real gains + 30 A limits

## NVM save ritual (learned the hard way)

Motion Studio edits live in **RAM** and vanish on power cycle. After any change:
1. Device → Save Settings
2. Power off / on
3. Re-open **both** General Settings and Velocity Settings tabs and verify (PID is not visible in the General tab)

## Pi-side control notes

- Device is the Pi 5 GPIO UART `/dev/ttyAMA0` (requires `dtparam=uart0=on` in config.txt; NOT `/dev/ttyAMA10`, which is the debug/console UART)
- `sudo usermod -aG dialout $USER`; purge `modemmanager` if present
- Deployed controller is the **`roboclaw_driver` ROS 2 node** (wimblerobotics/Sigyn, built from the `kahleeeb3` fork in the Dockerfile), run via docker-compose; params in `scout/config/roboclaw.yaml`. Address 0x80, 115200 baud. (`scout/scout/motor_test.py` is a standalone raw-UART bench test only — no ROS, open-loop duty.)
- The node converts `/cmd_vel` → per-wheel QPPS and sends **`MIXEDSPEEDACCELDIST` (SpeedAccelDistance)** each time a *new* `cmd_vel` arrives. So the **`cmd_vel` publisher** must stream ≥5 Hz (20–50 Hz) to satisfy the 0.2 s deadman, and must publish an explicit **zero** Twist to stop — otherwise the deadman just Free-Wheels to a coast (idle mode is Free Wheeling; it does not brake). It is NOT `SpeedAccelM1M2`; there is no send-always in the driver (it only re-sends on a new cmd_vel sequence)
- **`accel` doubles as a top-speed limiter — do NOT use "a few thousand" here.** Each command carries a distance cap = `commanded_counts/s × max_seconds_uncommanded_travel (T)` and must decelerate to a stop within it, so reachable speed ≈ `sqrt(2·accel·V·T)`. To actually reach commanded V you need **`accel ≥ V/(2·T)`** (e.g. 1.2 m/s ≈ 7885 counts/s at T=0.2 s needs accel ≥ ~19,700; current config is 20000). accel 3000 pinned the robot at ~0.3–0.4 m/s. Lowering T tightens stop/overrun distance but raises the accel floor
- The C++ driver already wraps serial I/O in try/except with `resyncSerial()` — transient CRC errors (a few are normal at startup) are retried, not fatal; dropouts coast via the deadman and the driver reconnects and resumes

## LED strip (APA102) Pi-side notes

- Driver is **`scout/scout/led_test.py`** — standalone, `spidev` + stdlib only, no ROS. Runs via docker-compose under the `led` profile: `docker compose --profile led run --rm led_control`. `python3-spidev` is installed in the Dockerfile; `privileged: true` exposes `/dev` so the container sees the SPI device
- **Enable SPI first:** uncomment `dtparam=spi=on` in `/boot/firmware/config.txt` and **reboot**. Same pattern as the UART `dtparam=uart0=on`
- **The header SPI0 is `/dev/spidev0.0` (bus 0).** On the Pi 5 it lives on the RP1 (`/axi/pcie…/rp1/spi@50000`, alias `spi0`) and only appears once `dtparam=spi=on` is set
- **`/dev/spidev10.0` is a trap — do NOT use it.** It is the BCM2712 SoC's internal SPI (`spi@7d004000`, alias `spi10`), present even when header SPI is disabled, and **not wired to GPIO10/11**. Opening it and calling `xfer2` succeeds silently (exit 0, no error) while nothing ever reaches pins 19/23 — a dark strip with a "passing" program. This burned an entire debugging session: the real fix was enabling `dtparam=spi=on`, not chasing a bus number
- **Verify the mux, don't trust the device node:** after reboot `pinctrl get 10,11` must read **`a0`** (ALT0 = SPI). If it reads `none`, the pins are plain GPIO and SPI0 is not actually enabled, regardless of what `/dev/spidev*` exists
- APA102 frame format (in `show()`): start = 4×`0x00`; per-LED = `0xE0 | brightness(0–31)`, then **BLUE, GREEN, RED**; end = `ceil(NUM_LEDS/16)` bytes of `0xFF`