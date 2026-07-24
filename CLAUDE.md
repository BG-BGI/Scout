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
- Library: BasicMicro `roboclaw_3.py` (needs pyserial). Address 0x80
- Stream `SpeedAccelM1M2(0x80, accel, v_left, v_right)` every cycle at 20–50 Hz (send-always, not send-on-change — the 0.2 s deadman requires it). Use a real accel value (a few thousand counts/s²), never instant sign flips
- Wrap serial I/O in try/except and reopen on failure — dropouts coast to a stop via the deadman, code should reconnect and resume