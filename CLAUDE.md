# Skid-Steer Robot

## ⚠ Never command motion without explicit confirmation

**Ask the operator every single time before running anything that moves the robot, and say what it will do — direction, speed, duration, and the space it needs.** Standing permission is never implied: a yes for one run does not carry to the next, and it does not carry across a change of surface or location. The operator has to know where the robot is and that the area is clear, and only they can know that.

**Cancelling the agent's shell command does NOT stop the robot.** Learned the hard way: interrupting `docker compose run` kills the *view* of the container, not the container, which keeps streaming `cmd_vel` and keeps driving. The 0.2 s deadman only helps once `cmd_vel` actually stops, so an orphaned publisher defeats it entirely. To really stop it:

```bash
docker ps --filter name=<service>          # find the generated run-container name
docker stop <container>                    # ends cmd_vel; deadman free-wheels within 200 ms
```

Note this is a *coast*, not a brake — idle mode is Free Wheeling. There is no hardware e-stop yet; S3 is still free for one (see **I/O**), and this is the argument for wiring it.

## ⚠ OPEN FAULT: front-left wheel does not always drive (found 2026-07-30)

**Observed:** during an in-place pivot the **front-left wheel was not spinning CCW, but all four spun CW.** This invalidates pivot/scrub calibration until fixed — a wheel that is dragged sideways instead of rolling adds enormous resistance, so the chassis rotates far less than the wheels imply.

**It is quantitatively consistent with the measured asymmetry** (hard floor, 2026-07-30): scrub 0.51 CCW vs 0.77 CW, reproducible across 1.0 / 1.5 / 3.0 rad/s, with rotation surging and stalling rather than holding rate (peak yaw 48 deg/s CCW vs 57 CW against a 57.3 deg/s command). Visual count confirmed the gyro (~1.0 rev CCW vs ~1.5 CW), so **the instrument is fine — it was faithfully reporting a robot with a dragging wheel**. Probably also the original "walking / bouncing and shaking during pivots" symptom that started the whole tuning effort.

**The driver cannot see this.** The two motors per side are **paralleled on one channel and only the rear encoders are wired**, so the velocity loop closes on the rear wheel while the front one is unmonitored. Paralleled DC motors share terminal voltage, not speed — one can stall while the other spins normally, and neither the PID nor odometry will notice.

**It is fixed to the robot, not the floor.** With inflated tires, **CW is the better direction on carpet *and* hard floor** (carpet 0.638/0.627/0.693 CW vs 0.486/0.518/0.614 CCW at 1.0/1.5/3.0 rad/s). It only looked surface-dependent because the one run that favoured CCW was the pre-inflation carpet session — which is also when the robot was handled. Operator also reports the **other front wheel does it too, less severely**, which argues against a single broken component.

**The speed trend identifies the mechanism: a fixed drag torque, NOT hub slip.** The CCW/CW split *narrows* as speed rises (27% → 19% → 12% on carpet; same trend on hard floor). A slipping set screw would do the opposite — more torque at speed means it gives up sooner. A constant drag that the motor overcomes once it has enough voltage produces exactly this: the front wheel stalls at low duty and starts turning at higher duty. So check, in this order:
1. **High-resistance connection in the front-left motor circuit** (screw terminals at the RoboClaw and at the motor). Series resistance starves that motor worst at low duty — same speed-dependent stall signature
2. **Excess drag in the front-left drivetrain** — a rub, tight bearing, or gearbox drag
3. **Hub / set screw slip** — de-prioritised by the speed trend above, but a hand check for play between wheel and shaft is free
4. **Load discrimination test:** with the wheels off the ground it should turn freely both directions if this is drag or current starvation; if it still refuses unloaded, it is electrical or gearbox

**Re-confirmed 2026-07-30 by the odometry EKF**, on hard floor at 1.5 rad/s: scrub 0.476 CCW vs 0.727 CW, i.e. the fault is unchanged and still direction-specific. That reading came from fused-vs-wheel yaw with none of the original tooling involved, so it is an independent instrument agreeing with the old one (0.51 / 0.77). See "Odometry EKF".

**Stable and reproducible, and NOT electrical.** Two carpet runs 35 min apart, with a motor screw-terminal check in between, agreed to 1% (mean scrub 0.596 then 0.601) with the CCW deficit unchanged. A marginal connection would have shifted when disturbed. Deprioritise wiring; the fault is mechanical or inherent to the motor.

**Best guess at the real scrub, from the CW (all-wheels-driving) direction: ~0.72 on carpet** — CW settles at 0.713 / 0.720 at 1.5 / 3.0 rad/s, close to the 0.71 originally estimated by eye, and ~0.73 on hard floor. CW at 1.0 rad/s still sags to 0.64, consistent with the operator's report that the *other* front wheel misbehaves mildly: at the lowest duty both front wheels are marginal, and only the worse one keeps failing as duty rises. Do not apply these: a one-direction value is a guess until both directions agree after repair.

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
- Wiring (into the strip's **DI/CI input end** — arrows point away from it). The strip has 6 wires at that end; use them as a **star ground split** (power return and signal reference separated at the strip):

  ```
  Green (DATA)   → GPIO10 / SPI0 MOSI (pin 19)   short pigtail
  Yellow (CLOCK) → GPIO11 / SPI0 SCLK (pin 23)   short pigtail
  Red + Red      → Buck Out +                    (both, for current capacity)
  Black #1       → Buck Out −                    (power return)
  Black #2       → Pi GPIO GND                    (signal reference, bundled with DATA/CLOCK into the header)
  ```

  - **Keep the DATA/CLOCK pigtails as short as possible** — long unshielded jumpers pick up EMI (see EMI note below).
  - **Black #2 is a dedicated signal-ground reference**, not a power return: it carries ~no current and keeps the strip's logic ground tied tightly to the Pi's ground that the SPI logic thresholds against. Route it *bundled with* the green/yellow signal pigtails.
  - Power +/− jumper length is not critical (low-impedance rails); it's the signal lines and the signal-ground reference whose length/routing matter
- Powered from its own 5V/10A buck (NOT the GPIO header). Shares ground with the Pi through the non-isolated buck (Pi and LED share only the buck **output** ground; motor return flows Battery± → driver → Battery± on the input side, so motor current does not flow through the strip's ground path)
- **3.3V GPIO drives it directly — no level shifter needed** (verified; the strip also ran off a 3.3V ESP32). Confirmed working at 1 MHz SPI
- Current budget: 131 LEDs × ~60 mA ≈ **7.9 A at full white**. Fine at the default low brightness (8/31), but do not `set_all(255,255,255)` at brightness 31 on a buck shared with the Pi — it can brown out and reset the Pi
- **Watch:** in the star-ground scheme above, black #1 now carries *all* the power return. Check it doesn't overheat at high brightness — power-inject at the far end or beef up the return if pushing high current

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
- **Measured ground top speed ≈ 1.30 m/s (8510–8521 counts/s, ~92% of QPPS) at a full 20.4 V pack** (155 mm wheels; commanded 1.6 m/s, so it saturated below the command). No-load QPPS ceiling 9240 = 1.41 m/s; the ~8% gap is on-ground load. Expect the ceiling to fall to ~1.0 m/s as the pack sags to 16 V — a `max_linear_velocity` above ~1.0 will clip late in discharge. Config is now **1.0** (see "Straight-line duty" below for the full duty-vs-speed curve)
- **Measured rotation (in-place spin, ~20.1 V pack):** true yaw ≈ **0.71 × commanded** (skid-steer scrub — physically 2.75 rev vs 3.89 odom rev over 12 s @ 2.0 rad/s cmd, so **odom over-reports yaw ~41%**). Spinning is high-load: wheels saturate at only ~5600–6740 counts/s spinning (vs 8510 straight-line), giving a max true spin ≈ **4.6 rad/s** (~1.4 s/rev) — but those saturation figures were measured **on carpet** and are surface-specific (see "Pivot duty ceiling & surface dependence" below). Config `max_angular_velocity: 3.0` → ~2.1 rad/s true (~3.0 s/rev). **Driver does NOT normalize the wheel pair**, so a turn is driven only by the duty the linear speed leaves over — at `max_linear_velocity: 1.0` that is ~1.2 rad/s of true yaw, and it collapses to ~0.24 rad/s if the limit is raised to 1.2 (in-place pivots unaffected). The 0.71 scrub factor is a starting point for `wheel_separation` calibration
- Position PID: untouched, all zeros — not used. Only tune (cascaded position autotune) if sub-crawl-speed motion is ever needed

## Operating limits & known behaviors

- Breakaway ≈ 7% duty (off-ground, both channels)
- Velocity-loop floor ≈ 300–500 counts/s: at the ~300 Hz control loop the encoder delivers ~1 count/tick, so tracking below this is quantization-limited. Enforce a minimum-speed floor in Pi code; use position cascade if smooth crawl is ever required
- **Current telemetry is unreliable below ~15–20% duty.** Verified from CSV: phantom smooth readings to ±30 A during breakaway at 5–12% duty, frozen fake plateaus (4.71 A) during no-load cruise — while battery voltage never moved. BasicMicro blanks current limiting at low duty for this reason. Judge health by speed tracking, temperature, and battery sag, not the current channel at low duty. Readings above ~20% duty / a few amps are meaningful
- Original symptom decoded: "low-speed stall + current spike" = velocity loop running unsaved/default gains with 12 A/−10 A limits clamping the thrash (motor humming = armature limit-cycling through gear backlash). Fixed by real gains + 30 A limits

## Pivot duty ceiling & surface dependence

**What limits in-place rotation is duty, not the gains — and duty demand is dominated by the floor.** Measured by sweeping commanded yaw both directions and reading applied duty back with **GETPWMS (cmd 48)**. Compare surfaces using **motor volts = duty × pack voltage**, not raw duty: duty alone shifts as the pack drains, motor volts don't.

Forward-side motor volts needed to hold a commanded in-place spin (155 mm wheels, ~19.2–19.4 V pack):

| Commanded | Off ground | Hard floor | Carpet |
|---|---|---|---|
| 2.5 rad/s | — | 10.0 V | 13.4 V |
| 3.0 rad/s | — | 11.1 V | 14.8 V |
| 4.0 rad/s | **8.9 V** (45% duty) | 13.4 V (0% saturated) | **17.3 V (pinned at 100% duty 20% of the run)** |

- **Carpet adds 2.2–3.8 V of demand over a hard floor.** On a hard floor the pivot is clean at every speed up to 4.0 rad/s — 99.8–100.2% speed tracking, peak duty 92%, zero saturation, ~0 cm of walk. On carpet at 4.0 rad/s the forward-driving side hits 100% duty, falls to 96% of target with 15% of samples dipping (worst 82%), and the robot **walks ~2.5 cm per revolution**
- **Failure mode:** the forward side saturates → cannot go faster → falls behind while the reverse side holds → the wheel-pair difference becomes both a yaw error and a translation, so the robot lurches and creeps *backward* through the pivot. Dips last ~100–450 ms and recur every 0.5–2.5 s. It reads as "won't rotate in place, arcs across the floor"
- **The forward-driving side always costs more than the reverse-driving side**, and the deficit swaps channels with spin direction — it is NOT a weak channel. On carpet the gap is 1.9–3.2 V and CW costs more than CCW; **on a hard floor the gap collapses to 0.5–1.4 V and the two directions equalize** (7.6 vs 7.8 V @ 1.5, 11.1 vs 11.1 @ 3.0). So the direction asymmetry is the **carpet's nap**, not the robot
- **Gains are not the cause.** Off the ground both channels hold 100.0% of target with 0.1–0.2% ripple, zero dips, and encoder totals matching to 5 counts in 46,780 (0.01%). Anything that looks like a pivot tuning problem should be checked against an off-ground run first — it takes 30 s and eliminates the entire drivetrain
- **Current telemetry is useless here too, not just at low duty** (extends the warning above): it sat at 0.84–0.96 A while duty ranged 45%→100%, which is physically impossible. Judge saturation by the duty readback only
- The `max_angular_velocity: 4.0` figure and the "wheels saturate ~5600–6740 counts/s spinning" note above were taken on carpet and are **surface-specific**. Config is now **3.0** as a compromise (carpet just off the stop on a fresh/mid pack, clipping late in discharge); 2.5 for clean carpet pivots at any battery state; 4.0 is fine on hard floors
- Secondary effect: with the distance cap active, refreshing `cmd_vel` at 10 Hz instead of 30 Hz made carpet dips worse (15.7% → 24.2% of samples, walk 2.3 → 3.0 cm/rev). The cap starts braking before the next command lands, stealing margin from a channel that has none. Publish at 20–50 Hz

## Straight-line duty & why `max_linear_velocity` is 1.0

**Straight-line demand is almost purely back-EMF: duty rises linearly at ~15 V of motor volts per m/s, with a negligible load offset.** Measured on a hard floor at a 19.1 V pack, driving forward then reverse at each speed (wheels commanded together rather than opposed):

| Commanded | Achieved | Duty | Motor volts | Speed tracking | Ripple | Veer |
|---|---|---|---|---|---|---|
| 0.3 m/s | 0.300 | 25% | 4.8 V | 100% | 0.4% | ≤0.3 deg/m |
| 0.6 m/s | 0.600 | 49% | 9.4 V | 100% | 0.3% | ≤0.2 deg/m |
| 0.9 m/s | 0.899 | 72% | 13.9 V | 100% | 0.3% | ≤0.2 deg/m |
| 1.0 m/s | 1.000 | 80% | 15.3 V | 100% | 0.3% | ≤0.2 deg/m |
| 1.2 m/s | 1.179 | **96% (peaks pinned at 100%)** | 18.3 V | **98%** | **1.0%** | **up to 1.5 deg/m** |

- **Reachable top speed ≈ (pack V − 0.3) / 15, derated ~3%.** That model independently reproduces the 1.30 m/s measured at a full 20.4 V pack, and predicts 1.22 m/s at 19.1 V, 1.15 at 18 V nominal, and **1.02 at the 16 V cutoff**. So 1.2 m/s only exists in the top third of the discharge; below that the robot silently drives slower than commanded, and **odometry inherits the error**
- **The deciding factor was turning authority, not straight-line speed.** Because the driver does not normalize the wheel pair, whatever duty the linear speed leaves over is all the outer wheel has for a turn. At 1.0 m/s the spare 20% is worth ~1.2 rad/s of true yaw at full speed; at 1.2 m/s the spare 4% is worth ~0.24 rad/s, so turns taken at top speed come out wider than commanded
- **Unlike pivots, the command cadence is not a limiter here.** Uncapped 30 Hz, capped 30 Hz, and capped 10 Hz all reached 1.18 m/s with zero dips, so `accel: 20000` clears the distance-cap threshold for straight-line motion (needs ≥ ~16,400 at 1.0 m/s). Contrast this with carpet pivots, where 10 Hz measurably hurt
- **The wheels are extremely well matched: imbalance ≤1% at every speed**, so straight-line veer is not a drivetrain asymmetry. Veer stays under 0.2 deg/m below saturation, and the residual **flips sign between forward and reverse** — the signature of the chassis tracking slightly crooked, not of one side running fast. Only at a saturated 1.2 m/s does it grow to 1.2–1.5 deg/m, because a channel already at 100% duty has nothing left to correct with
- The 1.2 m/s row is the same saturation signature as the carpet pivots, just milder: duty pinned, tracking short of target, ripple up 3×. **Ripple tripling while duty approaches 100% is the tell for "this speed is past the hardware ceiling," not "the gains are wrong."**

## Current tuning state & next step (`wheel_separation`)

Tuned so far: velocity PID (NVM), `max_angular_velocity: 3.0`, `max_linear_velocity: 1.0`, `accel: 20000`. **`wheel_separation` is still the raw geometric track (0.290) and has NOT been calibrated.**

- **Tire pressure is a precondition for every geometric calibration.** It sets the **loaded rolling radius** (hence `wheel_radius` and every counts↔metres conversion), and a soft tire has a **larger, longer contact patch** — which is exactly what pivot scrub fights, so underinflation inflates the measured scrub. Unequal left/right pressure is also a real wheel-diameter mismatch, i.e. straight-line veer. **Tires were inflated 2026-07-30 (firm by feel, pressure NOT gauged — a reproducibility gap).** To close it, record a gauge reading *and* the **axle-centre height to the floor under load**, which is a durable geometric marker that needs no gauge to re-check
- **`wheel_radius: 0.0775` is VERIFIED post-inflation on hard floor — do not change it.** A 2 m commanded drive measured **81 in (2.0574 m) by tape vs 2.0464 m reported**, i.e. +0.54%, which implies 0.0779 but is *inside* the ±0.5 in tape precision (which spans 0.0774–0.0784). Fitting to that would be fitting to noise. The real finding is that effective rolling radius ≈ the 155 mm nominal, so **the tires are barely deflecting** — independent confirmation they are properly inflated, since soft tires would read measurably short. Method (the tool has since been deleted): drive out and back under `/cmd_vel`, and compare the tape against **net displacement**, which is what a tape between two marks actually measures — not path length, which is longer whenever the robot veers
- **Straight-line quality after inflation is excellent:** forward vs reverse displacement agreed to **0.02%** (2.0464 / 2.0459 m) and heading changed **≤0.1° over 2 m** — the veer seen pre-inflation is gone, so equal inflation evidently fixed the side-to-side match
- **One number fixes both error paths.** Commanding uses `Δv = ω × W`, odometry uses `ω = Δv / W`, so a single constant sets both. Skid-steer scrub makes the chassis rotate less than the wheels imply, which means the robot under-turns *and* odom over-reports by the same factor. Both are corrected by `W_eff = W_config / scrub`, where `scrub = yaw_true / yaw_odom`. Calibrate UP from the geometric track, never down
- **Raising W re-opens the pivot duty problem:** the same `ω` command then demands proportionally more wheel speed, pushing carpet pivots back toward the saturation documented above. Derate `max_angular_velocity` by the same factor in the same edit (`new_max = old_max × scrub`) — the physical motion stays identical and only the number becomes honest. `max_linear_velocity` is unaffected (W does not enter the linear path)

**⚠ ALL SCRUB NUMBERS BELOW ARE VOID — measured against the front-left wheel fault (see top of file), and the earlier set was also pre-inflation.** Kept only as the method's shakedown and for the qualitative patterns. Re-measure both surfaces **after the wheel is repaired**. What did survive the exercise: the measurement chain itself is validated (gyro confirmed by visual count, integration math verified exact against synthetic data in both directions), and the *mean* of the two directions was strikingly stable at 0.64 across five independent runs — but that mean is the average of a good direction and a faulty one, so it is not the robot's real scrub factor either. Gyro yaw vs the driver's own `/odom` pose yaw, both directions, integrated through the deceleration so the two streams cover the same motion:

| Commanded | CCW scrub | CW scrub | True yaw rate |
|---|---|---|---|
| 1.0 rad/s | 0.517 | 0.470 | 0.52 rad/s |
| 1.5 rad/s | 0.616 | 0.524 | 0.94 rad/s |
| 3.0 rad/s | 0.647 | 0.622 | 1.98 rad/s |

- **The old 0.71 figure was optimistic and is superseded** — it came from one eyeballed revolution count. Odom over-reports yaw by ~60%, not 41% (at least on soft tires). Mean scrub 0.60 would imply `wheel_separation ≈ 0.48` and `max_angular_velocity ≈ 1.8`, but see the pressure blocker
- **Scrub is not a constant ratio, so no single W is right at all speeds.** True yaw rate against command (0.52 / 0.94 / 1.98 for 1.0 / 1.5 / 3.0) fits a roughly *fixed angular deficit* far better than a fixed fraction, which is the signature of stiction in the pivot: the loss matters relatively more the slower you turn. Spread across runs was 20%. Expect this to shrink once the contact patch is right
- **The CCW/CW split tracks the carpet nap**, and by the same sign as the duty sweeps: 16% at 1.5 rad/s but only 4% at 3.0, with CW the more expensive direction. More slip means more wheel rotation per degree of chassis rotation, so a harder direction yields a *lower* scrub factor. Physically consistent, which is a useful check that the measurement is sane
- **The odometry side is self-validating:** 1.5 rad/s for 12 s should be 2.87 rev and `/odom` reported 2.91, the excess being deceleration. So the driver tracks its command and the unwrapping is right; any discrepancy is real scrub, not a measurement artefact
- **A single W cannot be correct on both carpet and hard floor** — scrub is strongly surface-dependent (see "Pivot duty ceiling"), so any chosen value is a compromise that is wrong somewhere. The permanent fix is fusing gyro yaw, and **that is now built and running** — see "Odometry EKF" below. The live `/wheel_odom` → `/odom` yaw ratio *is* the scrub factor on whatever floor you are on, which also means the calibrator described above no longer needs building: the EKF is the instrument

**D455 IMU as a yaw reference (VERIFIED READING on hardware — see measured numbers below):** BMI055 or BMI085 depending on part number (K83122-100 vs -110/111; indistinguishable without librealsense), ±1000 deg/s at 200/400 Hz, 50 µs timestamp accuracy — integrate on timestamps, not assumed dt (rate tolerance ±0.3%). Bias (~±1 deg/s) is the dominant term and is removable by averaging a few seconds of stillness before each run; the residual ~1–2% sensitivity error is irrelevant against a 41% effect. `rs-imu-calibration` corrects gyro **bias only, not scale** (accel gets bias + scale + misalignment). Gyros are untrustworthy for *unbounded* heading, not for bounded few-second measurements — that distinction is the whole argument. **There is no zero-install path** — the Pi kernel ships no `hid-sensor-*` modules, so the camera's HID interface stays on generic `usbhid` and no IIO device appears.

**librealsense must be built from source with the RSUSB backend — the apt debs do not work here.** `ros-humble-librealsense2` / `ros-humble-realsense2-camera` exist as arm64 debs (and `python3-pyrealsense2` does not), but the deb's **native backend cannot read the D455 IMU on this Pi** — it wants the kernel HID-sensor path that doesn't exist (see above). Only `FORCE_RSUSB_BACKEND=ON`, which reads the IMU over raw libusb instead, gets gyro data. So the Dockerfile builds **v2.57.7 from source** (`realsenseai/librealsense`). Do not "simplify" this back to an apt install — it will build and run and silently produce no IMU.

**The `cmake` invocation in that RUN step is known-good — do not change it.** Its flags (including `CMAKE_INSTALL_LIBDIR=lib/aarch64-linux-gnu`) are deliberate and verified. Build failures in this step have so far all been *missing apt packages*, not wrong flags, so fix them in the System Dependencies layer:
- **`libudev-dev` is required.** With RSUSB the SDK compiles a bundled libusb whose udev hotplug backend needs `libudev.h`; without it the build dies in `third-party/libusb/.../linux_udev.c`. The visible error is a generic `make: *** Error 2` ~70 lines later and a Dockerfile banner pointing at the whole RUN step, so **read *up* the log to the first `fatal error:`** instead of suspecting the cmake line
- **`python3-dev`** for `BUILD_PYTHON_BINDINGS=ON` (`ros:humble-ros-core` has no `Python.h`)
- Also present for this step: `libusb-1.0-0-dev`, `pkg-config`
- **Install paths are all correct as-is, verified in the running container:** `/opt/ros/humble/lib/aarch64-linux-gnu` **is** on `LD_LIBRARY_PATH` (ROS's `local_setup` adds the triplet dir, not just `lib`), and `pyrealsense2` lands in `/usr/lib/python3/dist-packages/` which is on the default `sys.path`. `import pyrealsense2` works with no `PYTHONPATH` help. Do not "fix" `CMAKE_INSTALL_LIBDIR` or add `PYTHON_INSTALL_DIR`
- Runtime: `privileged: true` **is sufficient** — it bind-mounts the host `/dev`, so `/dev/bus/usb/*` is already visible with no explicit device mount (same reason the LED node reaches `/dev/spidev0.0`)

**Measured gyro performance (D455 chassis-mounted, 2026-07-29, RSUSB source build):** enumerates as `Intel RealSense D455` FW 5.17.0.10 on USB 3.2, Motion Module offering Accel@100/200/400 and **Gyro@200/400 Hz**. Delivered 199 Hz clean at a 200 Hz request (8965 samples / 45 s).

| Quantity | Measured | Verdict |
|---|---|---|
| Stationary bias | x −0.254, y −0.209, z −0.128 deg/s | 4–8× *better* than the ±1 deg/s assumed above |
| Noise (sd) | 0.11–0.15 deg/s | ~0.1% of a 150 deg/s pivot |
| Scale error | **−0.6% over two hand turns** (−715.7 deg measured vs 720 true; individual turns −359.5 and −356.2) | Irrelevant vs the 41% scrub effect — the whole argument holds |
| Drift after bias removal | **0.003 deg/s** (0.05 deg over 19 s stationary) ≈ 10 deg/hr | vs −12.5 deg/**min** uncorrected: bias removal buys ~80× |

- **Yaw is gyro `y`, and `ROS yaw rate = −gyro_y`.** Gravity reads on accel `y` (−9.64 m/s², so `|g|` is 1.7% low — accel scale error, ignore it), meaning the camera's **Y axis points down**. By the right-hand rule about a downward axis, CCW-from-above (ROS/REP-103 positive yaw) is **negative** `gyro_y`. Confirm the sign once against a known-direction turn before trusting odometry comparisons
- A 12 s pivot measurement inherits only ~0.04 deg of drift error, so the "gyros are fine for *bounded* measurements" argument is now measured, not assumed

**Two pyrealsense2 API traps (both cost a confusing measurement):**
- **The first ~13 frames arrive in `timestamp_domain.hardware_clock`, then the stream switches to `global_time`** (epoch ms) mid-flight. The dt across that switch is ~1.785e12 ms and will detonate any naive integrator. **Filter on `f.get_frame_timestamp_domain() == rs.timestamp_domain.global_time`** — dropping a fixed number of leading frames is not reliable, the switch happened after frame 13 in one run and frame 2 in another
- **With gyro *and* accel both enabled, `wait_for_frames()` returns each gyro frame ~twice** (388 framesets/s for a 193 Hz stream, measured 50.2% duplication), because a frameset carries the latest sample of every stream. Dedupe on `get_frame_number()`, use per-sensor callbacks, or enable gyro alone (gyro-only measured 0% duplication)

**The live ROS IMU stack is now built and verified end to end (2026-07-29).** `realsense2_camera` 4.57.7 is in the image, version-matched to librealsense v2.57.7 (wrapper `4.X.Y` ↔ lib `2.X.Y` — bump both together). Verified on hardware:
- Motion Module **starts** (no `HID Motion Sensor Failure`), gyro and accel both open at 200 FPS, and `/camera/camera/imu` publishes at **200.07 Hz**. The launch prints `For the 'unite_imu_method' param update to take effect, re-enable either gyro or accel stream` — **that warning is benign**, the united topic works anyway. If it ever does not, `/camera/camera/gyro/sample` is already a `sensor_msgs/Imu` and can be used directly
- Neither pyrealsense2 trap above applies through the wrapper: it handles the timestamp domain switch and the duplicate-frame issue itself
- **`gyro_calibrator` end-to-end drift: −0.07 deg/min** on the corrected `/imu/data` (−0.034 deg over 30 s), against −12.1 deg/min raw — a 170× improvement, so a 12 s pivot inherits ~0.014 deg. It now also re-estimates bias whenever the robot is stationary (MEMS bias moves as the camera warms, so a value taken once at cold boot goes stale), publishes nothing during the startup window, and fills `angular_velocity_covariance` at 7.0e-6 (the measured noise variance)
- **Gyro scale confirmed in situ:** it reported 1.67 rev where a counted spin was ~1.7. Combined with the −0.6% two-turn measurement above, the gyro is trustworthy for bounded measurements — which is what the whole approach rests on
- `/odom` from the driver runs at **30 Hz**, not the 67 Hz in the config; ample for yaw unwrapping (0.1 rad between samples at 3 rad/s, against a π limit)

**Docker: one shared overlay for source-built packages, and a volume-shadowing trap.** `/ros_ws/install` is a **named volume**, and a named volume only seeds from the image while it is still empty. Anything the Dockerfile installs there after the volume exists is **silently invisible**, and later image rebuilds can never reach it — the build succeeds, the files are in the layer, and the package simply is not there at runtime. Proven: the image's own `/ros_ws/install/setup.bash` was dated Jul 29 while the mounted volume's was Jul 27.
- So image-baked source packages go to a **single shared overlay**, `$OVERLAY=/opt/overlay`. `colcon` merges into an existing install base without orphaning what is already there (verified), so each package keeps its own cached `RUN` layer. Adding the Nth package is one line: `git clone … "$OVERLAY/src/<name>" && build-overlay --packages-up-to <pkg>`. The `build-overlay` helper sources ROS plus the overlay itself, so a later package can depend on an earlier one. The entrypoint loops over the overlay list, so **it never needs editing again**
- **Consequence to fix: the running `roboclaw_driver` is the stale Jul 27 copy from the volume**, which shadows the fresh one in the overlay (`ros2 pkg prefix roboclaw_driver` → `/ros_ws/install/...`). Bumping its git pin and rebuilding has *no runtime effect*. One-time surgical fix, which keeps the `scout` build: `docker compose run --rm --entrypoint bash build_package -c 'rm -rf /ros_ws/install/roboclaw_driver'`
- **Put new apt packages *after* the librealsense `RUN`.** Adding them to the earlier layer invalidates its cache and costs a full librealsense rebuild — 13 min of the 13.5 min total on a Pi 5, versus 1.5 min for the wrapper alone

**⚠ All bench and calibration tooling was DELETED on request (2026-07-30), along with the whole compose `test` profile.** Every measurement in this file was produced by tools that no longer exist, so treat the numbers as the record and expect to rebuild the instrument before extending them. Removed: `spin_diagnostic.py` (in-place spin *and* straight-line sweeps with duty/current/encoder/voltage logging), `tune_velocity_pid.py` (RoboClaw packet-serial client + PID autotune), `wheel_separation_calibrator.py`, `wheel_radius_calibrator.py`, `motor_test.py` (raw-UART open-loop duty + breakaway ramp), `led_test.py`. Only `gyro_calibrator.py` was kept, being a runtime node rather than a test. Three of these were untracked, so there is no git history to restore from.

**The same applies to `pivot_check.py`, written and deleted on request 2026-07-30** after producing the scrub table in "Odometry EKF". The EKF has made this trivial to rewrite, so the design is worth keeping rather than the file: pivot in place at a fixed rate in both directions, and diff the **unwrapped pose yaw** of `/wheel_odom` against fused `/odom` over the same window — their ratio is the scrub factor, with none of our own integration in the comparison and no gyro handling at all. Take the pose, not the reported rate. Refuse to move unless `/odom`, `/wheel_odom` and `/imu/data` are all live (**subscribe to `/imu/data` with sensor QoS or it will read as silent**), publish `cmd_vel` at 20–50 Hz, and always send an explicit zero Twist at the end because the deadman only coasts. No `wheel_separation` parameter is needed any more — the answer is a ratio of two live streams — but converting it to an implied `wheel_separation` still requires the value the driver is running.

## Odometry EKF — gyro yaw + encoder distance (built 2026-07-30)

`robot_localization`'s `ekf_node` now fuses the two sensors along their good axes: **forward speed from the wheels, yaw rate from the gyro, and nothing else from either.** `ros-humble-robot-localization` 3.5.4 is an apt install in the Dockerfile (in the post-librealsense layer, so it does not cost a 13 min rebuild). Config is `scout/config/ekf.yaml`, compose service `ekf`.

**Topic and TF ownership moved:**
- The driver's raw estimate is remapped to **`/wheel_odom`**, and **`/odom` is now the fused output**. So anything reading `/odom` gets the good yaw for free, and nothing downstream needed changing
- The `roboclaw_driver` compose service **no longer uses `roboclaw_driver.launch.py`** — it runs `ros2 run roboclaw_driver roboclaw_driver_node` directly, because the launch file cannot remap and the node hardcodes the topic name `odom`. `--params-file` behaves identically to the launch file's `parameters=`, since the node names itself `roboclaw_driver`, matching the key in `roboclaw.yaml`
- **`publish_tf` in `roboclaw.yaml` is now `false`** — the EKF owns `odom→base_link`. Verified only two `/tf` publishers remain: `robot_state_publisher` and `ekf_filter_node`
- The EKF depends on the **`camera` and `robot_description` services being up**, because it rotates the IMU into `base_link` through TF (the IMU is stamped `camera_imu_optical_frame`). No TF, no yaw

**⚠ THE TRAP: `imu0_config` names the SENSOR's axes, not the robot's — and "vyaw alone" silently fuses nothing here.** robot_localization builds a diagonal mask from those 15 flags, rotates it by the sensor→`base_link` transform, and whichever rows survive decide which state variables actually get updated. On a body-aligned IMU sensor-Z *is* the yaw axis, so the distinction is invisible and every tutorial's "set vyaw true" works. This IMU reports in an **optical** frame with **Z out of the lens**, so a lone `vyaw` flag asks for rotation about the robot's *forward* axis; the mask rotates onto `vroll`, and `two_d_mode` then forces that to zero.
- **Failure signature: absolutely no warning.** Not in the node log (0 bytes), not in `/diagnostics` (which reported "functioning properly"), and the IMU showed a healthy subscription count. Every rejection path that reports itself was untouched, because nothing was *rejected* — the measurement was faithfully applied to the wrong axis
- **How to tell fused from ignored, in one number:** watch `twist.covariance[35]` (vyaw variance) in `/odom`. Fused it sits at **6.6e-6**, essentially the gyro's own 7.0e-6 measurement variance. Ignored it **climbs without bound** (2.5 and rising when broken). Same trick for the wheels via `twist.covariance[0]`: 1.1e-2 against the driver's advertised 0.1. A stationary robot cannot show you this any other way — both sensors read ~zero, so the estimate looks perfect while being unfused
- **Fix: mark all three rotational rates true.** It costs nothing (`two_d_mode` overwrites vroll/vpitch with zero at 1e-6 covariance) and it is mount-agnostic, so it stays correct if the camera is ever remounted
- **The sign needs no fix, and this is now VERIFIED UNDER REAL ROTATION** (hard floor, 1.5 rad/s, 8 s each way): `+ω` raised the fused yaw and `−ω` lowered it, agreeing with the wheels in both directions. The measurement is rotated by TF, and the URDF's camera mounting already encodes `ROS yaw rate = −gyro_y` — visible in the transform, whose `base_link ← camera_imu_optical_frame` basis rows are `(0,0,1) (−1,0,0) (0,−1,0)`, so the yaw row dotted with the gyro vector is exactly `−gyro_y`
- **A QoS mismatch will silently starve any new IMU consumer.** `gyro_calibrator` publishes `/imu/data` best-effort (`qos_profile_sensor_data`); a default *reliable* subscription receives **nothing** and only says so in a one-line `incompatible QoS` warning at discovery. The EKF gets this right on its own; hand-written tools must ask for sensor QoS explicitly

**Measured:** `/odom` publishes at 30.0 Hz with much tighter jitter than the driver's (±0.2 ms vs ±1.8 ms — it runs on the filter's own clock, not the serial poll). **Stationary yaw drift +0.070 deg/min over 30 s**, which is exactly `gyro_calibrator`'s own end-to-end figure, so the EKF inherits the calibrated gyro's performance and adds nothing. x/y hold at 0.00000 m stationary, and `two_d_mode` pins z.

**The EKF is now the scrub instrument, and its first reading independently reproduces the wheel fault.** Comparing `/wheel_odom` yaw against fused `/odom` yaw over the same pivot gives the live scrub factor directly, with no external tooling and no integration of our own — which retires the calibrator design sketched under "Current tuning state" as something to rebuild. First reading (**hard floor, 1.5 rad/s commanded, 8 s per direction, 2026-07-30**):

| Direction | Wheel odom says | Truth (fused) | Scrub |
|---|---|---|---|
| CCW | 1.95 rev (+701 deg) | 0.93 rev (+334 deg) | **0.476** |
| CW | 1.95 rev (−704 deg) | 1.42 rev (−511 deg) | **0.727** |

- So the **encoders over-report yaw by 110% in the bad direction and 38% in the good one** — worse than the ~60% previously believed, because that figure averaged the two.
- **This matches the earlier gyro-tooling numbers (hard floor 0.51 CCW / 0.77 CW) to within a few percent, from a completely different measurement path.** Two independent instruments agreeing is strong evidence that the asymmetry is the robot, and that **the front-left wheel fault is still present and unchanged**. Also note true yaw of only 0.73 rad/s CCW against 1.11 CW for the same 1.5 rad/s command.
- **Still void as calibration inputs** — one good direction and one faulty one cannot average into a real scrub factor. Re-read both after the wheel is repaired; agreement between directions is the signal that it is fixed.

**What this does NOT fix: the command path.** The EKF corrects what the robot *reports*, not what it *does*. Commanding still goes through `Δv = ω × W` in the driver with the uncalibrated geometric `wheel_separation: 0.278`, so a commanded `ω` still produces less yaw than asked. `wheel_separation` therefore still wants calibrating for command fidelity — but it is no longer load-bearing for odometry, which removes the pressure and the surface-dependence problem (one W cannot suit carpet and hard floor; the gyro suits both). Note also that raising W to fix commanding re-opens the pivot duty ceiling, and that **all scrub numbers remain void until the front-left wheel fault is repaired**.

**Deliberately not fused:** the wheel pose (`x`/`y`/`yaw` were integrated by the driver using its own bad yaw, so fusing them reintroduces the error), the wheel `vyaw` (the scrub-corrupted number this exists to replace), wheel `vy` (hardcoded 0 with zero covariance, and lateral slip in a skid-steer turn is real), IMU orientation (none exists — `gyro_calibrator` now advertises that per the `sensor_msgs/Imu` spec with `orientation_covariance[0] = -1`, so a future config that fuses yaw gets a complaint instead of silently pegging heading to zero), and IMU linear acceleration (worthless integrated for position, and the wheels already measure speed well).

**Untuned:** `process_noise_covariance` and `initial_estimate_covariance` are upstream defaults. One consequence worth knowing: the reported **yaw variance grows to ~1.7 rad² in 30 s**, wildly pessimistic against a gyro drifting 0.07 deg/min, because the default yaw process noise is 0.06 rad²/s and there is no absolute yaw reference to pull it back. It does not affect the yaw *estimate* — with no yaw measurement anywhere in the filter, process noise only inflates the covariance, it cannot move the mean — but any future consumer that gates on pose uncertainty will need this tuned first.

## LiDAR — streaming `/scan` (built 2026-07-30)

**⚠ The attached scanner is NOT the "RPLIDAR C1" in NOTES.md's parts list.** Interrogated on the bench it identifies as an **A2-family (triangulation)** unit: **256000 baud**, 16.0 m max range, firmware **1.32**, hardware rev **6**, S/N `9A8FECF0C3E09ED4A0EA98F309574116`. A C1 is 460800 baud and 12 m and has none of the modes below. This is not cosmetic — at 460800 the lidar never answers at all.

Driver is `rplidar_ros` **built from source** into `$OVERLAY` (SDK 2.1.0), config `scout/config/rplidar.yaml`, compose service `lidar`. Connected over a **CP2102 USB-UART bridge on `/dev/ttyUSB0`**; `usb_max_current_enable=1` is already set in `/boot/firmware/config.txt` for the motor.

**Verified running:** `/scan` at **11.7 Hz** (motor runs slightly above the commanded 10 Hz), **1800 beams at 0.20°** over the full 360°, **96% valid returns**, range limits 0.15–16.0 m.

**This unit's scan modes, all at 16.0 m** — divide the point rate by ten for points per revolution at 10 Hz:

| Mode | Points/s | Per rev | Resolution |
|---|---|---|---|
| Standard | 4.0K | ~400 | 0.90° |
| Express | 7.9K | ~790 | 0.45° |
| Stability | 10.0K | ~1000 | 0.36° |
| Boost | 15.9K | ~1590 | 0.23° |
| **Sensitivity** (configured) | 15.9K | ~1590 | 0.23° |

Sensitivity is both the highest point rate and what the lidar reports as its *typical* mode, and it reads low-reflectivity surfaces (dark furniture, black baseboards) better than Boost at the same rate. Drop to **Stability** if bright ambient light causes dropouts. **Asking for an unsupported mode name is a safe way to make the node print the whole table** — that is how the one above was obtained.

**⚠ `/dev/serial/by-id/...` DOES NOT EXIST INSIDE THE CONTAINER, and the failure is deeply misleading.** Those symlinks are created by **udev on the host**; a privileged container gets its own `/dev` without them, so the "more robust" by-id path resolves to nothing. This corrects the note under **LED Strip** that privileged "bind-mounts the host `/dev`" — real device nodes like `/dev/ttyUSB0` and `/dev/bus/usb/*` are there, but udev's symlink farms (`/dev/serial`, `/dev/disk/by-*`) are not.
- **The SDK does not report the missing path as a bind failure.** It opens nothing, then dies in `getDeviceInfo` with `Error, unexpected error, code: 80008004` (`RESULT_OPERATION_NOT_SUPPORT`), which reads like "this lidar model is unsupported" and sends you hunting the wrong problem entirely. `connect()` returns success; the node's own "cannot bind to the specified serial port" message never fires
- Consequence: the config must use `/dev/ttyUSB0`, whose number is assigned in probe order. Unambiguous today (only USB serial device; the RoboClaw is on the GPIO UART), but if a second is added, confirm identity **from the host** with `ls -l /dev/serial/by-id/` against the S/N above

**Sweep the baud before suspecting hardware.** A wrong baud gives `SL_RESULT_OPERATION_TIMEOUT`, which is indistinguishable from a dead or unpowered lidar. 115200 / 460800 / 1000000 all time out on this unit; only 256000 answers. The sweep costs 25 s and settles it — loop `ros2 run rplidar_ros rplidar_node --ros-args -p serial_baudrate:=$b` and watch for the S/N line.

**`laser` is a new REP-103 frame in the URDF, and the driver is pointed at it — not at the exporter's `lidar1_link`/`lidar2_link`.** Those are CAD-style (Z forward, Y up), so using one would tip every range reading 90° out of the floor plane; same problem and same fix as `camera_link`. The joint hangs off `lidar1_link` (the rotating head, the closest thing the CAD offers to the beam origin) with `rpy="0 -1.5707963267949 -1.5707963267949"`.
- **It is self-checking: the rotation works out to exactly zero against `base_link`.** Verified — `base_link→laser` reads translation `(0.073, 0.000, 0.241)` and RPY `0 0 0`, matching the predicted `(0.0725, 0, 0.2406)`. So `tf2_echo base_link laser` showing any nonzero rotation means the chain is broken

**⚠ THE SCANNER IS MOUNTED BACKWARDS — 180° of yaw, corrected in the URDF (measured 2026-07-30).** Nothing in the CAD records how the unit is bolted to the mast, and the error announces itself nowhere: the scan looks perfectly plausible, it is just rotated, which would silently build a wrong map.

Measured by parking the robot in a deliberately asymmetric spot and comparing the scan against the operator's description of the room:

| Feature | Reported | Actually |
|---|---|---|
| 8.5 m open sight line | 180° | **in front** |
| 0.35 m obstacle arc | −85°…−7° (right) | **on the left** |
| 2.1 m opening | +90° (left) | on the right |

- **Fitting `true = s × reported + offset` needs both features, and that is the point of using two.** A single feature cannot separate a rotation from a mirror: `s = +1, offset = 180°` puts the tight arc on the left, while `s = −1, offset = 180°` puts it on the right. The operator's "left" picks the first. So the scan is **NOT mirrored** — the driver correctly stays `inverted: false` — and it is a pure yaw
- **Fixed in the `lidar1_to_laser` joint, NOT in the driver**, because it is a statement about how the hardware is bolted on. Composing 180° of yaw onto the CAD correction flips both signs, `0 -pi/2 -pi/2` → `0 pi/2 pi/2`, and `base_link→laser` now reads RPY `0 0 ±pi` instead of `0 0 0`
- **Confirmed end to end through TF**, transforming the scan into `base_link` the way SLAM and Nav2 will: the open direction moved to 0° (8.70 m, longest sight lines at ±2.5°), the tight arc to +89°…+164° at 0.26–0.39 m, and the 2.03 m opening to −90°
- **Reusable method**, better than the single-object test it replaces: dump a top-down ASCII map plus the bearings of the closest returns and the longest sight lines, have the operator name where two *different* features really are, then solve for `s` and `offset`. It needs no props and no robot motion

## NVM save ritual (learned the hard way)

Motion Studio edits live in **RAM** and vanish on power cycle. After any change:
1. Device → Save Settings
2. Power off / on
3. Re-open **both** General Settings and Velocity Settings tabs and verify (PID is not visible in the General tab)

## Pi-side control notes

- Device is the Pi 5 GPIO UART `/dev/ttyAMA0` (requires `dtparam=uart0=on` in config.txt; NOT `/dev/ttyAMA10`, which is the debug/console UART)
- `sudo usermod -aG dialout $USER`; purge `modemmanager` if present
- Deployed controller is the **`roboclaw_driver` ROS 2 node** (wimblerobotics/Sigyn, built from the `kahleeeb3` fork in the Dockerfile), run via docker-compose — `ros2 run`, not its launch file, so its odometry can be remapped to `/wheel_odom` for the EKF (see "Odometry EKF"); params in `scout/config/roboclaw.yaml`. Address 0x80, 115200 baud. There is no longer any standalone raw-UART bench path — `motor_test.py` was deleted, so open-loop duty commands now need a new tool.
- The node converts `/cmd_vel` → per-wheel QPPS and sends **`MIXEDSPEEDACCELDIST` (SpeedAccelDistance)** each time a *new* `cmd_vel` arrives. So the **`cmd_vel` publisher** must stream ≥5 Hz (20–50 Hz) to satisfy the 0.2 s deadman, and must publish an explicit **zero** Twist to stop — otherwise the deadman just Free-Wheels to a coast (idle mode is Free Wheeling; it does not brake). It is NOT `SpeedAccelM1M2`; there is no send-always in the driver (it only re-sends on a new cmd_vel sequence)
- **`accel` doubles as a top-speed limiter — do NOT use "a few thousand" here.** Each command carries a distance cap = `commanded_counts/s × max_seconds_uncommanded_travel (T)` and must decelerate to a stop within it, so reachable speed ≈ `sqrt(2·accel·V·T)`. To actually reach commanded V you need **`accel ≥ V/(2·T)`** (e.g. 1.2 m/s ≈ 7885 counts/s at T=0.2 s needs accel ≥ ~19,700; current config is 20000). accel 3000 pinned the robot at ~0.3–0.4 m/s. Lowering T tightens stop/overrun distance but raises the accel floor
- The C++ driver already wraps serial I/O in try/except with `resyncSerial()` — transient CRC errors (a few are normal at startup) are retried, not fatal; dropouts coast via the deadman and the driver reconnects and resumes

## LED strip (APA102) Pi-side notes

- Driver is the **`led_node`** ROS node (`scout/scout/led_node.py` + `apa102.py`), run by the `led` compose service. The old standalone `led_test.py` was deleted. `python3-spidev` is installed in the Dockerfile; `privileged: true` exposes `/dev` so the container sees the SPI device
- **Enable SPI first:** uncomment `dtparam=spi=on` in `/boot/firmware/config.txt` and **reboot**. Same pattern as the UART `dtparam=uart0=on`
- **The header SPI0 is `/dev/spidev0.0` (bus 0).** On the Pi 5 it lives on the RP1 (`/axi/pcie…/rp1/spi@50000`, alias `spi0`) and only appears once `dtparam=spi=on` is set
- **`/dev/spidev10.0` is a trap — do NOT use it.** It is the BCM2712 SoC's internal SPI (`spi@7d004000`, alias `spi10`), present even when header SPI is disabled, and **not wired to GPIO10/11**. Opening it and calling `xfer2` succeeds silently (exit 0, no error) while nothing ever reaches pins 19/23 — a dark strip with a "passing" program. This burned an entire debugging session: the real fix was enabling `dtparam=spi=on`, not chasing a bus number
- **Verify the mux, don't trust the device node:** after reboot `pinctrl get 10,11` must read **`a0`** (ALT0 = SPI). If it reads `none`, the pins are plain GPIO and SPI0 is not actually enabled, regardless of what `/dev/spidev*` exists
- APA102 frame format (in `show()`): start = 4×`0x00`; per-LED = `0xE0 | brightness(0–31)`, then **BLUE, GREEN, RED**; end = `ceil(NUM_LEDS/16)` bytes of `0xFF`

## LED strip EMI (resolved — motor noise → flicker/wrong colors)

**Symptom:** the APA102 strip flickered and showed wrong colors *only while the motors were driving* — worst on the left side driving forward — plus glitches on motor start/stop.

**Root cause — two mechanisms:**
1. **Capacitive coupling** onto the DATA/CLOCK lines from the RoboClaw's high-`dV/dt` H-bridge switching. It's **capacitive (E-field / displacement current), not magnetic** — confirmed because twisting the motor leads did *nothing* (that only cancels magnetic/inductive `dI/dt` coupling) while a grounded foil shield *did* help (only blocks E-fields). Mechanism: Maxwell displacement current across the parasitic capacitance between the motor leads and the signal wires, driven by the drive's `dV/dt`
2. **Ground-reference bounce** on motor start/stop — the strip's logic ground momentarily shifting relative to the Pi's during the current transient, moving the reference the SPI logic thresholds against

**Fix — target the victim, not the source:**
- **Shortened the DATA/CLOCK jumper pigtails** to the bare minimum to reach the header. The long unshielded jumpers were acting as pickup antennas; shortening them cleared the gross flicker
- **Dedicated signal-ground reference:** used the strip's two black wires as a star-ground split — black #1 → Buck Out − (power return), **black #2 → Pi GPIO GND, bundled with the green/yellow signal pigtails into the header** (near-zero-current logic reference that keeps the strip's ground tied tightly to the Pi's). This cleared the start/stop residual. See wiring diagram under **LED Strip** above

**Principles (why it worked):**
- The coupling was **capacitive** (`dV/dt` displacement current), so the cure is shorter/shielded signal runs, not lead twisting
- **Split signal ground from power ground at the strip (star ground):** power-return current flows through black #1 to the buck; black #2 carries ~no current, so motor/LED power-return current stays *out* of the reference the SPI logic compares against
- **Signal-line and signal-ground length/routing are what matter**; power +/− jumper length is not critical (low-impedance rails)
- It is **not** made worse by the shared buck ground: motor return flows Battery± → driver → Battery± on the input side; the Pi and strip share only the buck *output* ground, so motor current never flows through the strip's ground path directly