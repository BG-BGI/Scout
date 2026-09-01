# Skid-Steer Robot

## ⚠ The operator is a robotics and ROS 2 (Humble) expert. Be terse.

**Assume he already knows it.** Most exchanges are questions — answer them and stop. Sometimes he delegates a goal he can't do himself; do that and report the result.

Findings only. No teaching, no restating the mechanism, no recapping what was just done, no "worth noting", no closing summary. Use ROS/controls terms bare — `map→odom`, QPPS, deadman, tf buffer — and do not explain them.

Report a result in a sentence or two, or bullets. If a number is the finding, give the number. Long explanations do not get read, so anything important buried in one is lost — length actively costs information.

Exceptions, kept short anyway: a **root cause** that changes what to do next, and the **operator instructions** below.

**The job is measurement, not advice.** Design the experiment, collect the data, analyse it, report the number. When asked what to do next, **name one test and say why in one line** — do not lay out options, alternatives, risk trade-offs, or a menu to choose from. The operator will ask for alternatives if he wants them. Say what to measure and how to measure it well; he decides whether to run it.

## ⚠ When the operator has to do something, say so plainly at the end, then stop

Anything that needs human hands or eyes — clicking a goal in Foxglove, clearing the floor, turning the battery on, plugging something in, checking whether all four wheels turn — goes at the **end** of the message as plain numbered instructions, and then **wait for approval before doing anything else.** No burying the ask mid-paragraph, no starting the next step "while you do that", no assuming the answer.

The reason is not politeness. Most of the asks are physical, and the operator is the only one who can see the robot; acting before they confirm means acting on a guess about the real world.

## ⚠ Never command motion without explicit confirmation

**Ask the operator before every single run that moves the robot, and say what it will do — direction, speed, duration, space needed.** Permission never carries over: not to the next run, and not across a change of surface or location. Only the operator knows where the robot is and whether the area is clear.

**Cancelling the agent's shell command does NOT stop the robot.** Interrupting `docker compose run` kills the *view* of the container, not the container — it keeps streaming `cmd_vel` and keeps driving. To actually stop it:

```bash
docker ps --filter name=<service>   # find the generated run-container name
docker stop <container>             # ends cmd_vel; deadman free-wheels within 200 ms
```

That is a **coast, not a brake** (idle mode is Free Wheeling). There is no hardware e-stop; S3 is still free for one.

## Tire state: ALL FOUR DELIBERATELY DEFLATED (2026-08-14, operator's traction fix)

Soft tires are the operating condition, not a fault. The old flat-front-left story (2026-07-30) and its 2.5 rad/s stall floor are RETIRED — measurements below supersede everything from the inflated era.

- **wheel_radius is 0.0780** (re-verified by 2 m out-and-back tape test: wheels under-report +0.70% deflated; robot returned to the mark within 0.5 in). Re-run the tape test after any pressure change.
- **Pivots scrub enormously but nothing stalls.** wheel/gyro yaw ratio at 1.5 rad/s: **1.93 CCW / 1.60 CW** (all four wheels visibly turning, at different rates — normal for voltage-sharing paralleled fronts). A ~20% direction asymmetry persists; it costs nothing because yaw is gyro-fused.
- **Pivot walk is the real cost, and speed is the mitigation: ~10 cm/rev at 1.5 rad/s vs ~2.5 cm/rev at 2.5 rad/s.** So 2.5 stays the *recommended* pivot rate where position matters (tight spaces, scout-skills `rotate` default, tight-tunnel profile) — as a walk minimizer, not a stall floor. The hard clamps were removed from joystick_teleop and trick_player.
- **Straight-line behavior is unaffected**: out-and-back legs matched to 0.3%, net drift 1.3 cm over 4 m.
- **The paralleled-front blindness still applies**: only rear encoders are wired, fronts share voltage not speed — watch all four wheels during any new pivot diagnosis.
- **"Front wheel screams, rear looks stopped" during pivots is NOT a fault (measured 2026-08-14).** During CCW pivots (the high-scrub direction) a front wheel breaks traction and freewheels fast at the shared channel voltage while the rear tracks its commanded ~0.6 rev/s — dramatic contrast that reads as a stalled rear. Instrumented: rear encoders never dropped below normal speed, channel currents 1.6–1.8 A mean / 2.7 A max (stall would be ~5 A, open rear circuit ~0.3 A). Electrically healthy; the costs are front tire wear and the 1.93/1.60 scrub asymmetry. Mitigation if ever needed: firmer front tires than rears.
- **Duty/motor-volt tables elsewhere in this file predate deflation** (straight-line ~15 V/m/s, pivot duty ceilings, carpet numbers). Soft tires drag more; re-measure before leaning on those numbers for margin calculations.

## Hardware

**Compute** — Raspberry Pi 5 16 GB (https://www.adafruit.com/product/6125)

**Motors** — Four Pololu #4693 37D gearmotors, 50:1, 24 V, quadrature encoders (https://www.pololu.com/product/4693)
- 64 CPR motor shaft → 3200 counts/wheel rev. 200 RPM / 100 mA no-load, 3 A stall @ 24 V → ~8 Ω/motor, ~4 Ω/paralleled channel
- Per channel: ~0.2 A no-load, ~5 A hard-stall, ~10 A worst-case plugging transient. **Nothing sustained above ~5 A/channel can be real**

**Motor controller** — RoboClaw 2x30A, firmware 4.4.9 (https://www.basicmicro.com/RoboClaw-2x30A-Motor-Controller_p_9.html)
- Drives all four motors, two paralleled per channel. Screw Terminal Adapter (V1) on the I/O header
- Only the two rear encoders wired (battery weight sits at the rear); front encoders unconnected
- Controlled over the **GPIO UART** (packet serial, GPIO14/15 → `/dev/ttyAMA0`). USB not used

**LiDAR** — see the LiDAR section: an **A2-family** unit (not the C1 in NOTES.md), CP2102 USB-UART on `/dev/ttyUSB0`

**Camera** — Intel RealSense D455, FW 5.17.0.10, USB 3.2. IMU for yaw; color for live view; depth→decimated XYZ for under-lidar local costmap marking

**LED strip** — APA102, 131 LEDs, **SPI-driven (separate DATA + CLOCK)**. NOT a WS2812/NeoPixel, so `rpi_ws281x` and bit-banged timing libraries do not apply.

Wiring at the strip's **DI/CI input end** (arrows point away from it), using its 6 wires as a **star ground split**:

```
Green (DATA)   → GPIO10 / SPI0 MOSI (pin 19)   short pigtail
Yellow (CLOCK) → GPIO11 / SPI0 SCLK (pin 23)   short pigtail
Red + Red      → Buck Out +                    (both, for current capacity)
Black #1       → Buck Out −                    (power return)
Black #2       → Pi GPIO GND                   (signal reference, bundled with DATA/CLOCK)
```

- **Keep DATA/CLOCK pigtails as short as possible** — long unshielded jumpers pick up motor EMI (see LED EMI section)
- **Black #2 is a signal-ground reference, not a power return.** It carries ~no current and keeps the strip's logic ground tied to the ground the Pi's SPI logic thresholds against. Route it bundled with the signal pigtails. Power +/− jumper length is not critical
- Own 5 V/10 A buck, **not** the GPIO header. Non-isolated, so Pi and strip share the buck *output* ground; motor return flows Battery± → driver → Battery± on the input side and never through the strip's ground path
- **3.3 V GPIO drives it directly, no level shifter** (verified). Confirmed at 1 MHz SPI
- Budget ~7.9 A at full white (131 × 60 mA). Fine at the default brightness 8/31, but `set_all(255,255,255)` at brightness 31 can brown out the shared buck and reset the Pi
- **Watch:** black #1 now carries all the power return in the star scheme. Check it does not overheat at high brightness

**Power**
- DEWALT 20V MAX 5.0 Ah packs — 5s li-ion, ~21.0 V full, ~18 V nominal (https://www.amazon.com/DEWALT-Lithium-Ion-Battery-Charger-DCB205-2c/dp/B0CZ9XR2Z7)
- Power Wheel Adapter, bare terminals, **no BMS or low-voltage cutoff** — RoboClaw Min Main Battery is the only pack protection (https://www.amazon.com/dp/B0CDGR4Y8K)
- Tobsun 24V→5V 10A buck powers the Pi over USB-C, non-isolated (Pi ground tied to battery negative) (https://www.amazon.com/dp/B01M03288J)
- Dewalt terminal splits to motor driver + buck. USB-C without PD caps Pi peripheral current at 600 mA; the limit is manually disabled

## RoboClaw configuration (verified on board after save + power cycle)

**Serial** — Packet Serial, address 128 (0x80), 115200 baud over the GPIO UART. Swap Encoder Channels / Multi-Unit / USB-TTL Relay all off; the RC/Analog panel is inert.

**Timeout 0.2 s = deadman.** Motors stop if no valid packet for 200 ms, so the Pi must stream >5 Hz (use 20–50 Hz). **Do not set 0 to "fix" dropouts.**

**Battery** — Cutoff: Use User Settings, autodetect off. Min Main 16.0 (3.2 V/cell). Max/Min Logic inert.
- Max Main 22.0 (regen clamp ~4.4 V/cell). **The displayed value ratchets down through readback scaling** (21.9 → 21.7 → 21.4 across sessions) — periodically re-enter 22.0. MBH warnings on a full pack mean it drifted low

**Motors (both channels)** — Max Current 30.0 / Max Regen −30.0. Motors physically cannot exceed ~10 A/channel, so these never engage normally; they only catch wiring faults and give headroom over current-sense error. The original 12 A / −10 A limits were what made autotune fail. Idle Free Wheeling, 1.0 s delay. Default Speed 100%, Accel/Decel 200%.

**Encoders** — Both Quadrature. **Encoder 1 Invert checked, Encoder 2 unchecked, Reverse boxes unchecked.** Verified in duty mode: +duty = forward = counts increase on both channels. **Re-verify in duty mode after touching any Invert/Reverse box** — wrong encoder sign in closed loop is instant full-speed runaway.

**I/O** — S3/S4/S5 disabled.

**NVM save ritual.** Motion Studio edits live in RAM and vanish on power cycle: Device → Save Settings, power cycle, then re-open **both** General and Velocity Settings tabs to verify (PID is not visible in the General tab).

## Velocity PID (autotuned at 20.2–20.3 V, saved to NVM)

| | Motor1 | Motor2 |
|---|---|---|
| P | 5.76070 | 5.96193 |
| I | 0.33235 | 0.35071 |
| D | 0 | 0 |
| QPPS | 9240 | 9240 |
| Error Limit | 0 (disabled) | 0 (disabled) |

- Autotune varies ±5% run to run — don't chase decimals. Channels within a few % of each other is healthy
- QPPS scales with pack voltage: 7920 @ 18.1 V, 9240 @ 20.3 V. **Cap commanded speed ≈ 7000 counts/s in Pi code** so the loop never saturates as the pack sags toward 16 V
- Position PID is untouched (all zeros) and unused. Only tune the cascaded position autotune if sub-crawl motion is ever needed

## Operating limits

- Breakaway ≈ 7% duty (off-ground, both channels)
- **Velocity-loop floor ≈ 300–500 counts/s** (~0.046–0.077 m/s): at the ~300 Hz control loop the encoder delivers ~1 count/tick, so tracking below this is quantization-limited. Enforce a minimum-speed floor in Pi code
- **⚠ Current telemetry is unreliable below ~15–20% duty and useless during pivots.** Verified: phantom ±30 A readings during 5–12% duty breakaway, frozen 4.71 A plateaus at no-load cruise, and 0.84–0.96 A while duty swept 45%→100% — all while battery voltage never moved. BasicMicro blanks current limiting at low duty. **Judge health by speed tracking, temperature, and battery sag; judge saturation by the duty readback only.** Readings above ~20% duty are meaningful
- Historical: "low-speed stall + current spike" was the velocity loop running default gains with the 12 A/−10 A limits clamping the thrash. Fixed by real gains + 30 A limits

## Straight-line performance (hard floor, 19.1 V pack)

Duty demand is almost purely back-EMF: **~15 V of motor volts per m/s**, negligible load offset.

| Commanded | Achieved | Duty | Motor volts | Tracking | Ripple | Veer |
|---|---|---|---|---|---|---|
| 0.3 m/s | 0.300 | 25% | 4.8 V | 100% | 0.4% | ≤0.3 °/m |
| 0.6 m/s | 0.600 | 49% | 9.4 V | 100% | 0.3% | ≤0.2 °/m |
| 0.9 m/s | 0.899 | 72% | 13.9 V | 100% | 0.3% | ≤0.2 °/m |
| 1.0 m/s | 1.000 | 80% | 15.3 V | 100% | 0.3% | ≤0.2 °/m |
| 1.2 m/s | 1.179 | **96%, peaks pinned at 100%** | 18.3 V | **98%** | **1.0%** | **1.5 °/m** |

- **Reachable top speed ≈ (pack V − 0.3) / 15, derated ~3%.** Reproduces the 1.30 m/s measured at a full 20.4 V pack; predicts 1.15 m/s at 18 V nominal and **1.02 at the 16 V cutoff**. So `max_linear_velocity: 1.0` — above that the robot silently drives slower than commanded late in discharge and odometry inherits the error
- **The deciding factor was turning authority, not speed.** The driver does not normalize the wheel pair, so whatever duty the linear speed leaves over is all the outer wheel has for a turn: 20% spare at 1.0 m/s ≈ 1.2 rad/s of yaw, 4% spare at 1.2 m/s ≈ 0.24 rad/s
- **Wheels are matched to ≤1% at every speed**, so veer is not a drivetrain asymmetry — the residual flips sign between forward and reverse, which is the chassis tracking slightly crooked
- **Ripple tripling as duty approaches 100% is the tell for "past the hardware ceiling," not "gains are wrong."**
- Command cadence is not a limiter here: uncapped 30 Hz, capped 30 Hz and capped 10 Hz all reached 1.18 m/s with zero dips

## Pivot performance & surface dependence

**What limits in-place rotation is duty, not gains.** Read applied duty back with **GETPWMS (cmd 48)** and compare surfaces using **motor volts = duty × pack voltage**, not raw duty (duty shifts as the pack drains, motor volts don't).

Forward-side motor volts to hold a commanded in-place spin (155 mm wheels, ~19.2–19.4 V pack):

| Commanded | Off ground | Hard floor | Carpet |
|---|---|---|---|
| 2.5 rad/s | — | 10.0 V | 13.4 V |
| 3.0 rad/s | — | 11.1 V | 14.8 V |
| 4.0 rad/s | **8.9 V** (45% duty) | 13.4 V (no saturation) | **17.3 V, pinned at 100% for 20% of the run** |

- **Carpet adds 2.2–3.8 V over a hard floor.** Hard floor is clean to 4.0 rad/s (99.8–100.2% tracking, peak duty 92%, ~0 cm walk). Carpet at 4.0 rad/s saturates: 96% of target, 15% of samples dipping (worst 82%), robot **walks ~2.5 cm per revolution**
- **Failure mode:** forward side saturates → falls behind while the reverse side holds → the wheel-pair difference becomes both yaw error and translation, so the robot lurches and creeps backward. Reads as "won't rotate in place, arcs across the floor"
- **The forward-driving side always costs more than the reverse-driving side, and the deficit swaps channels with spin direction** — it is not a weak channel. Carpet gap 1.9–3.2 V; on hard floor it collapses to 0.5–1.4 V and directions equalize. The old conclusion that this asymmetry is carpet nap is **half right at best** — it predates the flat tire, which is a real robot-side asymmetry on every surface
- **Gains are not the cause.** Off the ground both channels hold 100.0% of target with 0.1–0.2% ripple and encoder totals matching to 5 counts in 46,780. **Check any suspected pivot tuning problem against a 30 s off-ground run first** — it eliminates the whole drivetrain
- `max_angular_velocity` is **3.0** as a compromise. Use 2.5 for clean carpet pivots at any battery state; 4.0 is fine on hard floors. The older 4.0 figure and the "wheels saturate at 5600–6740 counts/s" note were taken on carpet and are surface-specific
- Refreshing `cmd_vel` at 10 Hz instead of 30 Hz made carpet dips measurably worse (15.7% → 24.2% of samples). The distance cap starts braking before the next command lands. **Publish at 20–50 Hz**

## Geometry (from `scout_description.urdf`, not calibration)

Tuned so far: velocity PID (NVM), `max_angular_velocity: 3.0`, `max_linear_velocity: 1.0`, `accel: 20000`.

- **`wheel_radius: 0.0780` — re-verified 2026-08-14 on DEFLATED tires** (tape 2.0066 m vs wheel odom 1.9896/1.9958 per leg → +0.70%; the inflated-era 0.0775 measured +0.54% the same way). Tires barely deflect even soft. Only exercises the **rear** pair. Method: drive out and back, compare tape against **net displacement**, not path length; re-run after any pressure change
- **`wheel_separation: 0.278` is the URDF geometric track and is deliberately NOT calibrated.** A skid-steer pivot scrubs by definition, so the chassis always rotates less than the wheels imply and **encoder-only yaw is untrustworthy at any value** — which is why yaw is fused from the IMU. Also: only the rear encoders are wired, so front-wheel scrub is invisible to the very sensors that would measure it; and scrub depends on speed and surface, so no single constant is correct. Calibrating it would buy **command fidelity** (a commanded ω under-produces yaw) but nothing for odometry
- **Straight-line quality after inflation is excellent:** forward vs reverse displacement agreed to 0.02% and heading changed ≤0.1° over 2 m
- **⚠ Two traps when reading wheel positions out of the URDF.** The wheel joints are children of `chassis_link`, and `base_link_to_chassis` carries a **90° yaw**, so chassis-frame axes are swapped relative to the robot's — the lateral offset is the `0.111` component, not `0.089606`. And a joint origin sits at the **inboard hub face**, not the tire centreline: `wheel_link.STL` spans `0 … 0.0562` along its rotation axis at radius 0.079. Skip either correction and the geometry is wrong by tens of percent
- **Chassis envelope, from mesh bounds: 0.337 m long × 0.334 m wide** (longitudinal half-extent `0.0896 + 0.079`, lateral to the tire's outer face `0.111 + 0.0562`), circumscribed radius 0.238, inscribed 0.167. **The wheels are the widest and longest parts** — the largest body mesh, `spacer_link.STL`, is ±0.1295 × ±0.1105, and the lidar mast ±0.1075. Re-check if anything is bolted on outboard of the wheels. The same numbers confirm the 0.278 track: tire centreline at `0.111 + 0.0562/2 = 0.1391`. **Centreline is the right quantity for kinematics, outer face for collision extents** — mixing them up gives 0.22

## D455 IMU — the yaw reference

BMI055 or BMI085 (K83122-100 vs -110/111, indistinguishable without librealsense), ±1000 deg/s at 200/400 Hz, 50 µs timestamp accuracy — **integrate on timestamps, not assumed dt**. `rs-imu-calibration` corrects gyro **bias only, not scale**. Gyros are untrustworthy for *unbounded* heading, not for bounded few-second measurements; that distinction is the whole argument.

**Measured (chassis-mounted, 2026-07-29, 200 Hz request delivering 199 Hz clean):**

| Quantity | Measured |
|---|---|
| Stationary bias | x −0.254, y −0.209, z −0.128 deg/s |
| Noise (sd) | 0.11–0.15 deg/s (~0.1% of a 150 deg/s pivot) |
| Scale error | **−0.6% over two hand turns** (−715.7 vs 720 deg) |
| Drift after bias removal | **0.003 deg/s** ≈ 10 deg/hr, vs −12.5 deg/**min** raw |

- **Yaw is gyro `y`, and `ROS yaw rate = −gyro_y`.** Gravity reads on accel `y` (−9.64 m/s²), so the camera's Y axis points down; by the right-hand rule, CCW-from-above is negative `gyro_y`. (`|g|` being 1.7% low is accel scale error — ignore it)
- A 12 s pivot inherits only ~0.04 deg of drift error

**⚠ librealsense must be built from source with `FORCE_RSUSB_BACKEND=ON` — the apt debs do not work here.** `ros-humble-librealsense2` exists as arm64 but its **native backend cannot read the D455 IMU on this Pi**: it wants a kernel HID-sensor path that does not exist, because the Pi kernel ships no `hid-sensor-*` modules, so the camera stays on generic `usbhid` and no IIO device appears. RSUSB reads the IMU over raw libusb instead. The Dockerfile builds **v2.57.7** from `realsenseai/librealsense`. **Do not "simplify" this to an apt install — it will build, run, and silently produce no IMU.**

**The `cmake` invocation in that RUN step is known-good — do not change it.** Every failure there so far has been a *missing apt package*, so fix those in the System Dependencies layer:
- **`libudev-dev`** — RSUSB compiles a bundled libusb whose udev backend needs `libudev.h`. The visible error is a generic `make: *** Error 2` ~70 lines later, so **read *up* the log to the first `fatal error:`**
- **`python3-dev`** for `BUILD_PYTHON_BINDINGS=ON` (`ros:humble-ros-core` has no `Python.h`). Also present: `libusb-1.0-0-dev`, `pkg-config`
- **Install paths are correct as-is, verified in the container.** `/opt/ros/humble/lib/aarch64-linux-gnu` is on `LD_LIBRARY_PATH` (ROS's `local_setup` adds the triplet dir), and `pyrealsense2` lands in `/usr/lib/python3/dist-packages/`, already on `sys.path`. Do not "fix" `CMAKE_INSTALL_LIBDIR` or add `PYTHON_INSTALL_DIR`

**ROS side, verified end to end (2026-07-29).** `realsense2_camera` 4.57.7, version-matched to librealsense v2.57.7 (wrapper `4.X.Y` ↔ lib `2.X.Y` — bump both together). Motion Module starts, gyro and accel open at 200 FPS, `/camera/camera/imu` publishes at **200.07 Hz**.
- The launch's `For the 'unite_imu_method' param update to take effect, re-enable either gyro or accel stream` warning is **benign**. If the united topic ever does fail, `/camera/camera/gyro/sample` is already a `sensor_msgs/Imu`
- **`gyro_calibrator` end-to-end drift: −0.07 deg/min** on the corrected `/imu/data` (vs −12.1 raw, 170×). It re-estimates bias whenever the robot is stationary (MEMS bias moves as the camera warms), publishes nothing during the startup window, fills `angular_velocity_covariance` at 7.0e-6, and advertises "no orientation" per spec with `orientation_covariance[0] = -1`

**⚠ Two pyrealsense2 traps** (they do not apply through the ROS wrapper, which handles both):
- **The first ~13 frames arrive in `timestamp_domain.hardware_clock`, then the stream switches to `global_time`** mid-flight. The dt across the switch is ~1.785e12 ms and detonates any naive integrator. **Filter on `f.get_frame_timestamp_domain() == rs.timestamp_domain.global_time`** — dropping a fixed number of leading frames is unreliable (the switch came after frame 13 in one run, frame 2 in another)
- **With gyro *and* accel enabled, `wait_for_frames()` returns each gyro frame ~twice** (50.2% duplication measured), because a frameset carries the latest sample of every stream. Dedupe on `get_frame_number()`, use per-sensor callbacks, or enable gyro alone

## Odometry EKF — gyro yaw + encoder distance (built 2026-07-30)

`robot_localization`'s `ekf_node` fuses each sensor along its good axis only: **forward speed from the wheels, yaw rate from the gyro, nothing else from either.** Apt `ros-humble-robot-localization` 3.5.4, config `scout/config/ekf.yaml`, started from `robot.launch.py`.

**Topic and TF ownership:**
- The driver's raw estimate is remapped to **`/wheel_odom`**; **`/odom` is the fused output**, so anything reading `/odom` gets the good yaw for free
- The `roboclaw_driver` service runs `ros2 run roboclaw_driver roboclaw_driver_node` directly, **not** `roboclaw_driver.launch.py`, because the launch file cannot remap and the node hardcodes the topic name `odom`. `--params-file` behaves identically since the node names itself `roboclaw_driver`
- **`publish_tf: false` in `roboclaw.yaml`** — the EKF owns `odom→base_link`
- The EKF **depends on `camera` and `robot_description` being up**, because it rotates the IMU into `base_link` through TF (the IMU is stamped `camera_imu_optical_frame`). No TF, no yaw

**⚠ THE TRAP: `imu0_config` names the SENSOR's axes, not the robot's — and "vyaw alone" silently fuses nothing here.** robot_localization builds a diagonal mask from those 15 flags, **rotates it by the sensor→`base_link` transform**, and the surviving rows decide which state variables update. On a body-aligned IMU sensor-Z *is* yaw, so every tutorial's "set vyaw true" works. This IMU reports in an **optical** frame with Z out of the lens, so a lone `vyaw` flag asks for rotation about the robot's *forward* axis; the mask rotates onto `vroll`, and `two_d_mode` zeroes it.
- **There is absolutely no warning.** Node log 0 bytes, `/diagnostics` reported "functioning properly", subscription counts healthy. Nothing was *rejected* — the measurement was faithfully applied to the wrong axis
- **How to tell fused from ignored, in one number:** `twist.covariance[35]` (vyaw variance) in `/odom`. Fused it sits at **6.6e-6**, essentially the gyro's own 7.0e-6. Ignored it **climbs without bound** (2.5 and rising). Same trick for the wheels via `twist.covariance[0]`: 1.1e-2 vs the driver's advertised 0.1. **A stationary robot cannot show you this any other way** — both sensors read ~zero, so the estimate looks perfect while unfused
- **Fix: mark all three rotational rates true.** Costs nothing (`two_d_mode` zeroes vroll/vpitch at 1e-6 covariance) and stays correct if the camera is remounted
- **The sign needs no fix — VERIFIED UNDER ROTATION** (hard floor, 1.5 rad/s, 8 s each way). The URDF's camera mounting already encodes `ROS yaw rate = −gyro_y`: the `base_link ← camera_imu_optical_frame` basis rows are `(0,0,1) (−1,0,0) (0,−1,0)`
- **⚠ QoS: `gyro_calibrator` publishes `/imu/data` best-effort** (`qos_profile_sensor_data`). A default *reliable* subscription receives **nothing** and says so only in a one-line `incompatible QoS` warning at discovery. The EKF gets this right; hand-written tools must ask for sensor QoS explicitly

**Measured:** `/odom` at 30.0 Hz with tighter jitter than the driver's (±0.2 ms vs ±1.8 ms — it runs on the filter's clock, not the serial poll). **Stationary yaw drift +0.070 deg/min**, exactly `gyro_calibrator`'s own figure, so the EKF adds nothing of its own. x/y hold at 0.00000 m; `two_d_mode` pins z.

**Deliberately not fused:** wheel pose (integrated by the driver using its own bad yaw), wheel `vyaw` (the scrub-corrupted number this exists to replace), wheel `vy` (hardcoded 0 with zero covariance, and lateral slip in a skid-steer turn is real), IMU orientation (none exists), IMU linear acceleration (worthless integrated; the wheels measure speed well).

**Untuned:** `process_noise_covariance` and `initial_estimate_covariance` are upstream defaults. Consequence: reported **yaw variance grows to ~1.7 rad² in 30 s**, wildly pessimistic against 0.07 deg/min of drift, because default yaw process noise is 0.06 rad²/s with no absolute yaw reference to pull it back. It cannot move the *estimate* (process noise only inflates covariance), but any consumer that gates on pose uncertainty needs this tuned first.

**The EKF is also the flat-tire diagnostic**: compare `/wheel_odom` yaw against fused `/odom` yaw over the same pivot, both directions. Measured on hard floor at 1.5 rad/s, the encoders over-reported yaw by roughly **twice as much one way as the other**. **The two directions agreeing is the signal that the tire is repaired.** Take the *pose* yaw, unwrapped, not the reported rate, so none of our own integration enters the comparison. Refuse to move unless `/odom`, `/wheel_odom` and `/imu/data` are all live (subscribe to `/imu/data` with **sensor QoS** or it reads as silent), publish `cmd_vel` at 20–50 Hz, and always send an explicit zero Twist at the end.

## LiDAR — streaming `/scan` (built 2026-07-30)

**⚠ The attached scanner is NOT the "RPLIDAR C1" in NOTES.md's parts list.** It is an **A2-family (triangulation)** unit: **256000 baud**, 16.0 m range, firmware 1.32, hardware rev 6, S/N `9A8FECF0C3E09ED4A0EA98F309574116`. A C1 is 460800 baud and 12 m — at 460800 this lidar never answers at all.

`rplidar_ros` built from source into `$OVERLAY` (SDK 2.1.0), config `scout/config/rplidar.yaml`, started from `robot.launch.py`, on a **CP2102 USB-UART bridge at `/dev/ttyUSB0`**. `usb_max_current_enable=1` is already set in `/boot/firmware/config.txt`.

**Verified:** `/scan` at **11.7 Hz** (motor runs slightly above the commanded 10 Hz), **1800 beams at 0.20°** over 360°, 96% valid returns, range 0.15–16.0 m.

**Scan modes, all at 16.0 m** (divide points/s by ten for points per rev at 10 Hz):

| Mode | Points/s | Per rev | Resolution |
|---|---|---|---|
| Standard | 4.0K | ~400 | 0.90° |
| Express | 7.9K | ~790 | 0.45° |
| Stability | 10.0K | ~1000 | 0.36° |
| Boost | 15.9K | ~1590 | 0.23° |
| **Sensitivity** (configured) | 15.9K | ~1590 | 0.23° |

Sensitivity is the highest point rate, the lidar's own reported *typical* mode, and reads low-reflectivity surfaces (dark furniture, black baseboards) better than Boost. Drop to **Stability** if bright ambient light causes dropouts. **Asking for an unsupported mode name makes the node print the whole table** — that is how this one was obtained.

**⚠ `/dev/serial/by-id/...` DOES NOT EXIST INSIDE THE CONTAINER.** Those symlinks are made by **udev on the host**; a privileged container gets its own `/dev` without them. Real device nodes (`/dev/ttyUSB0`, `/dev/bus/usb/*`, `/dev/spidev0.0`) are there, but udev's symlink farms are not.
- **The failure is deeply misleading:** the SDK opens nothing, `connect()` returns success, and it dies in `getDeviceInfo` with `Error, unexpected error, code: 80008004` (`RESULT_OPERATION_NOT_SUPPORT`) — which reads like "unsupported lidar model". The node's own "cannot bind to the specified serial port" message never fires
- So the config must use `/dev/ttyUSB0`, assigned in probe order. Unambiguous today (only USB serial device; the RoboClaw is on the GPIO UART). If a second is added, confirm identity **from the host** with `ls -l /dev/serial/by-id/` against the S/N above

**Sweep the baud before suspecting hardware.** A wrong baud gives `SL_RESULT_OPERATION_TIMEOUT`, indistinguishable from a dead or unpowered lidar. 115200 / 460800 / 1000000 all time out; only 256000 answers. Loop `ros2 run rplidar_ros rplidar_node --ros-args -p serial_baudrate:=$b` and watch for the S/N line — 25 s and it is settled.

**`laser` is a new REP-103 frame in the URDF and the driver points at it**, not the exporter's `lidar1_link`/`lidar2_link`, which are CAD-style (Z forward, Y up) and would tip every range reading 90° out of the floor plane. The joint hangs off `lidar1_link` (the rotating head).

**⚠ THE SCANNER IS MOUNTED BACKWARDS — 180° of yaw, corrected in the URDF (measured 2026-07-30).** Nothing in the CAD records how it is bolted to the mast, and the error announces itself nowhere: the scan looks perfectly plausible, just rotated, silently building a wrong map.

| Feature | Reported | Actually |
|---|---|---|
| 8.5 m open sight line | 180° | **in front** |
| 0.35 m obstacle arc | −85°…−7° (right) | **on the left** |
| 2.1 m opening | +90° (left) | on the right |

- **Fitting `true = s × reported + offset` needs two features.** One cannot separate a rotation from a mirror: `s=+1, offset=180°` puts the tight arc on the left, `s=−1, offset=180°` on the right. The operator's "left" picks the first — so the scan is **not mirrored** (`inverted: false` is correct) and it is a pure yaw
- **Fixed in the `lidar1_to_laser` joint, not the driver**, because it states how the hardware is bolted on. Composing 180° onto the CAD correction flips both signs: `0 -pi/2 -pi/2` → `0 pi/2 pi/2`
- **Self-checking via TF:** `base_link→laser` reads translation `(0.073, 0.000, 0.241)` with RPY `0 0 ±π`. Any *other* rotation means the chain is broken. Transformed into `base_link`, the open direction sits at 0° (8.70 m), the tight arc at +89°…+164°, the opening at −90°
- **Reusable method:** dump a top-down ASCII map plus bearings of the closest returns and longest sight lines, have the operator name where two *different* features really are, then solve for `s` and `offset`. No props, no robot motion

## SLAM — slam_toolbox mapping and localization (built 2026-07-30)

`slam_toolbox` 2.6.10 (apt) turns `/scan` into `/map` and owns **`map→odom`**. Config `scout/config/slam.yaml`, launch `scout/launch/slam.launch.py`, compose service `slam`. Inputs: `/scan`, **`odom→base_link` from the EKF**, and `base_link→laser` from `robot_state_publisher` — so the camera and `gyro_calibrator` are load-bearing, since the EKF's yaw comes through TF from the IMU.

Mapping rides on the *fused* `/odom`, so the flat tire does not corrupt the map — expect commanded pivots to under-rotate, not the map to be wrong.

**Three modes, selected by launch argument** (`mode:=new` default / `localization` / `continue`). Operating recipes are in NOTES.md. **⚠ Since ADR-0028 (2026-08-27), `localization` no longer runs slam_toolbox at all — it brings up nav2_amcl + nav2_map_server + a lifecycle manager (`scout/config/amcl.yaml`) on the grid map `<name>.yaml`/`.pgm`.** The webui Save Map writes both formats (`serialize_map` + `save_map`); the launch guard demands the grid, so pre-ADR sites must re-save from a `continue` session. tag_relocalizer's `/initialpose` seed is amcl-native. The slam_toolbox localization traps below (silent `serialize_map` SUCCESS, local-only lock, `map_start_at_dock` warning) are kept as history of *why*.

| Mode | Executable | Extra params | Behaviour |
|---|---|---|---|
| `new` | `async_slam_toolbox_node` | none | Fresh map |
| `continue` | `async_slam_toolbox_node` | `map_file_name`, `map_start_at_dock: true` | Loads a graph, keeps extending it |
| `localization` | amcl + map_server (ADR-0028) | `yaml_filename`, `initial_pose.*` | Loads a grid, adds nothing |

**Multi-map sites (ADR-0029, 2026-09-01):** `site.json` v2 holds a `maps` dict (label/floor/`map_start_pose` per map) + `active_map` (v1 `default_map` normalized on read, upgraded on fleet_status's next write). Map files stay flat in `sites/<name>/maps/`. tags.db surveys are stamped with their home map; tag_relocalizer auto-switches floors in localization mode (N-sighting hysteresis + cooldown, no nav goal, `/map_server/load_map` live swap + tag-solved `/initialpose`, then POSTs `active_map` to fleet_status). One surveyed pose per tag ID → **distinct physical tag per floor**. Manual switch: webui Site panel map list or scout-skills `switch_map`. Waypoints carry a `map` key; `go_to_waypoint` refuses cross-map.

**⚠ THE `mode` PARAMETER IS DEAD — every upstream config and tutorial sets it and it does nothing.** There is no `declare_parameter("mode", ...)` anywhere in `slam_toolbox_common.cpp`, `slam_mapper.cpp` or karto's `Mapper.cpp`. It is a comment with a colon in it. **The real switch is which executable runs:** `async_slam_toolbox_node` leaves `processor_type_` at `PROCESS`, while `localization_slam_toolbox_node`'s constructor sets `PROCESS_LOCALIZATION` (and kills the map saver, and forces `enable_interactive_mode_ = false`). So `slam.yaml` carries no `mode` key at all — copying a tutorial's params file gives a node that silently keeps mapping while called "localization".

**⚠ The map-loading params must be ABSENT, not `false`.** `shouldStartWithPoseGraph` requires `map_file_name` **and** one of `map_start_pose` / `map_start_at_dock`, testing them with `get_type() != PARAMETER_NOT_SET`. So `map_start_at_dock: false` reads as *set*. That is why the mode params are built as a dict in the launch file rather than living in `slam.yaml`. A bare `map_file_name` with neither companion loads **nothing**, logging only `Map starting pose not specified`.

**⚠ A missing map file is NOT fatal, and that is the dangerous part.** `deserializePoseGraphCallback` logs `Failed to read file` and then **`return true`** — the node comes up, publishes, and reports healthy while running on an empty graph, i.e. localizing against nothing. The launch file therefore checks `<name>.posegraph` exists and raises first, listing the maps that do exist.

- **⚠ `serialize_map` in localization mode does nothing and reports SUCCESS.** The callback logs `Cannot call serialize map in localization mode!` and `return false`, but slam_toolbox binds callbacks into rclcpp's `void(header, req, resp)` signature, so **the bool is discarded** and `resp->result` goes back default-initialised at `0` = `RESULT_SUCCESS`. The only evidence is the node's log and the missing file. Use **`continue`** to load a map and still be able to re-save it
- **`map_start_at_dock` is unusable in localization mode** — it warns `Starting localization at first node (dock) is correctly not supported` and localizes at the pose instead. So `localization` needs `map_start_pose` (defaults to origin; refine with `/initialpose`), and `continue` is the only mode that can use the dock
- **⚠ Every restart in `localization` mode therefore assumes the robot is back at the map's origin, and a wrong guess looks like a right one.** Scan matching only searches locally, so a small `map→base_link` correction means either "genuinely near the origin" or "stuck at a bad guess it could not escape" — the number cannot tell you which. **Confirm from the scan-vs-map overlay in Foxglove before trusting a plan**, since a mislocalized robot plans through walls and fails in ways that mimic tuning problems. If the stack is restarted anywhere but the starting spot, seed it with `/initialpose` (in the `map` frame — see the Nav2 goal-frame trap)
- **`map_file_name` is a basename with no extension** — `serialization::write`/`read` append `.posegraph` and `.data`. Only **`serialize_map`** writes the pair the load modes need; **`save_map`** writes a `.pgm`+`.yaml` that slam_toolbox cannot load back
- **`base_frame` must be `base_link`**, not the upstream default `base_footprint`, which this URDF does not define. `base_link` already sits at floor level; the default fails every TF lookup and logs `Failed to compute odom pose` forever
- **Maps go in the active site's `sites/<name>/maps/` (ADR-0023)**, reached as `/ros_ws/src/sites/active/maps` (inside the `.:/ros_ws/src/` bind mount, so they reach the host). The directory must exist first — boost serialization will not create it and the failure surfaces only as `RESULT_FAILED_TO_WRITE_FILE`, hence the tracked `maps/.gitkeep`. Paths passed to either service must be **absolute** (the node's CWD is `/ros_ws`). Files are written root-owned, but `maps/` is user-writable so deleting them needs no `sudo`. A stationary session's `.posegraph` was already **11.8 MB** — expect tens of MB for a house, which is why `maps/` contents are gitignored
- **No QoS trap here:** `rplidar_node` publishes `/scan` RELIABLE and slam_toolbox subscribes BEST_EFFORT, which is compatible
- **`min_laser_range: 0.15` / `max_laser_range: 16.0`** match this unit rather than the 20 m default. `enable_interactive_mode: false` because the graph-editing markers are an rviz feature and Foxglove is the viewer
- **`minimum_travel_distance` / `minimum_travel_heading` are 0.3**, tightened from 0.5 — judgement, not measurement (0.5 rad is 29° between keyframes on a robot that pivots a lot). These are the first knobs to relax if the Pi struggles, along with `map_update_interval` (2.0 here vs 5.0 upstream), the one periodic cost that scales with map *area* rather than motion
- **⚠ Two benign startup lines, both alarming-looking.** `minimum laser range setting (0.1 m) exceeds the capabilities of the used Lidar (0.2 m)` — **both numbers are the same 0.15**; the parameter is float64 and `range_min` is a promoted float32, so the comparison is true by 6e-9 and `%.1f` rounds them opposite ways. The clamp lands on the value we wanted. **Do not raise the config to 0.2 to silence it** — that discards real returns. And `Message Filter dropping message: frame 'laser' ... queue is full` fires exactly **once**, in the ~2 s before TF is warm; `scan_queue_size: 1` correctly drops an untransformable scan rather than queuing a stale one

**Verified end to end (2026-07-30, stationary, hard floor):** `/map` at exactly the configured 0.500 Hz, `map→odom` appears, `/tf` publishers are exactly three (`robot_state_publisher`, `ekf_filter_node`, `slam_toolbox`). A stationary scan produced a 188 × 150 cell grid at 0.05 m. All three modes exercised — `localization` scan-matched back onto a saved graph with a **1 mm** correction on an unmoved robot. All three launch guards (unknown mode, missing `.posegraph`, malformed `map_start_pose`) refuse to start.

**The deb costs 304 packages and ~866 MiB (image ~2.5 → ~3.2 GB) and there is no lighter path.** `slam_toolbox`'s `CMakeLists.txt` has a hard non-optional `find_package(rviz_common REQUIRED)` for its rviz plugin, so the whole rviz/Qt/OGRE stack comes with it. Building from source would need the rviz **dev** packages — strictly worse.

## Nav2 — path planning and following (built 2026-08-03)

Nav2 **1.1.20** (apt), config `scout/config/nav2.yaml`, compose service `nav2`, running upstream's `nav2_bringup/navigation_launch.py` directly (as `robot.launch.py` reuses `rs_launch.py` for the camera). Eight lifecycle nodes: controller, smoother, planner, behavior, bt_navigator, waypoint_follower, velocity_smoother, lifecycle manager. **No `amcl` or `map_server` section in `nav2.yaml`** — the `slam` service owns `/map` and `map→odom` (slam_toolbox when mapping; amcl + map_server in localization mode, ADR-0028).

**Topic and TF ownership:**
- `navigation_launch.py` remaps controller_server's output to **`/cmd_vel_nav`** and velocity_smoother's `cmd_vel_smoothed` back to **`/cmd_vel`**, so `roboclaw_driver` needs no change and no launch file of our own is required
- **Nav2 publishes no TF.** Inputs are `/scan`, `/map`, and `map→odom→base_link→laser`, so the entire chain (lidar, camera, gyro_calibrator, ekf, robot_description, slam) is load-bearing
- Goals arrive from Foxglove on `/goal_pose`; bt_navigator forwards them to its own NavigateToPose action. **Foxglove stamps a clicked goal in whatever frame the 3D panel is *displaying*, and it must be `map`** — see the goal-frame trap below

**The whole file is tuned for tight clearance (pipes barely wider than the robot), and the mechanism is to move the collision decision from "cost under the robot's centre" to "does the footprint polygon overlap an obstacle".**
- **The real footprint polygon replaces `robot_radius: 0.22`:** `[[0.169, 0.167], [0.169, -0.167], [-0.169, -0.167], [-0.169, 0.167]]` from the mesh bounds, with `footprint_padding: 0.0` (upstream 0.01). This drops the **inscribed radius from 0.22 to 0.167**, and that is the number that decides the narrowest passable gap: `InflationLayer` marks everything within the inscribed radius as cost 253 and **NavFn treats 253 as impassable**, so the planned corridor is `pipe_width − 0.334` — exactly the physical fit and nothing more
- `inflation_radius: 0.25` / `cost_scaling_factor: 10.0` (upstream 0.55 / 3.0), leaving ~8 cm of steeply-decaying gradient past the inscribed radius instead of 33 cm of penalty on a 33 cm robot. **These are the "do not be afraid" dials**, and they must stay in step between the two costmaps or the planner and controller disagree about the cost of hugging a wall
- **`ObstacleFootprint` replaces upstream's `BaseObstacle` in the DWB critic list, and that single swap is the core of it.** `BaseObstacle` scores the one cell under `base_link`, so with any inflation it is really a distance-from-wall penalty — inside a pipe the centre cell is permanently inflated and the robot refuses to be where it must be. `ObstacleFootprint` rasters the oriented polygon and rejects a trajectory only when it actually covers a lethal cell, so passing 2 cm from a wall scores like passing 20 cm from it. Keep its `scale` tiny (0.02) so residual inflation stays a nudge
- **Depth (under-lidar):** both costmaps carry an **`stvl_layer`** (spatio_temporal_voxel_layer, 2026-08-24 — replaced the old `depth_mark`/`depth_clear` ObstacleLayer pair and the Python clutter_mapper; ADR-0024 records the migration). Marks the 0.05–0.22 m band from the decimated depth cloud; clearing is frustum decay, never lidar raytrace (doorway over-clear bug). Local decays 15 s, global 4 h (clutter memory). A second **marking-only `cliff` source** on the same layers carries cliff_detector's odom-latched ledge marks (ADR-0024); collision_monitor additionally hard-stops on `/cliff/stop_points`
- **`always_send_full_costmap: True` temporarily** so Foxglove can see `/local_costmap/costmap` (it ignores `/costmap_updates`). Flip both costmaps back to `False` when done testing — controller/planner do not use the published topic; full grids are DDS/CPU only
- Controller is `RotationShimController` wrapping **DWB** (pivot onto the path, then follow it — a skid-steer should never arc onto a path, and in a pipe there is no room to). The primary controller's params live in the **same `FollowPath` namespace**; that is how the shim loads them
- Local costmap plugins: `obstacle_layer` + `stvl_layer` + `inflation_layer`, no static layer: a live 3 m rolling view at **0.025 m** cells. Global costmap is static + obstacle (lidar) + stvl + inflation with `track_unknown_space: true` and the planner's `allow_unknown: true`

**Verified end to end (2026-08-03, hard floor, ~20.x V pack):**
- All eight nodes activated on the first try with **zero warnings or errors**. Local costmap 120 × 120 at 0.025 m; global costmap 106 × 134 at 0.05 m; published footprint is the real polygon
- Zero-motion planning: **4 of 4** `ComputePathToPose` goals at ±1 m in x and y succeeded
- Controller output **15.6 Hz** on `/cmd_vel_nav` pinned at the 0.35 m/s cap; velocity_smoother re-emitted at **28.5–30 Hz** on `/cmd_vel`
- **A 0.8 m forward goal SUCCEEDED in 2.84 s.** Odom displacement 0.665 m with 0.013 m of lateral drift, final yaw −0.003 rad. It stopped **0.135 m short**, i.e. exactly inside the 0.15 m `xy_goal_tolerance` — expect that, and tighten the tolerance (not the speed) if a pipe needs the robot to arrive closer. `/cmd_vel` ended in explicit zero Twists and then went silent
- **Zero control-loop misses and zero new driver serial errors during the drive.** Checked node by node, **none of the nav2 nodes advertises `/tf`** (only `ekf_filter_node` did, of everything discovered) — so nav2 adds no transform publisher. Note this was confirmed per node rather than by a total publisher count, for the discovery reason below
- **Four consecutive Foxglove goals across a room SUCCEEDED back to back on a saved map: 1.14 m in 3.8 s, then 4.23 m in 24.9 s, 4.42 m in 14.2 s, 3.96 m in 17.3 s** — the first clean end-to-end runs of the real path (operator click → `map`-framed `/goal_pose` → `slam_toolbox` localization → drive), and the first that outlast the 10 s goal-frame trap below by more than double. **Arrival error was 0.13–0.14 m on every single one**, which is the `xy_goal_tolerance` showing through, not scatter — it is a systematic stop-short, so tighten the tolerance if a pipe needs a closer arrival
- **Localization held to a 0.30 m / 2.1° `map→odom` correction over ~17 m of driving**, so slam_toolbox in `localization` mode tracks well once it starts from the right place
- **The `Spin` recovery fired once mid-goal and the goal still succeeded** (3.2 s, `spin completed successfully`), so the recovery path works and is not automatically a lost goal
- Not yet exercised: reverse (`min_vel_x: 0.0`), `waypoint_follower`, `smoother_server`

**⚠ DO NOT RUN `docker compose run` DIAGNOSTIC CONTAINERS WHILE NAV2 IS NAVIGATING — it starves the Pi and fails the goal.** One throwaway container spawned to read TF during a 4.87 m drive **aborted it**: `Behavior Tree tick rate 100.00 was exceeded!` climbed from 5/s to 16/s over four seconds, then *every* action call missed its acknowledgement (`Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose`, then `follow_path`, `wait`, `backup`, `spin`), and bt_navigator tore through the whole recovery branch in ~200 ms and aborted. Host load was 3.47 on 4 cores. **The warnings name the BT and the action servers, so it reads like a nav2 or a tuning fault, and it is neither.** Watch a live goal from **host-side `docker compose logs`**, which costs nothing, and save container-based checks (`tf2_echo`, `topic echo`) for when the robot is idle.

**⚠ "Goal failed" DOES NOT STOP THE ROBOT — behaviors already dispatched keep running, and this is the second way a goal outlives the thing that reported it.** In that aborted run bt_navigator gave up at `t+12.6 s`, and behavior_server then went on to finish a 0.30 m `BackUp` (`t+14.1`), a `Wait` (`t+17.3`) and attempt a 90° `Spin` (failed at `t+22.3`) — **~10 s of unattended motion after the error line**. All three had been requested in the same 200 ms burst, so they also ran *concurrently*. **`controller_server` kept following the path through all of it and logged `Reached the goal!` at `t+31.8 s`** — the drive the BT called failed actually completed, 19 s after the error. Treat the abort as the start of the danger window, not the end of it, and note that "Goal failed" says nothing about where the robot ended up.

**⚠ A GOAL STAMPED IN `odom` INSTEAD OF `map` WORKS FOR ~10 SECONDS AND THEN FAILS — set Foxglove's 3D panel display frame to `map`.** Symptom: the robot starts off correctly, then `planner_server` logs `Could not transform the start or goal pose in the costmap frame` on every replan, the BT falls through to `Spin`/`BackUp` recovery, and the goal aborts. Nothing names Foxglove or the frame.
- **Mechanism:** bt_navigator keeps the goal's *original* header and hands the same `frame_id` + stamp to the planner on every 1 Hz replan. `Costmap2DROS::transformPoseToGlobalFrame` returns immediately when `frame_id` already equals the global frame, but otherwise does a real TF lookup **at that fixed stamp**, and the tf2 buffer only holds 10 s. So the lookup succeeds until the goal is 10 s old and fails forever after. A `map`-framed goal takes the short-circuit and cannot age out
- **This is why short goals appeared to work while longer ones "failed"** — the 0.8 m / 2.84 s test finished inside the window. Duration, not distance, is what decides
- **The `frame_id` is the whole diagnosis, and only the wire shows it.** Capture it with `ros2 topic echo /goal_pose geometry_msgs/msg/PoseStamped` — pass the type explicitly or `echo` exits instantly when the topic is idle, and **run it without piping into `grep`**, which block-buffers and hides the message until 4 KB accumulates (use `grep --line-buffered` or `stdbuf -oL`)
- The same trap applies to anything else clicked in that panel, notably **`/initialpose`** for correcting localization

**⚠ A GOAL IS NOT CANCELLED BY KILLING THE CLIENT — the most dangerous thing found here.** `ros2 action send_goal` hitting its timeout kills only the *client*; bt_navigator keeps the goal, replans at 1 Hz and **keeps streaming 0.35 m/s forward commands indefinitely**. It survived for minutes, visible only in the node's log, and would have driven the robot the instant the driver came up. There is no GUI cancel button in Foxglove. To actually clear it, `docker compose restart nav2` (or deactivate `bt_navigator` via its lifecycle service). This is on top of the older trap that `docker stop` on the container leaves the last command latched for up to `velocity_timeout` and then only coasts.

**⚠ DDS discovery from `docker compose run` throwaway containers is unreliable on this host, and it lies in the direction of "nothing is there".** Measured, all false negatives: `/cmd_vel` read as *no publisher at all* for 16 s while it was publishing at 28.5 Hz; `ros2 node list` omitted `robot_state_publisher` and `slam_toolbox` while both were running; `ros2 lifecycle get /controller_server` said `Node not found` while that process's own child costmap was publishing; `/roboclaw_status` read fine twice then vanished. **Judge liveness from the node's own log and from data topics, never from a fresh container's discovery** — and re-check a negative after 10–20 s before believing it. Relatedly, **`ros2 topic pub --once /goal_pose` from a throwaway container is silently lost**, because it publishes and exits before discovery matches; use the action instead, which waits for the server.

**Other traps and deliberate departures from upstream:**
- **The global costmap's `resolution` is very nearly cosmetic.** `StaticLayer` resizes the master grid to whatever `/map` arrives at, logging `StaticLayer: Resizing costmap to 106 X 134 at 0.050000 m/pix`. Finer global cells for pipe work means changing `resolution` in `slam.yaml`, not `nav2.yaml`
- **Local costmap `width`/`height` are declared `int`** — `3.0` is a type error, not a rounding
- **The drivetrain velocity floor is enforced in DWB, not as a smoother deadband:** `min_speed_xy: 0.05` + `min_speed_theta: 0.35` (~300–500 counts/s, and 2·0.05/0.278 for a pivot). `KinematicParameters::isValidSpeed` rejects a sample only when it is below **both**, so a slow pure rotation and a straight crawl stay legal while untrackable "barely creeping and barely turning" is thrown out. A `deadband_velocity` would instead chop the smoother's own ramp and act per axis
- **`behavior_server` publishes `/cmd_vel` DIRECTLY, bypassing the velocity smoother**, so its `cycle_frequency` *is* a cmd_vel rate — raised 10 → **20 Hz**, since 10 Hz is the cadence that measurably worsened carpet dips
- `controller_frequency: 15.0` with `smoothing_frequency: 30.0`: the smoother interpolates its own 30 Hz output between the controller's commands, so **the smoother is what feeds the 200 ms deadman** and the controller is free to be slower than 5 Hz would allow
- **The default recovery BT backs up 0.30 m at 0.05 m/s**, right on the drivetrain floor, so recovery may barely move. Copy the XML to `/ros_ws/src/scout/` with ~0.10 m/s if it matters
- Pivot rates are conventional (`rotate_to_heading_angular_vel: 1.2`, `max_vel_theta: 1.2`, behavior_server `max_rotational_vel: 1.2`) and **deliberately ignore the flat tire's ~2.5 rad/s scrub floor** — revisit all three together when the tire is repaired
- **`Rotation Shim Controller was unable to find a goal point, a rotational collision was detected, or TF failed to transform into base frame! what(): Failed to transform pose to base frame!` is INFO, not an error, and is benign** — 7 occurrences across four successful drives. The shim catches its own exception and hands the cycle to DWB, which is the intended fallback. Only worry if it repeats every cycle
- **Two benign log lines.** `No goal checker was specified in parameter 'current_goal_checker'. Server will use only plugin loaded general_goal_checker` fires once per run and is just the BT not using a goal-checker selector. And the global costmap's `Message Filter dropping message: frame 'laser' ... the timestamp on the message is earlier than all the data in the transform cache` appears a few times per session (distinct from slam's once-at-startup `queue is full` version)
- **The driver's startup serial burst is not a nav2 problem and not worth chasing:** 69 `RETRY COUNT EXCEEDED` plus CRC failures over the first 10 s, then completely clean, including through the drive. `resyncSerial()` doing its job. Confirm the link is really alive by reading a *hardware* number out of `/roboclaw_status` — encoder counts and the two temperatures can only come from the board
- **CPU:** nav2 ~39% of one core, the whole stack ~1 core of the Pi 5's 4, memory negligible. **`Control loop missed its desired rate of 15.0000Hz` is the stack's contention gauge** — a few per drive are normal (they cluster around the shim's fallback cycles), but it fires in bursts whenever anything else takes a core, which is how the throwaway-container abort above announced itself. First levers if it degrades: `vx_samples`/`vtheta_samples`, then `controller_frequency`, then the local costmap's 0.025 m resolution

## Docker

**One install tree: `$OVERLAY=/opt/overlay`.** Image-baked forks (`roboclaw_driver`, realsense, rplidar) and locally built Scout packages all install there. The entrypoint sources only `$OVERLAY/install`. Compose mounts named volume `ros_overlay_install` on `/opt/overlay/install` so `build_package` survives `run --rm` (an empty volume seeds from the image once).

- `colcon` merges into an existing install base without orphaning what is there, so each Dockerfile `RUN` keeps its own cached layer. Adding the Nth package is one line: `git clone … "$OVERLAY/src/<name>" && build-overlay --packages-up-to <pkg>`
- Compose `build_package` installs with `--install-base /opt/overlay/install --symlink-install` against the `.:/ros_ws/src/` bind mount
- **⚠ Volume still seeds only once.** Bumping an image-baked fork pin requires wiping `ros_overlay_install` (`docker compose down -v` or `docker volume rm …_ros_overlay_install`) then rebuilding so the volume re-seeds and `build_package` re-adds Scout. There is no longer a second `/ros_ws/install` path that can shadow a live overlay
- Compose runtime is four services: `robot` (core launch), `slam`, `nav2`, `foxglove_bridge`
- **Put new apt packages *after* the librealsense `RUN`.** Adding them to the earlier layer invalidates its cache and costs a full librealsense rebuild — 13 min of the 13.5 min total on a Pi 5, vs 1.5 min for the wrapper alone
- Runtime: `privileged: true` is sufficient for USB and SPI devices; no explicit device mounts needed (but see the `/dev/serial/by-id` caveat under LiDAR)

**⚠ All bench and calibration tooling was DELETED on request (2026-07-30), along with the compose `test` profile.** Every measurement in this file came from tools that no longer exist — treat the numbers as the record and expect to rebuild the instrument before extending them. Removed: `spin_diagnostic.py` (spin and straight-line sweeps with duty/current/encoder/voltage logging), `tune_velocity_pid.py` (packet-serial client + PID autotune), `wheel_radius_calibrator.py`, `motor_test.py` (raw-UART open-loop duty + breakaway ramp), `led_test.py`, `pivot_check.py`. Three were untracked, so there is no git history to restore from. Only `gyro_calibrator.py` was kept, being a runtime node.

**Exception (2026-08-12): `scripts/camera_health.py` and `scripts/camera_selfcal.py` are KEEPERS**, not bench rigs — recurring camera maintenance instruments (stereo calibration drifts with temperature and knocks), same class as the kept `gyro_calibrator`. `camera_health.py --plane` is the plane-fit RMS check (subpixel <0.1 good, >0.2 recalibrate); `--watch` is the MinZ/MaxZ instrument for the disparity-shift bench. Both need the robot service stopped first (device claim). **IMU flash recalibration is deliberately not provided** — online `gyro_calibrator` bias estimation is the only IMU path the EKF consumes, `rs-imu-calibration` corrects bias-not-scale anyway, and flash writes sit next to the Motion-Module wedge hazard. Do not re-litigate.

## Pi-side control notes

- RoboClaw is on the Pi 5 GPIO UART **`/dev/ttyAMA0`** (needs `dtparam=uart0=on` in config.txt). **NOT `/dev/ttyAMA10`**, which is the debug/console UART
- `sudo usermod -aG dialout $USER`; purge `modemmanager` if present
- Deployed controller is the **`roboclaw_driver` ROS 2 node** (wimblerobotics/Sigyn lineage; built in the Dockerfile from **our org fork `BG-BGI/roboclaw_driver`**, pinned by SHA — no dependency on personal repos), started from `robot.launch.py` so its odometry can be remapped to `/wheel_odom`. Params in `scout/config/roboclaw.yaml`, address 0x80, 115200 baud. **There is no standalone raw-UART bench path any more** — open-loop duty commands need a new tool
- The node converts `/cmd_vel` → per-wheel QPPS and sends **`MIXEDSPEEDACCELDIST`** each time a *new* `cmd_vel` arrives. There is no send-always. So the **publisher** must stream ≥5 Hz (use 20–50 Hz) to satisfy the deadman, and must send an explicit **zero** Twist to stop — otherwise the deadman just free-wheels to a coast
- **⚠ `accel` doubles as a top-speed limiter — do NOT use "a few thousand".** Each command carries a distance cap = `commanded_counts/s × max_seconds_uncommanded_travel (T)` and must decelerate to a stop within it, so reachable speed ≈ `sqrt(2·accel·V·T)`. To actually reach commanded V you need **`accel ≥ V/(2·T)`** — 1.2 m/s ≈ 7885 counts/s at T=0.2 s needs ~19,700, and the config is 20000. accel 3000 pinned the robot at ~0.3–0.4 m/s. Lowering T tightens overrun distance but raises the accel floor
- The C++ driver wraps serial I/O in try/except with `resyncSerial()` — transient CRC errors (a few are normal at startup) are retried, not fatal; dropouts coast via the deadman and the driver reconnects

## LED strip (APA102) Pi-side notes

- Driver is the **`led_node`** ROS node (`scout/scout/led_node.py` + `apa102.py`), started from `robot.launch.py`. `python3-spidev` is in the Dockerfile
- **Enable SPI first:** uncomment `dtparam=spi=on` in `/boot/firmware/config.txt` and **reboot**
- **The header SPI0 is `/dev/spidev0.0`.** On the Pi 5 it lives on the RP1 (`spi@50000`, alias `spi0`) and only appears once `dtparam=spi=on` is set
- **⚠ `/dev/spidev10.0` is a trap.** It is the BCM2712's internal SPI (`spi@7d004000`, alias `spi10`), present even when header SPI is disabled and **not wired to GPIO10/11**. Opening it and calling `xfer2` succeeds silently while nothing reaches pins 19/23 — a dark strip with a "passing" program. This burned a whole session
- **Verify the mux, don't trust the device node:** after reboot `pinctrl get 10,11` must read **`a0`** (ALT0 = SPI). `none` means the pins are plain GPIO regardless of what `/dev/spidev*` exists
- APA102 frame format (in `show()`): start = 4×`0x00`; per-LED `0xE0 | brightness(0–31)` then **BLUE, GREEN, RED**; end = `ceil(NUM_LEDS/16)` bytes of `0xFF`

## LED strip EMI (resolved)

**Symptom:** flicker and wrong colors *only while the motors were driving* — worst on the left side driving forward — plus glitches on motor start/stop.

**Two mechanisms:**
1. **Capacitive coupling** onto DATA/CLOCK from the RoboClaw's high-`dV/dt` H-bridge switching. Confirmed capacitive (E-field displacement current), not magnetic: twisting the motor leads did *nothing* (that only cancels inductive `dI/dt` coupling) while a grounded foil shield *did* help
2. **Ground-reference bounce** on motor start/stop — the strip's logic ground momentarily shifting relative to the Pi's during the current transient, moving the reference the SPI logic thresholds against

**Fix — target the victim, not the source:**
- **Shortened the DATA/CLOCK pigtails** to the bare minimum. The long unshielded jumpers were pickup antennas; shortening cleared the gross flicker
- **Dedicated signal-ground reference** via the star-ground split under **LED strip** above (black #1 → buck −, black #2 → Pi GND bundled with the signal pigtails). This cleared the start/stop residual

**Why it worked:** the coupling was capacitive, so the cure is shorter/shielded signal runs, not lead twisting. Splitting signal ground from power ground at the strip keeps power-return current out of the reference the SPI logic compares against. Signal-line and signal-ground length and routing are what matter; power +/− jumper length is not. The shared buck ground does not make it worse — motor return flows Battery± → driver → Battery± on the input side and never through the strip's ground path.
