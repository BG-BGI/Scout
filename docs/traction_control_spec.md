# Scout Traction Control — Spec

## Constraint
Left/right are single RoboClaw channels, two motors paralleled per channel, only rear encoder wired per side. Front wheel state is unobservable and uncontrollable independently. No per-wheel fix is possible — only per-side throttling.

**Observed fault is consistently front-wheel-only** (never rear). This means the rear encoder stays grounded/loaded and keeps tracking true wheel speed accurately — the closed loop's feedback is not corrupted, unlike the rear-lifts case CLAUDE.md warns about (front stalls, rear spins free, encoder blind to it). A lifted front wheel just reduces total side current for a given measured speed, with no speed-tracking confound. Use **measured speed** (`m1_speed`/`m2_speed` from `roboclaw_status`), not commanded QPPS, as the independent variable for the expected-current lookup below — it's trustworthåy in this failure mode.

## Goal
Detect "a wheel on this side is unloaded" (off ground / uneven terrain) and derate that side's commanded speed so it doesn't free-spin at full duty while doing no work.

## Signal
`/roboclaw_status` (existing, 10 Hz, `std_msgs/String` JSON) already carries `m1_current`, `m2_current`, `m1_speed`, `m2_speed`. No new driver/firmware changes needed.

- **Per-side apparent load** = motor_current / |measured_speed|, using the rear encoder's own reported speed (trustworthy here, since it's always the front that lifts). A side with its front wheel off the ground draws less current for the same measured speed than a side with both wheels loaded, because the free wheel carries none of the propulsion torque.
- Gate on **measured speed magnitude ≥ ~20% of `qpps` limit (9240 → ~1850 counts/s)**, matching the CLAUDE.md finding that current telemetry is noise below ~20% duty. Below that, suppress the check (no verdict, not "safe" or "fault") — this is also exactly the crawl-speed regime where uneven terrain is most likely to lift a front wheel, so it's a real coverage gap, not just a formality.

## Baseline (must calibrate empirically, not guessed)
Record current at several measured speeds ≥20% duty on:
1. both wheels loaded (normal floor drive), both sides
2. **one front wheel propped up** (block/ramp under a single front tire, rear stays weighted and grounded — matches the actual observed fault, not a whole-side lift), forward and pivot, both sides

Fit or table `expected_current(measured_speed)` for the loaded case per side (m1, m2 differ slightly per roboclaw.yaml PID gains). Flag when measured current on a side falls more than some margin (start at 30-40%, tune from data) below that expected curve at the same measured speed.

## Node design
New node in `scout` package, e.g. `traction_monitor.py` (mirrors `tilt_monitor.py`'s pattern: pure monitor + advisory topic, no direct actuation).

**Subscribes:**
- `/roboclaw_status` (JSON string) only — parse `m1_current`, `m2_current`, `m1_speed`, `m2_speed`. Commanded `/cmd_vel` is no longer needed as an input to the detector: gating and the expected-current curve are both keyed on *measured* speed, which is trustworthy under the front-only fault. (Still needed downstream at the actuation point to scale outgoing commands — see below.)

**Publishes:**
- `/traction/side_derate_factor` (or two Float32, `left`/`right`) — multiplier in [derate_floor, 1.0] applied to that side's target speed. 1.0 = normal, drops when slip flagged.
- `/traction/status` (diagnostic string/bool per side) — for logging/Foxglove, not required for control loop.

**Logic per control tick (10 Hz, matched to status_rate):**
```
for side in (left, right):
    if |measured_speed[side]| < gate_threshold:
        derate[side] = 1.0   # no verdict below the noise floor
        continue
    expected = expected_current(side, |measured_speed[side]|)
    if measured_current[side] < (1 - margin) * expected:
        derate[side] = max(derate_floor, derate[side] - step_down)
    else:
        derate[side] = min(1.0, derate[side] + step_up)
```
Asymmetric step (fast down, slow up) avoids chattering back to full speed the instant current blips back up from PID lag.

**Actuation point — pick one, does not change the node above:**
- (a) New node republishes a scaled `/cmd_vel` to the driver, becoming the sole writer of `/cmd_vel` that the driver subscribes to (teleop/nav2 publish to `/cmd_vel_raw`, this node remaps). Cleanest, single source of truth.
- (b) Driver takes a new per-side scale parameter/topic — requires a change to `roboclaw_driver` (external repo), not recommended first pass.

Recommend (a): no upstream repo changes, matches how `tilt_monitor` already intercepts `navigate_to_pose`/`explore/resume` rather than the driver.

## Known gaps / explicitly out of scope
- Cannot control the front wheel independently even once detected — only the whole side can be throttled (paralleled motor channel).
- Cannot equalize four wheel rates independently — hardware has 2 channels, not 4.
- Below ~20% duty (crawl speeds, exactly where uneven terrain matters most) the signal is unusable by CLAUDE.md's own current-telemetry finding — this is a real coverage gap, not a tuning problem. If low-speed detection is needed later, the only other lever is IMU yaw-vs-expected-yaw, which doesn't catch a single-wheel partial free-spin either (it only shows up as a mild yaw residual, not a clean fault signal) — flagged here, not designed.
- Needs baseline current curves collected on the actual robot before any threshold is trustworthy — do not ship default margins un-calibrated.

## First test to run
Prop up a single front wheel (block/ramp, rear stays grounded and weighted — matches the actual fault) at 2-3 measured speeds ≥20% duty, both sides, log `roboclaw_status` current + speed to compare against a normal-floor run at the same speeds. That comparison is the calibration data the whole threshold depends on — nothing else in this spec is buildable first.
