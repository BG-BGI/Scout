# ADR-0015: Three-tier process-exit policy in robot.launch.py (fail-fast bring-up)

Status: accepted · Date: 2026-08-15

## Context

`robot.launch.py` starts 20 nodes. Before this, only `led_node` had a death
policy (respawn); any other process dying left a **half-running stack** —
containers up, topics partially flowing, failure visible only to whoever reads
the right log. The worst cases lie actively: a dead `gyro_calibrator` starves
the EKF of yaw through TF while `/odom` keeps publishing wheel-only data; a
dead `twist_mux` silently disconnects every motion source from the driver.

The alternative considered was F4 (managed lifecycle bring-up,
remaining-plan §8.4): ordered activation + per-node deactivate/reactivate.
Honest ROI review said most of F4's value here is just "dead node must not
equal half-running stack" — which launch-native event handlers buy at ~5% of
the conversion cost. F4 stays specced; re-evaluate after living with this.

## Decision

Every node in `robot.launch.py` (and `description.launch.py`) is in one of
three tiers:

1. **Fail-fast** — `RegisterEventHandler(OnProcessExit(target_action=node,
   on_exit=[EmitEvent(Shutdown)]))` via the `_fail_fast` helper. Node death
   shuts the whole launch down; compose `restart: unless-stopped` recycles the
   service into a known-good full bring-up. Members: `twist_mux`, `estop`,
   `roboclaw_driver`, `gyro_calibrator`, `ekf_filter_node`,
   `robot_state_publisher`. Criterion: **the stack lies about itself when this
   node is dead** (motion chain, yaw source, TF backbone).
2. **Respawn** — `respawn=True, respawn_delay=2.0`. Death is recoverable and
   losing the node degrades one feature without corrupting anything:
   `led_node` (pre-existing: live SPI TimeoutError), `rplidar_node` (CP2102
   USB hiccups; robot stays drivable), `joystick_teleop` (gamepad
   unplug/replug), `apriltag` (vision-only).
3. **Plain** (no policy) — inert-until-called or purely-observing nodes whose
   death is survivable and surfaced elsewhere: `battery_monitor`,
   `health_monitor` (its own death = `/diagnostics` goes stale in Foxglove),
   `tilt_monitor`, `trick_player`, `follow_me`, `clutter_mapper`,
   `patrol_capture`, `led_status`, `link_watchdog`, `wheel_joint_relay`.

**The camera is deliberately outside the tiers.** It starts behind upstream
`rs_launch.py` via `IncludeLaunchDescription`, so there is no local Node
action for `OnProcessExit` to target. Its death cascades to `/imu/data`
silence → EKF yaw stale → health_monitor; if that proves too slow in
practice, the fix is wrapping the camera in a scout-owned Node action, not
weakening the tier rule.

Deadman note: fail-fast recycling the robot service coasts the drivetrain
(RoboClaw 200 ms timeout → Free Wheeling) — identical to today's
crash behavior. No new hazard.

## Consequences

- A dead load-bearing process now produces a **service restart with a named
  reason** (`Shutdown(reason=...)` in the launch log) instead of a limp.
- Restart loops become visible in `docker compose ps` (restart counter) —
  a crash-looping driver is diagnosable at a glance.
- Respawned nodes re-run their startup (lidar re-probes baud, joystick
  re-opens evdev); anything with one-shot state must tolerate that (all
  current tier-2 members do).
- tilt_monitor/estop asymmetry is intentional: estop is fail-fast because its
  death removes the mux lock heartbeat (a safety *actuator*); tilt_monitor
  only *requests* actions of others.
- Verify on Pi (remaining-plan §9.2): `kill -9` a tier-1 PID in the container
  → whole service recycles and comes back healthy; kill a tier-2 PID → node
  respawns publishing within ~2 s; stack otherwise undisturbed.
