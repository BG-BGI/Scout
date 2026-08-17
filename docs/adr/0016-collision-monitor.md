# ADR-0016: nav2_collision_monitor as the last-hop cmd_vel safety stage

Status: accepted (code) · Date: 2026-08-15 · On-robot verification pending

## Context

Until now nothing between twist_mux and the RoboClaw checked commands against
the world. nav2's costmaps only constrain nav goals; joystick, webui pad,
tricks, follow_me and skills motion went to the wheels unchecked, and the
documented hazard class — "goal failed ≠ robot stops", latched goals
surviving their client, behaviors running concurrently after an abort — all
end in exactly such unchecked commands. There is no hardware e-stop.

## Decision

Insert `nav2_collision_monitor` (in the nav2 1.1.20 apt image) between the
mux and the driver, in the `robot` service:

```
sources -> twist_mux -> /cmd_vel_out -> collision_monitor -> /cmd_vel_safe -> roboclaw_driver
```

- Config `scout/config/collision_monitor.yaml`; profile-aware via
  `merged_params` (ADR-0010) — `overlays/tight_tunnel/collision_monitor.yaml`
  shrinks both polygons to the footprint because pipe walls inside the
  default side margins would read as a permanent stop.
- Two boxes in `base_link` off the measured footprint (±0.169 × ±0.167):
  **stop** ±0.24 × ±0.21 (≈ one 11.7 Hz scan of travel at 0.6 m/s + coast),
  **slowdown** ±0.45 × ±0.32 at ratio 0.4. Source: `/scan` only for now (the
  under-lidar depth band can be added as a `pointcloud` source later).
- 1.1.20 semantics verified against upstream source: the threshold param is
  **`max_points`** (later distros renamed it `min_points` — do not copy
  newer docs), polygons declare `action_type` stop/slowdown/approach.
- It is a lifecycle node: a dedicated `lifecycle_manager_safety`
  (`autostart: true, node_names: [collision_monitor]`) activates it. Both are
  fail-fast tier (ADR-0015) — with either dead the driver hears nothing.

**Relationship to estop (ADR-0001):** independent layers. The e-stop is an
operator *intent* upstream in the mux (priority-255 lock + active brake); the
collision monitor is a world *reflex* downstream of every arbitration
decision. Neither replaces the other; a zero Twist passes the CM unchanged.

**Fail-safe direction:** `source_timeout: 2.0` means a scan gap >2 s (e.g.
the rplidar respawn window) invalidates the source and motion stops until the
lidar returns. Deliberate: blind ≠ drivable.

**Rejected alternative:** wiring teleop through nav2's AssistedTeleop action
— covers only sources that opt in, needs action plumbing in every client, and
is superseded by guarding the single choke point instead.

## Consequences

- Every motion source gains stop/slowdown protection with zero client
  changes; the deadman contract is unchanged (CM republishes per input).
- The driver's input topic is now `/cmd_vel_safe`
  (`robot_profile.topic_cmd_vel_safe`); the M3 on-blocks checklist gains a
  hop (`/cmd_vel_out` → mux pub + CM sub; `/cmd_vel_safe` → CM pub + driver
  sub).
- follow_me's close approach: the slowdown box (0.45 ahead) trims speed near
  the target before the standoff — safer, slightly slower docking. If it ever
  fights the standoff, shrink the stop box, not the source list.
- Latency cost is one extra intra-host hop at cmd rate — negligible against
  the 200 ms deadman.
- On-robot verify (remaining-plan §9.1): on blocks — obstacle in the slowdown
  box trims `/cmd_vel_safe`, in the stop box zeroes it, `polygon_stop` /
  `polygon_slowdown` render in Foxglove; then a floor pass with every source.

## Addendum (2026-08-17, first on-hardware verify): direction-blind stop lockout

Stop-zone triggering and release were verified working correctly on hardware.
But discovered the same session: **a plain `polygon` STOP zone gives no way
to drive out of it.** Confirmed against upstream source
(`nav2_collision_monitor/src/polygon.cpp`): `getPointsInside`/
`isTriggeredInternal` take only a geometric point count — never the
commanded Twist's direction. So once triggered on a static obstacle, CM
zeroes `/cmd_vel_safe` for EVERY commanded direction, including a reverse
command meant to back away, and the robot is stuck until the obstacle
physically leaves the box. Nav2's `velocity_polygon` shape type exists to
solve exactly this (direction-dependent zone) but isn't adopted here — would
need re-verification and is deferred.

**Immediate fix: a bounded, logged bypass**, `scout/scout/collision_bypass.py`.
`/collision_monitor/bypass_engage` PAUSEs `collision_monitor` via
`lifecycle_manager_safety`'s `manage_nodes` service (`ManageLifecycleNodes`,
command values STARTUP=0/PAUSE=1/RESUME=2/RESET=3/SHUTDOWN=4 — verified
against the Humble `.srv`) — the manager's own sanctioned pause/resume
surface, not a raw lifecycle `change_state` call (which would fight the
manager's bond-failure detection: the manager owns the bond to
collision_monitor and PAUSE/RESUME correctly tears it down and recreates it,
where an external deactivate would look like an unexpected failure).
`/collision_monitor/bypass_release` RESUMEs it. Auto-releases after
`bypass_max_duration_s` (default 30 s — enough to back away, not a general
disable) and logs a WARN every ~5 s while active. `/collision_monitor/bypassed`
(latched Bool) is the one source of truth for "is the safety stage off" —
webui/skills should surface it prominently, not bury it.

Verify (Pi): trigger a stop lockout, confirm `bypass_engage` frees motion,
confirm auto-release re-engages the stop after 30 s with no operator action,
confirm `bypass_release` re-engages immediately when called.
