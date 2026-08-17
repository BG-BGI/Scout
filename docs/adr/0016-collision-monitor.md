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

**First fix attempt was WRONG, caught on hardware.** `bypass_engage` initially
PAUSEd `collision_monitor` via `lifecycle_manager_safety`'s `manage_nodes`
service — the timing worked exactly as designed (auto-release fired on
schedule), but the robot stayed stuck. Root cause, confirmed against
upstream source (`nav2_collision_monitor/src/collision_monitor_node.cpp`):
`on_deactivate()` deactivates the OUTPUT publisher while leaving the input
subscription alive, so a paused node doesn't pass `/cmd_vel_out` through
unfiltered — it silently swallows it, and `/cmd_vel_safe` goes dark. Same
"stuck" symptom as the original lockout, different mechanism, and it only
showed up when actually tested on the robot.

**Actual fix: toggle the polygon's `enabled` parameter live, node stays
ACTIVE.** nav2_collision_monitor declares `<PolygonName>.enabled` as a plain
ROS parameter per polygon with a `dynamicParametersCallback` that applies
immediately — no lifecycle transition (confirmed in `polygon.cpp`).
`bypass_engage` calls `collision_monitor`'s own `set_parameters` service to
set `PolygonStop.enabled=false`; `PolygonSlow` stays enabled throughout
(harmless — it only caps speed, and extra caution while backing out is a
feature). `bypass_release` sets it back to `true`. Same bounded contract as
before: `bypass_max_duration_s` (default 30 s) auto-releases, a WARN logs
every ~5 s while active, `/collision_monitor/bypassed` (latched Bool) is the
one status source webui/skills surface prominently.

Verified working correctly (bypass_engage frees motion, auto-release
re-arms the stop, bypass_release re-arms immediately).

## Addendum 2 (2026-08-17, same session): the stop zone is direction-blind, not just PAUSE-broken

Passing between two obstacles narrower than the default stop box's ~0.42 m
gap tripped a full stop even though the 0.334 m chassis physically fit and
was driving straight through, not toward, either obstacle. Root cause is the
same geometric-only check as the PAUSE bug above: `PolygonStop` is a single
STATIC box, symmetric on all four sides, with zero awareness of the commanded
Twist's direction (confirmed against `polygon.cpp` again — `getPointsInside`/
`isTriggeredInternal` take only a point count). nav2's native fix,
`velocity_polygon` (a shape that switches based on the commanded velocity),
merged into nav2 Iron (Feb 2024) — after Humble had already frozen for new
features. Confirmed **absent** from this image: zero `VelocityPolygon`
symbols in the installed `libnav2_collision_monitor_core.so`. Building it
from source would need a newer `nav2_util`/`nav2_costmap_2d` than Humble's
apt provides (unconfirmed compatibility) or a full distro upgrade (touches
every other from-source pin in this repo) — not worth it for one feature.

**Fix: reimplement the mechanism ourselves**, using the same live-parameter
path proven by the bypass. `collision_monitor.yaml` now declares TWO
mutually-exclusive stop polygons: `PolygonStopStraight` (±0.175 side —
footprint + ~1 cm, default-enabled) and `PolygonStopTurn` (±0.21 side — the
original margin, default-disabled). `scout/scout/collision_polygon_manager.py`
(renamed from `collision_bypass.py`, merged in — both features mutate the
same two `enabled` flags and would race as separate nodes) subscribes
`/cmd_vel_out` — the same commanded Twist collision_monitor reads, so it
reacts to what's about to be commanded rather than lagging behind measured
odometry — and toggles which polygon is enabled based on `|angular.z|`
(hysteresis: enter "turning" above `turn_enter_rad_s` (0.15), only return to
"straight" after staying below `turn_exit_rad_s` (0.05) for `turn_exit_dwell_s`
(0.3 s), avoiding flapping). Forward margin (~7 cm) is unchanged in both —
braking distance doesn't depend on whether the robot is turning. Only
`|angular.z|` matters: this is a skid-steer, never holonomic.
`/collision_monitor/zone_mode` (latched String) reports which is active.

One-tick latency is inherent (this node and collision_monitor both receive
the same `/cmd_vel_out` message via DDS fan-out with no ordering guarantee,
so a transition applies from the NEXT message onward) — at 20–50 Hz that's
≤50 ms, folded into the existing margin, not a new gap.

`tight_tunnel` overlay collapses both to the same tiny box: turning at all is
already unsafe in a passage that tight, so there's no wider "turn" allowance
to fall back to.

Verify (Pi): drive straight between two obstacles narrower than 0.42 m but
wider than the chassis — confirm no stop; command a real turn/pivot near an
obstacle — confirm the wide zone still catches it; watch `/collision_monitor/
zone_mode` transition during a mixed drive (straight → turn → straight).
