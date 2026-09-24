# ADR-0037: Cliff detection removed

Status: accepted · Date: 2026-09-24 · Supersedes the detector half of ADR-0024

## Context

Live incident (2026-09-24, first drive after ADR-0035/0036): a clicked map
goal aborted with `Failed to make progress`. The new stop-reason timeline
traced it: cliff_detector flagged "cliff in the stop corridor" every ~5 s,
its 5-point stop cluster tripped PolygonStopFront/Turn, the CM zeroed the
command, and DWB's progress checker aborted the goal. The operator's bypass
moved the robot; the 30 s auto-release re-stopped it.

Root cause of the false positive: the detector projects depth through the
STATIC URDF camera transform — chassis pitch never enters TF (2D odom), so
driving over a bump makes the floor ahead read as below-floor
(`drop_base: 0.05` ≈ 2–3° of nose-up at 1.5–2 m). The phantom cells then
latch into a **300 s odom-frame memory** republished every frame, so one
bump blocks navigation for up to five minutes. Operator directive: remove
cliff detection entirely rather than gate it.

## Decision

Deleted: `scout/scout/cliff_detector.py`, `scout/scout/core/cliff.py`,
`scout/config/cliff.yaml`, `test_cliff.py`, `test_cliff_cm_coupling.py`, the
setup.py entry point, the CM `cliff` source (base + tight_tunnel overlay),
the stvl `cliff` sources in both nav2 costmaps, and health_monitor's cliff
row (+ `core.health.cliff_level`).

Kept: `_cliff_guard` in robot.launch.py — fail-louds if any profile's
collision_monitor ever lists a `cliff` source again, because with no
detector that source starves and `source_timeout` freezes autonomy
permanently (the exact failure ADR-0024's coupling guard existed for).

## Consequences

- **The stack has NO negative-obstacle safeguard.** Depth and lidar both see
  nothing below floor level; a down-stair reads as free space to the planner
  and the CM. Operating areas must exclude stairs/ledges, or cliff detection
  returns in a pitch-compensated form.
- If it returns, fix the mechanism first: gate mark-adding on chassis
  motion (gyro pitch-rate latch + settle dwell via `core.latch`, or live
  pitch from the tilt filter) and shorten `memory_s` — the 2026-09-24
  incident is the regression test.
- `/stop_reason` keeps working (cliff stops surfaced as `cm_stop:<zone>`;
  that lane simply can't fire from ledges any more).
