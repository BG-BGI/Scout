# ADR-0039: rosbridge handshake gate (replaces the single connection lock)

Status: accepted · Date: 2026-11-06 · Amends the rosbridge client in `docker/scout-skills`

## Context

`rosbridge.py` held one process-wide `asyncio.Lock` for the whole life of every
`RosBridge` context. Every tool call queued behind every other, and the AprilTag
watcher (every 2 s, on by default, ~1.2–5.5 s per scan) held the same lock, so
Magnus calls waited behind it. Worst-case idle `nav_status` is ~6.5 s before any
queueing. The lock existed for a real reason: a websocket handshake that lands
while rosbridge is mid-way through a ~1 MB camera frame starves past
`open_timeout`.

## Decision

- The lock becomes a **handshake gate**. Light calls take it only around the
  connect, so they run concurrently (handshakes still never interleave).
- `RosBridge(heavy=True)` holds the gate for its whole life — `get_map`,
  `camera_snapshot`, `detect_objects`, `detect_apriltags` (colour frame) and the
  map read in the site-switch path — so no handshake lands mid-frame. That
  preserves the original protection exactly where it was needed.
- `RosBridge(background=True)` (the tag watcher) never waits: it raises
  `RosBridgeBusy` when the gate is held or any foreground call is in flight, and
  the watcher skips that cycle.
- Handshake timeout is `ROSBRIDGE_OPEN_TIMEOUT` (default 5 s, was 10 s x 2).
- The watcher only rewrites `waypoints.json` when the tag standoff changed.
- `RosBridge.collect_multi` reads several topics in ONE shared window. `nav_status`
  (six sequential windows, ~6.5 s idle) and `_robot_pose` (1.5 s `/pose` then 1.0 s
  `/tf`) now each take one ~0.5-0.6 s window; pose parsing lives in `navpose.py`.

## Baseline (2026-11-06, deployed pre-change code, Mac -> Pi over Wi-Fi LAN)

`nav_status` median 7.7 s with the tag watcher on, 6.7 s off: the watcher is not the
dominant cost, the fixed windows are. `list_pois`/`list_waypoints` ~190 ms either way.
`get_map` 4.8 s (3.8 s watcher off). Re-measure after deploy.

## Not done

Persistent connection + topic cache (`RosLink`), per-tool deadlines, the other
fixed waits (`go_to` 0.7 s advertise settle, `_require_motion_idle`, `get_map`)
and the ros-mcp proxy hop remain open. The post-change numbers are unmeasured
(change not yet deployed).
