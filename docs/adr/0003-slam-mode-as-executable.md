# ADR-0003: slam mode is the executable, not a parameter

Status: accepted · Date: 2026-07-30

## Context

Every slam_toolbox tutorial sets a `mode:` parameter — but there is no
`declare_parameter("mode")` anywhere in slam_toolbox or karto. It is a dead
key. The real behavior switch is which executable runs, and several loading
params are tested with `PARAMETER_NOT_SET`, so a `false` reads as *set*.

## Decision

`slam.launch.py` selects behavior by **executable**: `async_slam_toolbox_node`
(mapping) vs `localization_slam_toolbox_node`. Map-loading params
(`map_file_name` + `map_start_pose`/`map_start_at_dock`) are built as a dict and
passed only when present — never as `false`. The launch file guards: unknown
mode, missing `.posegraph`, and malformed `map_start_pose` all refuse to start
(a missing map otherwise logs and comes up healthy on an empty graph).

## Consequences

- `slam.yaml` carries no `mode` key.
- `serialize_map` is a no-op in localization mode (and reports success anyway),
  so use `continue` to load-and-still-save. Clutter persistence is only sound
  under `continue`/`localization` (map frame stable) — see the 2026-08-12
  phantom-clutter incident in CLAUDE.md. Full mechanics: CLAUDE.md "SLAM".
