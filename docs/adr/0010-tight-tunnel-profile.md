# ADR-0010: Scenario profiles as parameter overlays

Status: accepted · Date: 2026-08-15 (overlay refactor planned)

## Context

The "tight tunnel" scenario shipped as full-file copies of nav2/slam/realsense
configs (~1,200 forked lines carrying ~23 real value changes). Worse, the copies
carried the *rationale* too, so they drifted — the tight-tunnel files assert
hardware facts the base files have since retracted.

## Decision

Base YAML + a small overlay of deltas, merged at launch (nav2 via a PyYAML
deep-merge in a thin `nav2.launch.py`, since RewrittenYaml cannot rewrite the
costmap `plugins` list that removes `depth_layer`; slam via stacked
`parameters=[base, overlay]`; realsense via the same deep-merge). A single
`profile` launch arg / `SCOUT_PROFILE` env selects it across every service, so
the switch is one line, not commented-out command edits. `default` returns the
base file path untouched (byte-identical). A launch-time guard fails if a
profile turns off the realsense pointcloud while nav2 still expects
`depth_layer` (ADR-0002 coupling).

## Consequences

- Rationale lives once (in the base config or here); overlays carry only deltas.
- Full param-dump equivalence is the acceptance test for the default profile.
