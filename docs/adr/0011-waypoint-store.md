# ADR-0011: One JSON waypoint/route store shared by patrol + skills

Status: proposed · Date: 2026-08-15 (not yet implemented)

## Context

Two systems are both called "patrol" and write the same `./maps` directory but
cannot see each other: the ROS `patrol_capture` uses indexed waypoints in
`patrol_route.yaml` (teleop-marked, sequential NavigateToPose, photos), while
scout-skills uses *named* waypoints in `waypoints.json` (save_waypoint +
AprilTag auto-refresh, NavigateThroughPoses). AprilTag sightings silently mutate
the skills set, invisible to the webui.

## Decision

One store, `maps/waypoints.json` schema v2: named `waypoints` (x/y/yaw/saved/
`source` ∈ operator|tag|mark|coverage) + `routes` (ordered lists of names or
inline poses). `source` makes the tag auto-refresh visible in the file. Share
the **schema, not code** with scout-skills (its container has no scout package):
its loaders grow ~15 lines of v2 wrap/unwrap + legacy tolerance; the contract is
pinned by `scout.core.waypoints` + test fixtures. patrol_capture resolves route
names at start, so a tag-refreshed waypoint is picked up automatically. Both
writers use atomic replace + re-read-before-mutate; last-writer-wins is accepted
on a one-operator robot. The two route topics keep distinct roles (they are not
duplicates): `/patrol_route` (Path) = webui display; `/route_poses` (PoseArray)
= the link_watchdog re-dispatch mirror.

## Consequences

- `maps/patrol_route.yaml` retires; a migration script converts it on the Pi.
- `/patrol/clear` clears the route + its `mark-*` waypoints only (a semantic
  change to flag).
