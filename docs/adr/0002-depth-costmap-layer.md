# ADR-0002: Under-lidar depth is its own costmap layer, never shared with lidar clearing

Status: accepted · Date: 2026-08-03

## Context

The lidar plane sits ~24 cm up, so chair bases, shoes and thresholds are
invisible to `/scan`. The D455 depth cloud catches them, but a single
mark-and-clear obstacle layer fed by both lidar and depth over-clears: a
doorway the lidar sees straight through erases depth marks that are still real.

## Decision

Depth marking is a separate local costmap layer (`depth_layer`): a mark-only
pass over the 0.05–0.22 m height band, and a clear-only pass over the full
cloud height so removing an object frees cells via floor/wall rays. The global
costmap stays lidar-only. `clutter_mapper` keeps the persistent, map-frame
memory (`/clutter_points` → the global ObstacleLayer); StaticLayer is rejected
for it because two static layers fight over the master grid size.

## Consequences

- Doorways stop over-clearing; low obstacles persist across the camera's narrow
  window.
- The realsense pointcloud and the nav2 `depth_layer` are coupled — a profile
  that turns one off must turn the other off (guarded at launch — ADR-0010).
- `min`/`max` height bands live in two places (node vs nav2); keep them in step.
