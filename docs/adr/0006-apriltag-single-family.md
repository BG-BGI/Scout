# ADR-0006: One AprilTag family (tag36h11); registry ≠ detection coverage

Status: accepted · Date: 2026-08-15

## Context

An all-families fan-out (8 nodes on a throttled stream, 2026-08-14) OOM-killed
the Pi: the big-codebook families (Custom48h12/Circle49h12/Standard52h13) build
4–6 GB quick-decode tables each. Separately, the doghouse "home" feature was
dead: the printed tag (`tags/doghouse.svg`) is tag36h11/160 mm but the detector
was configured Standard52h13/0.12 m — the tag was invisible while the watcher
reported healthy.

## Decision

Detect **one** family. `apriltag.yaml` is `tag36h11` / `size 0.16`, matching the
checked-in print and scout-skills `register_tag` defaults; tag36h11's small
codebook is also the cheap-RAM choice. Detection COVERAGE (family/size, needs a
robot-service restart) lives in `apriltag.yaml`; tag MEANING (names, roles,
home, waypoints) lives in scout-skills' sqlite registry (`maps/tags.db`).

## Consequences

- Revisit multi-family only with per-family memory budgets and a camera_info
  relay (image_transport derives camera_info from the image topic namespace).
