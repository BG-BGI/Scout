# ADR-0009: Link-loss cancel-and-stash nav policy

Status: accepted · Date: 2026-08-14

## Context

The robot drove into a WiFi dead zone with a NavigateToPose goal latched.
bt_navigator replans at 1 Hz and streams cmd_vel entirely on-robot, so losing
the network removed every software stop; the goal ran until a human pulled the
battery.

## Decision

`link_watchdog` probes the default gateway (TCP connect). Link down
`pause_after_s` (5 s) → cancel active nav goals and **stash** them; link back
within `forget_after_s` (120 s) → re-dispatch; longer → drop the stash, stay
parked. "Pause" = cancel-plus-stash because Nav2 has no native pause. Goals are
captured from the wire: `/goal_pose` (every NavigateToPose client) and
`/route_poses` (scout-skills' multi-point routes, published so the watchdog can
resend them).

## Consequences

- Known gap: a ROS `patrol_capture` run advances on cancel unless it treats an
  external cancel as a stop (fixed 2026-08-15 — see patrol_capture). A latched
  `/nav_paused` Bool to hold patrol while paused is a proposed follow-up.
