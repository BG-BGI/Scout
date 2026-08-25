# ADR-0008: Deflated tires are the operating condition

Status: accepted · Date: 2026-08-14

## Context

All four tires were deliberately deflated as a traction fix. This is the
operating condition, not a fault. The earlier flat-front-left story and its
2.5 rad/s pivot stall floor are retired.

## Decision

Current measurements supersede everything from the inflated era. Notably:
`wheel_radius` 0.0780 (re-verified); pivots scrub enormously but nothing stalls;
the 2.5 rad/s figure is now a *walk minimizer* where position matters, NOT a
stall clamp. The hard pivot clamps were removed from joystick_teleop,
trick_player, and the webui (the webui copy lingered as a bug until 2026-08-15).

## Consequences

- Any code enforcing a ≥2.5 rad/s pivot floor is wrong — remove it.
- Tire-coupled params (min_vx, pivot walk, scrub asymmetry) all trace here.
- Re-run the tape test after any pressure change. Numbers: CLAUDE.md "Tire
  state" and "Pivot performance".
