# ADR-0007: Battery SoC from resting voltage only; threshold ladder

Status: accepted · Date: 2026-07-30

## Context

Nothing on this pack is trustworthy for charge but voltage: the RoboClaw's
current telemetry is fiction below ~20% duty, the 5 V buck taps upstream of the
RoboClaw, and there is no coulomb counter or BMS. Voltage sags hard under load.

## Decision

Estimate charge from a 5s li-ion resting-voltage curve, sampled **only at rest**
(both encoder speeds near zero for `rest_seconds`) and **medianed** per period
(the pack quantizes to 0.1 V — worth ~3% through the flat middle — so a single
reading would dither the estimate more than discharge moves it). The pure curve
+ replayable estimator live in `scout.core.battery`.

The threshold ladder is **critical 16.5 V < activity-floor 17.0 V < warn 17.5 V**
and lives in `robot_profile.yaml` (the SSOT — webui badge, led_status, and the
trick/patrol gating all read it), *not* in core.battery, so every surface
agrees. These are loaded readings, so they fire early under drive current —
the useful direction.

## Consequences

- `percentage` is NaN until the robot has been still a few seconds.
- core.battery holds curve logic; the profile holds the thresholds — one source
  each, no overlap.
