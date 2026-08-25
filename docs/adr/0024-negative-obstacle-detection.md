# ADR-0024: Negative obstacles — cliff_detector with an odom latch, marking-only STVL source, CM hard stop

Status: accepted · Date: 2026-08-24

## Context

Nothing in the stack could see a down-stair: the lidar plane is ~24 cm up, and
the STVL depth source (which replaced the ADR-0002 `depth_mark`/`depth_clear`
pair and the Python `clutter_mapper` on 2026-08-24 — recorded here since no
ADR captured that migration) marks only the 0.05–0.22 m band, so a ledge
produced no cost at all. The camera is deliberately level (AprilTag/detection
FOV), which puts the floor in view only from ~0.51 m ahead of `base_link` —
and once the robot is closer than that, the ledge leaves the FOV entirely.

Config-only fixes fail structurally. A negative-height STVL band marks the
tread *below* the drop (~0.25 m past the lip), and frustum decay erases the
marks while the camera stares into the returnless void. A negative-height
collision_monitor band sees points only ≥ ~0.5 m out — outside every stop
polygon — Humble's CM feeds every source to every polygon (a bigger cliff box
would stop 0.6 m from every wall via `/scan`), and CM is stateless, releasing
the stop the moment the ledge leaves view.

## Decision

A dedicated `cliff_detector` node (pure math in `scout.core.cliff`) finds
below-floor returns, projects each ray back to the z=0 floor plane so the mark
lands on the **lip** of the drop, votes hits into 5 cm cells, and **latches
the cells on an odom-frame grid** (300 s TTL) — the latch is the load-bearing
feature, covering both the <0.51 m blind zone and look-away decay. Two
outputs, both published on every processed cloud:

- `/cliff/points` (odom, z=0.12): the whole memory, feeding a **marking-only**
  `cliff` source on the existing `stvl_layer` in both costmaps — nothing may
  clear a ledge, and the ~5 Hz republish outruns every decay.
- `/cliff/stop_points` (base_link): a fixed 5-point cluster inside
  PolygonStopFront/Turn (outside StopRear, so BackUp recovery escapes) when a
  remembered cell is in the 0.6 m forward corridor, else an empty cloud —
  feeding a CM `pointcloud` source. On camera/TF/node death the node goes
  **silent**, so CM `source_timeout` (2 s) turns blind into stopped.

Only below-floor **returns** are used; missing-returns inference is deferred
(x4 decimation + spatial filter make holes routine — it would false-stop
constantly). `tight_tunnel` (depth off) skips the node and strips the CM
source; `robot.launch.py` fail-louds if that coupling breaks (mirror of
`nav2.launch.py`'s ADR-0002 guard).

## Consequences

Nav2 plans around remembered ledges and the CM stops autonomy ~0.6 m short of
one even if planning is wrong. Costs and residual gaps a future reader must
not re-litigate: a true zero-return void (glass-edged mezzanine) is NOT
detected; the camera is forward-only, so reversing toward a never-seen ledge
is unprotected; teleop bypasses the CM entirely (ADR-0016 — operator
decision); marks live in odom and die with the process. Supersedes ADR-0002's
layer mechanics (STVL carries the band now); its never-share-with-lidar-
clearing rationale still stands and is honored by the marking-only source.
