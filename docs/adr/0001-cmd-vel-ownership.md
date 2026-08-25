# ADR-0001: cmd_vel arbitration via twist_mux + the software e-stop

Status: accepted · Date: 2026-08-15

## Context

Six surfaces published `/cmd_vel` (joystick, trick, follow, webui, scout-skills,
Foxglove) and four claimed in a docstring to be "the sole writer". Safety rested
entirely on every author having read every other author's convention. There was
no arbiter and no real e-stop: the skills `nav_cancel` "software e-stop" left a
running patrol/follow/trick driving, and there was no way to lock the drivetrain.

## Decision

`twist_mux` (apt) is the arbiter. Every source publishes its own `/cmd_vel_*`
topic; the mux forwards the highest-priority *fresh* input to `/cmd_vel_out`,
which `roboclaw_driver` drives. Priority: stop(255) > joystick(100) > web(90) >
trick(60) > follow(50) > skills(40) > nav(10).

**Topology is inverted so nav2 needs no change**: nav2 keeps publishing plain
`/cmd_vel` (velocity_smoother *and* behavior_server's direct output both land
there) as the lowest-priority mux input, so upstream `navigation_launch.py` is
untouched and any legacy raw `/cmd_vel` publisher still drives, just below
everyone else.

The `CmdVelSource` module owns the per-source publish timer, the STOP_GRACE
zero-burst, cap clamping, and a staleness auto-idle (a dead caller loop can't
latch a live velocity). The **e-stop** is a `std_msgs/Bool /estop` twist_mux
lock published at 5 Hz by the `estop` node, plus an active-brake burst on
`/cmd_vel_stop` (priority 255, above the lock, so it passes while everything
else is locked out). The lock timeout is 1.0 s = fail-safe: a dead estop node
goes stale = locked.

## Consequences

- One seam decides what drives; the "sole writer" convention is gone.
- Per-source topic names live in `robot_profile.yaml`; one edit repoints all.
- The mux + driver share the `robot` container so the motion chain dies together.
- Verify on blocks before floor driving (endpoint count, preemption, `/estop`).
- The RoboClaw 200 ms deadman remains the coast backstop; the mux synthesizes no
  zeros — a source's own zero-burst is the commanded stop.
