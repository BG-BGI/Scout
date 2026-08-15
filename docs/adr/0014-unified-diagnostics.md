# ADR-0014: Unified health on /diagnostics via one aggregator node

Status: accepted · Date: 2026-08-15

## Context

The robot's health is scattered: battery_monitor publishes `/battery`
(BatteryState), tilt_monitor publishes `/tilt_alarm` (Bool), and roboclaw_driver
buries the pack + link state in a `/roboclaw_status` JSON String. No single
surface — neither Foxglove's Diagnostics panel nor the webui — can answer "is the
robot OK", and a *silent* subsystem (dead driver, crashed monitor) shows up only
as an absent topic that nobody is watching. `diagnostic_msgs/DiagnosticArray` on
`/diagnostics` is the ROS-standard answer and Foxglove renders it natively.

## Decision

Add one aggregator node, `health_monitor`, that subscribes to the existing health
topics and republishes them as a `DiagnosticArray` on `/diagnostics` at 1 Hz,
with an overall roll-up as the first status. It does **not** pull in the
`diagnostic_aggregator` package (no `/diagnostics_agg` tree, no extra apt dep /
image rebuild) — the fan-in is three topics and a single node is simpler.

The OK/WARN/ERROR/STALE decisions live in pure `scout.core.health` (injected
thresholds, no ROS import, 1:1 tested — ADR-0012/0013). Every subsystem is STALE
until its topic delivers and STALE again if it stops, so a dead publisher is a
red item, not a missing topic. Subsystems v1: battery (resting-voltage ladder,
warn/critical from robot_profile), tilt (the abort latch), drivetrain
(`/roboclaw_status` liveness — any parseable, recent message = serial link up +
driver alive). Battery warn/critical volts stay in `robot_profile.yaml` (SSOT
with battery_monitor/led_status); staleness timeouts are per-node params.

## Consequences

- One topic to watch; Foxglove Diagnostics panel + a webui health strip both read
  it. Silent-failure detection comes for free from the staleness gate.
- `diagnostic_msgs` must be present in `scout:latest` (it is, via nav2 /
  robot_localization) — verify with `ros2 interface show diagnostic_msgs/msg/DiagnosticArray`.
- Drivetrain temps and RoboClaw error flags are deliberately **not** in v1: the
  `/roboclaw_status` JSON schema beyond `main_battery`/`m1_speed`/`m2_speed` is
  unconfirmed off-hardware. Add them once the keys are read on the robot; do not
  guess field names.
- `scout.core.health` redeclares the DiagnosticStatus byte values (0/1/2/3) to
  stay ROS-free; `health_monitor` asserts they still match the real message at
  startup, so an upstream renumber fails loudly instead of silently miscolouring.
