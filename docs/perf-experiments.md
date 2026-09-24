# Performance A/B experiment runbook

Phase-2 protocol for the CPU-starvation investigation (handoff 2026-09-24;
ADR-0035/0036 are the code-fix half). Every run wraps itself in
`scripts/perf_capture.py` — the metrics columns below name its files
(`host.csv`, `containers.csv`, `threads.csv`, `ros.jsonl`; schema in the
script docstring). Compare arms only within one capture pair; never across a
deploy.

## Constants (every experiment, both arms)

| Constant | Concretely |
|---|---|
| Code | One `SCOUT_TAG` SHA for both arms (`meta.json` proves it) |
| Site & map | Same named site under `sites/`; slam in **localization** mode except experiment 3 (mapping grows state by design) |
| Mission | One saved waypoint patrol route via scout-skills (`patrol`/`go_through`) — commit the waypoint names here once chosen. Exploration arms use `explore_for(N)` in the same physically reset area |
| Profile | `SCOUT_PROFILE=default` in `.env`, recorded |
| Clients | Foxglove disconnected, webui tab closed, no MCP sessions, Dozzle off — except where the experiment varies them. Audit trail: rosbridge/foxglove_bridge logs |
| Companion | Up and bridged in both arms (`meta.json` records `COMPANION_HOST`) |
| Battery | Start ≥ an agreed resting-voltage floor (profile battery ladder) — QPPS and DVFS both track pack state |
| Thermal | Start below 60 °C with throttle-latch bits cleared (reboot clears; `vcgencmd get_throttled` bits 16–19) |
| Capture | `sudo ./scripts/perf_capture.py --minutes <run+2>` started before the mission, stopped after |

## Safety (every run that moves the robot)

Per CLAUDE.md, restated because this doc will be followed under time
pressure:

- **Explicit operator confirmation per run** — direction, speed, duration,
  space needed. Permission never carries over between runs or surfaces.
- Stop paths, in order: `/nav/cancel` (nav_manager — stops patrol, pauses
  explore, cancels both bt_navigator actions), webui STOP / E-Stop,
  `docker stop <container>` (**coast, not brake** — deadman free-wheels in
  200 ms). Interrupting the agent's/operator's shell does NOT stop the robot.
- Abort criteria are per-experiment below; any hard-throttle bit
  (`get_throttled` bit 0–3), temp > 80 °C, or `/nav_state` stuck > 60 s
  aborts everything.

## Experiment template

Each experiment records: Question / Setup / Held constant / Variable /
Procedure / Metrics / Abort criteria. Keep raw run dirs; report measured
deltas separately from expectations.

## 1 — Observability profile off vs on

- **Question:** does the (pre-ADR-0035 or fixed) observability stack
  measurably tax the control path?
- **Variable:** arm B adds `docker compose --profile observability up -d`.
- **Procedure:** fixed patrol mission per arm, same start pose.
- **Metrics:** `containers.csv` `throttled_usec_d` on robot/nav2/slam;
  `host.csv` `psi_cpu_full_avg10`, `ctxt_per_s`; `ros.jsonl` scan/odom
  `gap_max_ms`; mission wall time.
- **Abort:** general criteria above.

## 2 — Viewers closed vs open

- **Variable:** arm B = Foxglove connected (standard `Foxglove.json` layout)
  + webui tab open.
- **Metrics:** foxglove_bridge/rosbridge rows in `containers.csv` (usage +
  throttling — both are 0.3-capped), robot-container deltas, scan/TF ages in
  `ros.jsonl`.

## 3 — Fresh vs mature map / mission-duration growth

- **Setup:** slam in `new`/`continue` mode; three consecutive
  `explore_for(10)` windows **without restart**. No B arm — time is the
  variable.
- **Metrics:** trend per window: slam row in `containers.csv`, slam thread
  `run_delay_ms_d` in `threads.csv` (Ceres solve stalls), map dimensions
  from `/map` (webui or bag), `memory.current`.

## 4 — Exporter fix regression (after ADR-0035 deploys)

- Re-run experiment 1 identically; compare against its run dirs. Completion
  evidence for the P0: no queue growth under a deliberately slowed collector
  (stop rosbridge mid-run — `scout_poll_last_success_timestamp_seconds` must
  go stale, gauges must not freeze) and quantified overhead delta.

## 5 — Collision-zone coalescing (after ADR-0036 deploys)

- **Procedure (stationary + one supervised motion run):** drive a figure
  that churns zones (turn/reverse transitions); mid-idle,
  `docker compose restart nav2` (collision_monitor lives in `robot`'s
  launch — restart the CM's lifecycle instead via its manager if only the CM
  is the target); watch `/collision_monitor/zone_sync` dip and recover.
- **Metrics:** `/stop_reason` timeline in a bag; zone_sync recovery time;
  **gate: no `/cmd_vel` gap > 200 ms during zone churn** (`ros.jsonl`
  `cmd_vel_gap_max_ms` during commanded motion only — idle silence is
  normal, never a fault).

## 6 — Optional nodes out of the control budget

- **Variable:** one node at a time disabled via launch arg/trial branch:
  led_status, health_monitor, traction_monitor idle costs. (uhf/flipper are
  USB-bound to the robot — excluded; cliff_detector is safety — never.)
- **Metrics:** robot row in `containers.csv`; ekf/driver thread
  `run_delay_ms_d`.
- Fixed mission per arm; revalidate launch fail-fast behavior after any
  permanent move.

## 7 — SLAM knobs, one at a time

- **Variable (one per arm, `scout/config/slam.yaml`):**
  `minimum_travel_distance/heading` 0.3→0.5, `map_update_interval` 2.0→5.0,
  `scan_buffer_size`, `loop_search_maximum_distance`. The file's own
  comments name the first two as the sanctioned relief valves.
- **Quality gate:** `map→odom` correction stays ≤ the ~0.30 m / 2.1° per
  ~17 m baseline on the standard route, plus operator scan-vs-map overlay
  check in Foxglove. A knob that saves CPU but degrades localization loses.

## Reporting

Per experiment: run-dir pair, the 3–5 numbers that moved, and whether the
change is a candidate for a permanent commit (which then goes through the
normal ADR/PR path). Keep the baseline run dirs — they are the reference
for every later regression check.
