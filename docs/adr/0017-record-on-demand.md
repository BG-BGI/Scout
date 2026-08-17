# ADR-0017: rosbag record-on-demand via one subprocess-owning node

Status: accepted · Date: 2026-08-17

## Context

All bench/calibration tooling was deleted on request (2026-07-30), so every new
measurement starts with "rebuild the instrument". rosbag2 is the ROS-native
permanent replacement — but `ros2 bag record` from a shell needs a container,
the right QoS overrides, and someone remembering to stop it. The 5b depth_grid
refactor also has a hard dependency on a recorded depth bag (its required
old-vs-new corridor-min diff), which made a first-class recorder the next
instrument to build (2026-08-17 grill).

## Decision

One node, `bag_recorder`, owns the `ros2 bag record` **subprocess** lifecycle:
`/record/start` + `/record/stop` (`std_srvs/Trigger`), latched `/record/active`
(Bool) + `/record/path` (String). Subprocess, not `rosbag2_py` in-process: the
Python API has documented threading caveats under a spinning executor and the
CLI is the tested path. Argv/path assembly is pure `scout.core.recording`
(1:1 tested, SC7).

- **Bags land in `captures/bags/<UTC>/`** — own subtree, because
  `captures/<runstamp>/` is the patrol-photo namespace (CONTEXT.md). The
  existing `.:/ros_ws/src/` root bind already puts them on the host; no new
  mount.
- **Topic selection is the `topics` node parameter** (default = profile
  `record_topics`), set via `ros2 param set` before start — Trigger carries no
  payload and a custom .srv is against the ADR-0012 no-custom-interfaces
  stance. Read at spawn time only.
- **No bag splitting, ever:** `--max-bag-size`/`--max-bag-duration` produce
  bags that do not play back on Humble (only the last split plays —
  ros2/rosbag2#966). The runaway guard is instead a node-side auto-stop after
  profile `record_max_duration_s` (default 600 s) — SIGINT, so the bag
  finalizes playable.
- **`bag_qos_overrides.yaml` rides every recording:** gyro_calibrator publishes
  `/imu/data` best-effort, and a reliable-by-default recorder subscription
  receives NOTHING (the documented EKF QoS trap, now on the recording path).
  Best-effort subscriptions are compatible with reliable publishers too, so the
  sensor topics are pinned best-effort wholesale.
- Surfaces: skills MCP tools (`start_recording`/`stop_recording`/
  `recording_status`) and a webui Recording panel lit from `/record/active`.

Snapshot mode (`--snapshot-mode` + the recorder's `~/snapshot` service —
rosbag2 #850/#844, in Humble) is a **follow-up commit** after the continuous
path Pi-verifies; continuous is what 5b needs.

## Addendum (2026-08-17, first Pi verification)

**Discovered: the recorder subprocess must be a Discovery Server SUPER
CLIENT, not just this node.** `ros2 bag record <names>` isn't given each
topic's type on the CLI, so it resolves types (and matches publishers) by
enumerating the ROS graph — the exact operation Discovery Server v2 blinds
for a plain client (the same "near-empty graph" trap documented for
`ros2 topic list`/`ros2 doctor`). Without the fix, `/record/start` reports
`success: true`, a valid-looking `.db3` is created, and `/record/stop` looks
clean — but the bag has **zero messages and a sentinel int64-max
timestamp**, with no error anywhere. Confirmed by isolation: a manual
`ros2 bag record -o … /odom` as a plain client captured 0 messages over an
actively-publishing topic; the identical command with
`FASTRTPS_DEFAULT_PROFILES_FILE=super_client.xml` captured every message.
Fix: `bag_recorder._on_start` spawns the child with that env var set,
independent of whatever profile the node itself runs under.

## Consequences

- Any surface can capture a diagnosis bag with one call; the 5b bag is
  `ros2 param set /bag_recorder topics "[…, /camera/camera/depth/color/points,
  /tf, /tf_static, /odom]"` + start + a short operator-confirmed drive + stop.
- A forgotten recording costs at most `record_max_duration_s` of SD writes.
- Deliberate long captures must raise the profile value first — the guard is a
  policy, not a suggestion.
- Verify on the Pi: start → short operator-confirmed drive → stop;
  `ros2 bag info` lists topics + counts **including a nonzero `/imu/data`
  count** (the QoS-trap regression check); bag opens on the host.
