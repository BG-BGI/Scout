# ADR-0028: amcl + map_server replace slam_toolbox's localization mode

Status: accepted · Date: 2026-08-27

## Context

`localization_slam_toolbox_node` scan-matches only locally: a wrong start pose
locks in silently, and a small `map→odom` correction cannot distinguish
"genuinely near the seed" from "stuck at a bad guess" (CLAUDE.md trap; the
boot-relocalization work exists because of it). It also reports no usable
confidence, and its `serialize_map` no-ops while reporting SUCCESS.
tag_relocalizer already seeds via `/initialpose`, which is exactly amcl's
native interface.

## Decision

`mode:=localization` in `slam.launch.py` now brings up **nav2_amcl +
nav2_map_server + a lifecycle manager** (`scout/config/amcl.yaml`) instead of
any slam_toolbox executable. Mapping (`new`) and `continue` are untouched —
slam_toolbox remains the only mapper; amcl cannot build maps.

- Ownership is unchanged: the compose `slam` service still owns `/map`
  (map_server, latched) and `map→odom` (amcl).
- Localization loads the **grid pair** `<map>.yaml`/`.pgm`, not the posegraph.
  The webui Save Map button now writes both formats (`serialize_map` then
  `save_map`); the launch file refuses to start localization without the grid.
- `initial_pose.*` comes from the site's `map_start_pose`; tag_relocalizer's
  `/initialpose` re-centres the particle cloud on the first registered-tag
  sighting. `reinitialize_global_localization` exists for a true lost-robot
  reset.
- Lifecycle bonds are disabled (`bond_timeout: 0.0`) — the boot clock step
  kills nav2 bonds silently (see memory/CLAUDE.md).
- Site `auto` policy still never resolves to localization: with amcl running,
  slam_toolbox is absent entirely, so nothing can be saved or extended there.

## Consequences

- Localization confidence is now real: `/amcl_pose` covariance grows when
  lost instead of a scan-matcher silently holding a wrong lock.
- Sites mapped before this ADR have only `.posegraph` — re-save from a
  `continue` session to get the grid pair before pinning localization mode.
- Motion-model alphas in `amcl.yaml` encode the drivetrain: gyro-fused yaw →
  low rotation noise; pivot walk → elevated `alpha4`. Re-tune after tire
  pressure changes.
- ADR-0003 (mode is the executable) still governs the two slam_toolbox modes.
