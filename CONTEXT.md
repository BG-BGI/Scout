# Scout — domain context

Orientation for the software architecture. **CLAUDE.md** stays the hardware +
tuning log (measurements, PID, geometry); **docs/adr/** records the decisions.
This file names the concepts and maps the running system.

## Glossary

- **deadman** — the RoboClaw stops the motors if it gets no valid packet for
  `max_seconds_uncommanded_travel` (0.2 s). Idle mode is Free Wheeling, so this
  is a *coast*, not a brake. Every cmd_vel producer must stream ≥5 Hz.
- **zero-burst / STOP_GRACE** — a producer stops by publishing ~0.3 s of zero
  Twists (a commanded stop) then going silent (handing cmd_vel to others).
  Owned by `CmdVelSource`.
- **cmd_vel arbiter (twist_mux)** — every motion source publishes its own
  `/cmd_vel_*`; twist_mux forwards the highest-priority fresh one to
  `/cmd_vel_out`, which the driver drives. See ADR-0001.
- **software e-stop** — the `/estop` twist_mux lock + an active-brake burst on
  `/cmd_vel_stop`. Latching; fail-safe (a dead estop node = locked). No hardware
  e-stop exists. See ADR-0001.
- **velocity-loop floor** — ~0.05 m/s linear / ~0.35 rad/s pivot; below it the
  encoders quantization-stall. Advisory, not a clamp.
- **scrub floor / recommended pivot (2.5 rad/s)** — a skid-steer pivot walks;
  faster pivots walk less. A walk minimizer where position matters, NOT a stall
  clamp (the old flat-tire clamp is retired — ADR-0008).
- **under-lidar band (0.05–0.25 m)** — the height slice below the lidar's 24 cm
  plane where the D455 depth cloud catches chair bases / shoes / thresholds.
- **clutter** — persistent, map-frame memory of under-lidar obstacles
  (`clutter_mapper`), vs the live per-scan obstacle gate inside `follow_me`.
- **mover rejection / dwell / confirm window** — a grid cell only counts as a
  real obstacle after being seen ≥N times across a time span; a walking foot
  sweeps a cell too fast to confirm, so movers never stick.
- **see-through clearing** — a marked cell is cleared when a live ray reaches
  past it (the camera saw through where the mark was → it moved).
- **waypoint / route / patrol run / mark** — a named map pose; an ordered list
  of them; one execution of a route with photo capture; the act of recording
  the current pose as a waypoint. See ADR-0011.
- **coverage box** — a polygon dragged on the web map; planned into a serpentine
  route (`scout.core.coverage`).
- **zone (keepout / speed)** — a named polygon in `maps/zones.json` (per map,
  type + speed value) rasterized to a nav2 costmap-filter mask PGM by
  `zone_manager`. The JSON is the source of truth; the PGM is a derived
  artifact. See ADR-0019.
- **standoff / seek / reacquire** — follow_me's target distance; its lost-target
  pursuit to the loss anchor; its motion-gated re-lock after occlusion.
- **tag registry vs detection coverage** — the AprilTag *meaning* (names, roles,
  home) lives in scout-skills' sqlite; *which family/size is detected* lives in
  `apriltag.yaml`. See ADR-0006.
- **profile (default / tight_tunnel)** — a named set of nav2/slam/realsense
  parameter deltas for a scenario. See ADR-0010.
- **robot profile (robot_profile.yaml)** — the cross-surface SSOT for velocity
  caps, floors, rates, topic names, LED modes, status names, thresholds. Read by
  the ROS nodes, scout-skills, and the webui. Distinct from a *profile* above.
- **overlay volume** — the `ros_overlay_install` named volume holding the built
  workspace; seeds once from the image. See ADR-0005.

## System map (nodes → topics/services)

Core stack (`robot.launch.py`, compose `robot`):

| node | in | out | srv |
|---|---|---|---|
| roboclaw_driver | `/cmd_vel_out` | `/wheel_odom`, `/roboclaw_status`, `/joint_states` | — |
| twist_mux | `/cmd_vel_*` + `/estop` | `/cmd_vel_out` | — |
| estop | — | `/estop`, `/cmd_vel_stop` | `/estop/engage`,`/estop/release` |
| battery_monitor | `/roboclaw_status` | `/battery` | — |
| gyro_calibrator | `/camera/camera/imu` | `/imu/data` | — |
| ekf_filter_node | `/wheel_odom`,`/imu/data` | `/odom` (+ odom→base_link TF) | — |
| led_node | — | (APA102 SPI) | `/set_led_mode` |
| led_status | `/battery`,`/trick_status`,`/follow_status`,`/estop`,`/connected_clients` | → `/set_led_mode` | `/set_user_led` |
| joystick_teleop | joydev, `/follow_status` | `/cmd_vel_joy` | — |
| trick_player | `/battery` | `/cmd_vel_trick`,`/trick_status` | `/play_trick`,`/stop_trick` |
| follow_me | `/scan`, depth cloud | `/cmd_vel_follow`,`/follow_status` | `/follow_me/start`,`/stop` |
| clutter_mapper | depth cloud, TF | `/clutter_map`,`/clutter_points` | `/clutter/save`,`/clear` |
| patrol_capture | color, `/battery`,`/map`,`/coverage_box` | `/cmd_vel`(via nav2),`/patrol_status`,`/patrol_route` | `/patrol/{mark,clear,start,stop}` |
| link_watchdog | `/goal_pose`,`/route_poses`, action status | `/goal_pose`, cancels | — |
| tilt_monitor | `/imu/data` | `/tilt_alarm`,`/explore/resume` | — |
| apriltag | color | `/detections` + tag TF | — |
| nav_manager | both actions' status+feedback | `/nav_state` (latched), `/explore/resume` | `/nav/cancel` |
| bag_recorder | — | `/record/active`,`/record/path` (latched) | `/record/{start,stop}` |
| zone_manager | `/zone_cmd` | `/zones` (latched), mask PGMs | — |
| collision_polygon_manager | `/cmd_vel_out` | `/collision_monitor/{bypassed,zone_mode}` (latched) | `bypass_{engage,release}` |

Other stacks: `slam` (slam_toolbox → `/map`, map→odom), `nav2`
(navigation_launch.py → `/cmd_vel` at lowest mux priority), `foxglove_bridge`,
`rosbridge`, `webui`, `ros_mcp`, `scout_skills` (MCP over rosbridge :9001).

## Files on the Pi (bind-mounted `./` into the container)

All per-location state lives in `sites/<name>/` behind the `sites/active`
symlink (ADR-0023); switch sites from the webui Site panel. Per site:

- `sites/<name>/site.json` — display name, default_map, slam_mode policy.
- `sites/<name>/maps/waypoints.json` — named waypoints + routes (ADR-0011).
- `sites/<name>/maps/tags.db` — AprilTag registry (sqlite).
- `sites/<name>/maps/*.posegraph`,`*.data` — slam_toolbox serialized maps.
- `sites/<name>/maps/clutter.npz` — persistent clutter grid (only when the
  site has a default_map).
- `sites/<name>/maps/zones.json` — keepout/speed zone polygons (ADR-0019).
  The `zone_{keepout,speed}.pgm/.yaml` next to it are derived filter masks.
- `sites/<name>/captures/<runstamp>/` — patrol photos + manifest.
- `sites/<name>/captures/bags/<UTC>/` — rosbags from bag_recorder (ADR-0017).

All gitignored; migrate a pre-sites checkout once with
`python3 scripts/migrate_sites.py`.

## Status wire formats (std_msgs/String, split on `|`)

- `/trick_status`: `idle` | `name|#RRGGBB|mode`
- `/follow_status`: `idle` | `searching` | `seeking` | `locked|dist|deg` | `blocked`
- `/patrol_status`: `idle|<n>` | `<state>|<n>|<i>/<n>` | `plan|<text>`
- `/nav_state`: `idle` | `<status_name>|<dist 2dp or empty>|<recoveries>` (ADR-0018)
- `/zone_cmd` (command, grammar in `core.zones`): `add|<type>|<pct or empty>|x,y;…`
  | `delete|<name>` | `clear|` — `/zones` replies with the store's JSON schema

Kept as strings deliberately (ADR-0012); consumers on both sides of the
rosbridge boundary parse them, so the formats are frozen by tests —
`scout.core.status` owns the grammar and `scout/test/test_status.py` pins the
exact strings.

## Conventions (machine-enforced — ADR-0013)

`ruff check .` + `cd scout && pytest` is the definition of done; CI runs both
off-ROS. Repo-specific rules are structural tests (SC1–SC10 in
`scout/test/test_conventions.py`, `test_profile_constants.py`,
`test_status.py`): run_node-only mains, sensor QoS on sensor topics, TF via
node_util, Twist publishers only in cmd_vel_source/estop, no hand-rolled
quaternions, one owner for the bind-mount path, core modules adopted+tested,
profile values never re-hardcoded, wire formats frozen, deliberate copies
synced. Failure messages state the fix; waivers are reasoned `# noqa` /
`ALLOW` entries / `profile-exempt:` comments, all reviewed as code.
