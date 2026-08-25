# Confined-space inspection pipeline (companion)

One reviewable artifact per run: **.mcap** (visual record — Foxglove opens it
directly) + **cloud.ply** (as-built 3D) + **anomalies.json** (auto-flagged
frames). Everything runs on the companion; the Pi pays nothing.

## Run flow

1. **Drive the pass** — `explore_for(minutes)` (frontier, tuned for pipes) or
   a patrol_capture route. No perception stops; the recorder rides along.
2. **Recording is automatic.** `inspection_recorder` watches the bridged
   `/patrol_status` and `/explore/resume` and brackets the .mcap to the run.
   Manual runs: `ros2 service call /inspection/start std_srvs/srv/Trigger`
   (local graph), same for `/inspection/stop`. Output:
   `companion/captures/inspection/<site>/<UTC>/` (site = the active
   location site, ADR-0023; flat `<UTC>/` when no site is set).
3. **Post-run, automatic:** `rtabmap-export` drops `cloud.ply` beside the bag
   (best-effort; the .mcap is the primary artifact).
4. **Anomaly pass, on demand:**
   `docker compose run --rm captioner /captures/inspection/<site>/<UTC>`
   → Florence-2-base captions a frame every 2 s, keyword-flags water / debris /
   blockage / damage / corrosion, writes `anomalies.json` with timestamps +
   odom xy. Jump to flagged `t` values on the Foxglove timeline.
5. **Review:** drag the .mcap into Foxglove desktop; import the layout
   `companion/foxglove/inspection-review.json` (3D cloud+scan, color, depth,
   registry, speed plot).

## Recorded topics

color/compressed, aligned depth/compressedDepth, camera_info, `/scan`,
`/odom`, `/tf`, `/tf_static`, `/world/objects`, `/world/registry`,
`/rtabmap/cloud_map`. Edit `RECORD_TOPICS` in `inspection/recorder.py`.

## Constraints & notes

- **Splits are safe here.** Bags split at 600 s (`split_duration_s`), unlike
  the Pi bag path where splitting is banned — that ban is a rosbag2 *playback*
  bug (ros2/rosbag2#966) and Foxglove reads MCAP directly.
- Bridge allowlists carry `/patrol_status` + `/explore/resume` (read-only
  status; the inbound-control surface is unchanged, ADR-0001).
- Captioner image bakes the model weights — fully offline at run time. It has
  no ROS; it decodes the .mcap with `mcap-ros2-support`.
- Disk: budget ~0.5–1 GB per 10 min of compressed color+depth. `captures/` is
  gitignored.
- Data Platform tier (Foxlet auto-upload + Events API) is deliberately not
  wired — decide after local review proves out; see the plan doc.
