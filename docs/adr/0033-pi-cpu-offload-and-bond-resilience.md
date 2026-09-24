# ADR-0033: Pi CPU offload — stream tax removal, nav2 bond resilience

Status: accepted · Date: 2026-09-17

## Context

The Pi 5 is reported pegged at/near 100% CPU with nav2 appearing to crash
frequently. The last measured baseline ("nav2 ~39% of one core, whole stack
~1 core", CLAUDE.md) predates STVL, cliff_detector, apriltag, UHF, and the
companion zenoh link. Three mechanisms were identified from the tree (live
Pi unreachable at analysis time):

1. **Continuous compression tax.** `align_depth` resample and the
   `compressedDepth` PNG encoder are per-subscriber lazy — but the zenoh
   bridge holds a permanent subscription on both the moment the companion
   stack is up (which is `restart: unless-stopped`, i.e. always). Color JPEG
   likewise encodes at stream rate while the bridge subscribes. Estimated
   ~0.4–0.8 core continuous that did not exist in the baseline.
2. **Bond-timeout resets.** `lifecycle_manager_navigation` receives only its
   hardcoded launch dict (`navigation_launch.py` gives it no params file), so
   it runs the upstream ~4 s bond timeout. Under CPU starvation — or a
   mid-session clock step, which wait-clock-sane.sh only guards at boot —
   heartbeats arrive late and the manager force-deactivates all eight nodes.
   The container looks Up and healthy while nav2 is gone: reads exactly like
   "nav2 crashes". amcl.yaml already disables bonds for this reason; nav2's
   manager never got the fix.
3. **Component coupling.** `use_composition` puts all eight nodes in one
   `component_container_isolated`; a segfault in any one takes down all
   eight and the whole container restarts.

## Decision

**Kill the stream tax at the transport level:**

- Bridge `aligned_depth_to_color/image_raw` **raw** instead of
  `compressedDepth`. The PNG encode (~20–40 ms/frame) disappears entirely;
  the small align resample stays on the Pi (~10 ms). ~3 MB/s over the TCP
  link — nothing on a LAN. Companion `rtabmap` switches
  `depth_image_transport:=compressedDepth`→`raw`; the `detector` and
  `inspection_recorder` subscribe the raw topic (decode path updated).
- Color stream 15→5 fps. Every consumer already throttles below that
  (webui 4 Hz wire, detector 3 Hz, apriltag 2 Hz, rtabmap 1 Hz) — the JPEG
  encode cost drops two-thirds.
- IMU 200→100 Hz. Halves the per-message Python cost gyro_calibrator pays
  (~20%/core at 200 Hz, per the tilt_monitor comment) and the EKF's input
  rate. 100 Hz remains far above the fused-odom output rate.

**Make lifecycle resets survivable:**

- `nav2.launch.py` vendors the composable-node list (was a bare include of
  upstream `navigation_launch.py`) so `lifecycle_manager_navigation` is
  launched by Scout code with `bond_timeout: 0.0` — the same fix amcl.yaml
  already carries, same rationale: health is judged from topic liveness
  (`nav_state`, `health_monitor`), not bond resets.
- `lifecycle_manager_safety` (collision_monitor) gets `bond_timeout: 0.0`
  too — a starved bond reset there silently deactivates the autonomous
  cmd_vel path.

**Cut continuous CPU elsewhere:**

- `detect_objects` consumes the companion's new `/world/detections` topic
  (per-frame boxes + distances + map positions the detector already
  computes and discards) instead of running YOLO11n on the Pi. Pi-side
  inference remains as fallback when the topic is silent (companion down),
  keeping `detect.py` byte-identical (SC10) and the tool functional.
- `always_send_full_costmap: false` on both costmaps — the long-standing
  TODO in nav2.yaml. Costmap topics publish updates-only; controller and
  planner read the grid directly and never the topic.

**Get the missing observability:** the exporter gains host CPU temperature
and `scaling_cur_freq` gauges — a sustained-100% Pi 5 throttles at 85 °C,
and until now nothing in the metrics stack could see it.

## Consequences

- The Pi keeps every motion/safety-critical node: nav2, slam, EKF,
  collision_monitor, cliff_detector stay — cmd_vel never crosses the bridge
  (ADR-0001). What moved is *perception product*, not control.
- **Foxglove costmap panels show empty** with `always_send_full_costmap:
  false` (Foxglove ignores `/costmap_updates`). Deliberate trade; flip back
  temporarily for debugging sessions.
- `detect_objects` annotated boxes now lag the grabbed frame by up to ~1 s
  (companion detection cadence) — accurate on a stationary robot, noted in
  the tool's output. Pi-side YOLO + onnxruntime stay in the image purely as
  the companion-down fallback.
- `bond_timeout: 0.0` means a genuinely *hung* lifecycle node is no longer
  auto-reset — the accepted trade documented in amcl.yaml; recoveries are
  manual (`docker compose restart`).
- Raw aligned depth over zenoh uses ~3 MB/s sustained. If that ever
  matters, the follow-up is bridging the decimated unaligned depth and
  registering to color on the companion — needs a registration node the
  companion image doesn't have today (rtabmap_util ships none on ros2).
- IMU at 100 Hz: re-verify gyro bias behavior in `gyro_calibrator` and EKF
  yaw quality on the first run; revert is a two-line config change.
- D455 `640x480x5` color profile: if the wrapper rejects it it logs the
  supported list — fallback is `640x480x15`.
