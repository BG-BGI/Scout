# Companion world-model (continuous perception for Magnus)

Replaces stop-and-snapshot: Magnus queries an always-fresh world-state instead
of commanding a stop and grabbing one frame per turn. Heavy perception runs on
the companion (amd64, 4-core, 8 GB, **no GPU**); only compact read-only state
crosses back to the Pi.

## Why this shape

- **LLMs consume turns, not video.** "Continuous feed into Magnus" is a
  category error — the fix is continuous perception OUT of the LLM loop, feeding
  a queryable state.
- `camera_snapshot` (scout-skills `server.py:714`) already does NOT stop the
  robot — it reads one live frame off `/camera/.../color/image_raw`. The felt
  "stop and shoot" is the MCP request/response cadence, not a device limit.

## GPU decision: none, now

Magnus reasons in multi-second turns, so a 3–6 s caption refresh is matched to
the consumer, not a bottleneck. YOLO on onnxruntime-CPU (amd64) runs 3–5 fps
continuously. Add a GPU only for a real-time HUD for a **human**; the agent path
does not need it.

## Message contract — `std_msgs/String` JSON, latched

NOT `vision_msgs` — that type would need installing on three surfaces (companion
DDS, Pi DDS, and the rosbridge JSON layer scout-skills speaks, which has no ROS
in its image). String+JSON crosses zenoh natively, serializes over rosbridge,
needs no extra package anywhere, and the consumer is an LLM eating JSON anyway.
Both topics latched (transient_local, depth 1) so a late scout-skills subscriber
gets the last value immediately.

- `/world/objects` — `[{id, cls, score, xy:[x,y], z, last_seen}]`, map frame
- `/world/scene` — `{stamp, caption, tags:[...]}`

## Allowlist diff — first-ever inbound to the Pi (read-only telemetry)

Pi `scout/config/zenoh_bridge.json5`:
```
subscribers: [ "^/world/objects$", "^/world/scene$" ],
```
Companion `companion/config/zenoh_bridge.json5`:
```
publishers: [ "^/world/objects$", "^/world/scene$" ],
```
Anchored regexes. `/cmd_vel_*` stays absent from the Pi `subscribers` list, so
control authority is unchanged (ADR-0001). Never wildcard this.

## Companion nodes

**detector_node** — extend `scout-companion` image with onnxruntime+numpy+pillow
(no torch). YOLO11n (reuse `docker/scout-skills/detect.py` ONNX + decode) on
bridged compressed color -> median depth in bbox -> deproject via camera_info ->
TF `camera_color_optical_frame -> map` (`/tf`+`/tf_static` already bridged) ->
nearest-neighbor track (stable id, EMA xy, drop stale). Publish `/world/objects`
latched ~3 Hz; inference capped 3–5 fps.

**captioner_node** — own image (torch-cpu ~2 GB; isolate the heavy dep, per the
project's one-image-per-heavy-dep pattern). Moondream2 or Florence-2-base, CPU,
prompt "describe the scene". Trigger on frame-change to save CPU; publish
`/world/scene` latched (~0.2 Hz).

## scout-skills exposure (Pi `server.py`) — Magnus stays on one connector

- `whats_around_me()` -> subscribe_once `/world/objects` -> list + per-object age
- `describe_scene()` -> subscribe_once `/world/scene` -> caption + "as of Ns ago"
- `where_is(query)` -> filter objects by class
- Silent topic -> `"world-model offline (companion down)"` (Pi-standalone
  contract §0.7 preserved).

## Build order (gated)

1. **detector_node only** (no torch). Gate: one object's map xy matches
   tape / rtabmap cloud.
2. **allowlist edit + Pi inbound verify.** Gate: `/world/objects` reaches the
   Pi; confirm a companion-published `/cmd_vel` is still refused.
3. **scout-skills tools + Magnus round-trip.** Gate: `whats_around_me` returns
   live objects while driving, no stop.
4. **captioner_node** (torch-cpu image). Gate: caption latency measured;
   `docker stats` on companion confirms rtabmap+detector+captioner fit 4 cores.

Human video (Foxglove `ws://<box>:8766`) is untouched and independent.
