# Companion stack — live RTAB-Map 3D capture over the zenoh bridge (ADR-0022)

Runs on a **native Linux host** — the Debian box (10.1.57.18), see
`host-setup.md`. Each machine keeps its own loopback DDS graph; the only
cross-machine traffic is `zenoh_bridge`'s **single TCP connection** to the
Pi's `:7447` (corp inter-VLAN filtering drops UDP, so shared-domain DDS is
out — measured 2026-08-20, ADR-0022).

The box holds the **full Scout repo**; the companion services run from its
`companion/` subdir (`cd <repo>/companion`), which is where every command below
and in `host-setup.md` is run.

**Contract: the Pi must keep working with this stack absent** (spec §0.7).
The Pi-side bridge accepts only read-only world-model telemetry inbound
(`subscribers: ["^/world/objects$"]` — WORLDMODEL.md gate 2), so nothing here
can reach a control topic, `/cmd_vel_*` included. Extending what crosses =
editing BOTH allowlists (`config/zenoh_bridge.json5` here, mirrored in
`scout/config/zenoh_bridge.json5`) — deliberate, never wildcarded.

## Bring-up

```bash
cp .env.example .env        # set PI_IP (10.1.80.31)
./update.sh                 # git pull + compose pull (GHCR) + up -d
docker compose logs zenoh_bridge | tail
```

Images come from `ghcr.io/bg-bgi/scout-companion{,-detector}` (CI-built on
push, `.github/workflows/companion-images.yml`); one-time deploy-key +
`docker login ghcr.io` setup in `host-setup.md`. `docker compose build` is
for local dev only.

Bridge health, in order (ADR-0022 bring-up findings):
1. `New ROS 2 bridge detected: <zid>` — TCP session with the Pi is up.
2. `Route Subscriber ... created` lines — local DDS discovery works. Session
   up but **zero route lines = the cross-vendor localhost trap**: the bridge
   needs `ROS_LOCALHOST_ONLY=1` + `ROS_DISTRO=humble` + the `CYCLONEDDS_URI`
   peer-ping env (all three in docker-compose.yaml) or Fast DDS and Cyclone
   are mutually deaf on loopback with no error anywhere.
3. Data: `docker compose exec rtabmap bash -lc "source /opt/ros/humble/setup.bash && ros2 topic echo /scan sensor_msgs/msg/LaserScan --once"`.
   ⚠ `ros2 topic list` is NOT evidence of data — rtabmap's own subscriptions
   list the topics even when nothing flows.

View the live map: Foxglove → `ws://10.1.57.18:8766` → `/rtabmap/cloud_map`
(+ `/rtabmap/mapGraph`). Deliberately a local bridge so the cloud never hauls
back through the Pi's `:8765`.

Sanity (inside this box, topics arrive via the bridge):
```bash
docker compose exec rtabmap bash -lc \
  'source /opt/ros/humble/setup.bash && ros2 topic hz /scan'   # expect ~11.7 Hz
```

## Floorplan importer (`floorplan_importer`, :9004)

Turns a PDF or IFC drawing into a nav2 grid pair and installs it on the Pi by
POSTing to fleet_status (`POST /api/maps/install`); ADR-0040, operating recipe
`docs/slam.md` §6.7–6.10 (the web UI's **Import drawing…** talks to it). HTTP
only: no ROS, no zenoh topic, no docker socket, not privileged, no volumes
(uploads live in memory and are gone on restart). Optional: the robot never
depends on it, and a drawing pair can always be installed by hand
(`docs/slam.md` §6.1). Image `ghcr.io/bg-bgi/scout-floorplan-importer`, built by
`.github/workflows/floorplan-importer-image.yml` (amd64 + arm64).

```bash
curl -s http://<box>:9004/api/health   # {"ok": true, "version", "fleet_status_url", "jobs", "busy", "ifc", "rss_mb"}
docker compose ps floorplan_importer   # image HEALTHCHECK polls the same URL every 30 s
docker compose logs floorplan_importer | tail
```

- **Environment.** The compose file sets `IMPORTER_PORT` (default 9004),
  `FLEET_STATUS_URL=http://${PI_IP}:9003` (the Pi's fleet_status; from the same
  `PI_IP` as the zenoh bridge, never taken from a request; unset or unusable =
  the service still starts and installs answer `503`) and `IMPORTER_CORS_ORIGIN`
  (default `*`). The rest are read by the server but not set in compose, so add
  them under the service's `environment:` to change them:
  `IMPORTER_MAX_IFC_BYTES` (IFC upload cap, default 80 MiB),
  `IMPORTER_IFC_THREADS` (geometry threads per slice, default 1),
  `IMPORTER_IFC_FACE_BUDGET` (triangles per slice, default 3,000,000),
  `IMPORTER_MEMORY_LIMIT_MB` (what the IFC memory gate measures against; default
  the container's cgroup limit, unreadable = gate off). `IMPORTER_VERSION` is the
  image's build sha (written into every installed map's sidecar).
- **Caps.** `cpus: 1.0` (one heavy operation at a time, single-threaded; the box
  shares cores with rtabmap and the detector) and `mem_limit: 2g`. The 80 MiB IFC
  default is sized for 2g: **raise `mem_limit` before `IMPORTER_MAX_IFC_BYTES`**
  (~128 MiB at 3g, ~256 MiB at 6g). Not yet measured under these caps on the
  real box.
- **Restart behaviour.** `restart: unless-stopped`. The IFC memory gate
  deliberately exits the process (code 75, after answering `503` + `Retry-After:
  15`) and compose brings it back, dropping every open import; wait ~15 s and
  upload again. A `429` from the gate means an import is busy: nothing was
  dropped. On start the importer removes stale `importer-ifc-*` temp dirs.
  `GET /api/health` → `rss_mb` shows the resident set.
- **Exposure.** The server listens on all interfaces (the service is
  `network_mode: host`), has **no auth** and answers CORS `*`: anything on the LAN
  that can reach `:9004` can upload and install a *drawing* map through it (it
  cannot activate a map or replace a scan map). Restrict the port or set
  `IMPORTER_CORS_ORIGIN` to the web UI's origin if the box is on a shared
  network.
- **Reachability.** The operator's browser calls the importer directly, so it
  must reach `<box>:9004`; the importer must reach `http://<PI_IP>:9003`
  (box → Pi TCP; the bridge only proves `:7447`). A `502 … (upstream: …)` on
  install means the second hop failed.

## Teardown / absence

`docker compose down` (or the box going away entirely) must cost the robot
nothing: the Pi-side bridge idles and rtabmap resumes when the TCP session
re-establishes. Map DBs persist per location site in
`data/sites/<site>/rtabmap.db` (ADR-0023); fresh session for the active site =
stop rtabmap, delete that file, start rtabmap.

## Replay gate (offline, no network dependency)

Record a teleop bag on the Pi (ADR-0017 tooling) with:
`/camera/camera/color/image_raw/compressed`,
`/camera/camera/aligned_depth_to_color/image_raw` (raw, since ADR-0033),
`/camera/camera/color/camera_info`, `/odom`, `/tf`, `/tf_static`, `/scan`.
Copy it to this box and:

```bash
docker compose run --rm -v /path/to/bag:/bag rtabmap bash -lc \
  'source /opt/ros/humble/setup.bash && ros2 bag play /bag --clock' &
docker compose up rtabmap   # with use_sim_time:=true for the replay
```

Pass: nonzero-node database + coherent cloud in Foxglove.
