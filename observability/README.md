# Scout observability

Localhost dashboard + Claude MCP debugging for the Pi's docker stack. Runs
entirely on your dev machine; the Pi side is 3 opt-in containers behind the
`observability` compose profile.

## Setup

**On the Pi** (once, or after pulling this change):
```
docker compose --profile observability up -d
```
Starts `observability_exporter` (:9100, metrics), `observability_mcp`
(:9002/mcp), `dozzle_agent` (:7007, log streaming). All three are read-only
against the docker socket except `observability_mcp`'s `restart_container`
tool, which refuses `robot` unconditionally (see its docstring).

**On your dev machine:**
```
cd observability
cp .env.example .env   # edit PI_HOST if scout.local doesn't resolve
docker compose up -d
```
- Grafana: http://localhost:3000 (admin/admin first login) -- "Scout" dashboard, auto-provisioned
- Prometheus: http://localhost:9090 -- raw PromQL if the dashboard isn't enough
- Dozzle: http://localhost:8081 -- live/searchable logs across every service, no more `docker compose logs -f <service>` by hand

## Claude MCP

Register the Pi's MCP endpoint with Claude Code:
```
claude mcp add --transport http scout-observability http://scout.local:9002/mcp
```
Gives Claude: `list_containers`, `container_logs` (with grep), `container_stats`,
`ros2_topic_hz`, `ros2_topic_info` (QoS profiles, catches mismatches),
`ros2_node_list`, and `restart_container` (blocked for `robot`).

## What's NOT covered

- No host-level Pi metrics (temp, disk, non-container CPU) -- only container-scoped. Add if thermal throttling becomes a suspect.
- `network_mode: host` on every Scout service means per-container network rx/tx may read 0 (Docker only meters network stats inside a container's own netns) -- CPU and memory numbers are reliable, network numbers are a bonus if they show up at all.
- Nothing here touches `cmd_vel` or any motion path. It's read-only introspection plus non-motion restarts.
