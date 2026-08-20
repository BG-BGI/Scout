# Companion host provisioning (one-time, operator)

The companion is a **native Linux host** — the Debian box at 10.1.57.18
(ADR-0021/0022). The stack is containerized (`ros:humble-ros-base`), so the
host distro doesn't matter. 4+ cores, 8 GB RAM, 40 GB free (rtabmap DBs grow
to GBs at house scale).

Cross-VLAN placement is fine: the only network requirement is **outbound TCP
to the Pi's `:7447`** (the zenoh bridge). UDP/DDS never crosses machines
(corp inter-VLAN filtering drops it — measured 2026-08-20). If even that TCP
port is filtered, reverse the bridge roles (this box listens, the Pi dials)
or request the single port from IT.

Docker Desktop on macOS remains unusable as a companion host for anything
DDS-adjacent; with the zenoh bridge it would technically work (pure TCP), but
the Debian box is the primary and a bridged UTM VM the fallback.

## Requirements

1. `ping 10.1.80.31` works (routed is fine, same subnet not required).
2. **Docker Engine + Compose v2** (engine, not Desktop):
   https://docs.docker.com/engine/install/debian/ — then
   `sudo usermod -aG docker $USER`, re-login.
3. NTP synced (`timedatectl` → "System clock synchronized: yes") — TF and
   message stamps cross machines.
4. Stable address (DHCP reservation); record it as `COMPANION_HOST` in the
   Pi's `.env` and use it for the Foxglove bookmark (`ws://<box>:8766`).
5. No-internet box: load the two images from USB instead of pulling —
   `scout-companion-amd64.tar.gz` and `zenoh-bridge-ros2dds-amd64.tar.gz`
   (`gunzip -c <file> | docker load`). ⚠ The bridge tarball loads under its
   save-time tag — retag or compose won't find it:
   `docker tag zenoh-bridge-amd64:1.10.0 eclipse/zenoh-bridge-ros2dds:1.10.0`

## Install

```bash
git clone <repo-url> && cd Scout/companion   # or copy companion/ from USB
cp .env.example .env                          # PI_IP=10.1.80.31
docker compose build                          # skip if image loaded from USB
```

Bring-up/teardown, sanity checks, and the replay gate: `README.md`.
