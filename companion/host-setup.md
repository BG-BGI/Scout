# Companion host provisioning (one-time, operator)

The companion is any **native Linux host on the robot's LAN** (ADR-0021).
Primary: the Debian box. The stack is containerized (`ros:humble-ros-base`),
so the host distro doesn't matter — Debian is fine even though ROS debs are
Ubuntu-only. Any recent x86_64 or arm64 works; 4+ cores, 8 GB RAM, 40 GB free
(rtabmap DBs grow to GBs at house scale).

**Never Docker Desktop on macOS** — its NAT breaks the DDS data plane
(VM-internal locators, no multicast, host networking is a port proxy). If the
Mac must be the companion, use a UTM Ubuntu VM with a **bridged** NIC so it
owns a real LAN IP, then follow this doc inside the VM.

## Requirements

1. **Same L2 subnet as the Pi** (data plane is peer-to-peer UDP; discovery
   dials the Pi). Verify: `ip addr` shows an address in the Pi's subnet and
   `ping <pi-ip>` works.
2. **Docker Engine + Compose v2** (engine, not Desktop):
   https://docs.docker.com/engine/install/debian/ — then
   `sudo usermod -aG docker $USER` and re-login.
3. **No firewall blocking UDP** from the Pi: DDS uses ephemeral UDP both ways.
   Default Debian has no filter; if nftables/ufw is active, allow the Pi's IP.
4. **NTP synced** (`timedatectl` → "System clock synchronized: yes") — TF and
   message stamps cross machines now.
5. Stable address: DHCP reservation for the box, note it as `COMPANION_HOST`
   in the Pi's `.env` and use it for Foxglove bookmarks.

## Install

```bash
git clone <repo-url> && cd Scout/companion   # or copy companion/ alone
cp .env.example .env                          # set PI_IP=<pi address>
docker compose build                          # plain apt image, a few minutes
```

Bring-up/teardown, sanity checks, and the replay gate: `README.md`.
