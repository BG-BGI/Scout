# Companion host provisioning (one-time, operator)

The companion is a **native Linux host** — the Debian box at 10.1.57.18
(ADR-0021/0022). The stack is containerized (`ros:humble-ros-base`), so the
host distro doesn't matter. 4+ cores, 8 GB RAM, 40 GB free (rtabmap DBs grow
to GBs at house scale).

Cross-VLAN placement is fine: the only Pi-facing network requirement is
**outbound TCP to the Pi's `:7447`** (the zenoh bridge). UDP/DDS never crosses
machines (corp inter-VLAN filtering drops it — measured 2026-08-20). If even
that TCP port is filtered, reverse the bridge roles (this box listens, the Pi
dials) or request the single port from IT.

The box HAS internet but **cannot do interactive GitHub SSO**, so the deploy
path uses the two credential types that work without a browser login on the
box:
- **Read-only deploy key** on BG-BGI/Scout — deploy keys are repo-scoped and
  exempt from SAML SSO enforcement, so `git pull` works directly.
- **`read:packages` PAT for GHCR** — created and SSO-authorized from a browser
  on the Mac ("Configure SSO → Authorize" on the token page), then pasted once
  into `docker login ghcr.io` on the box.

Images are built by CI (`.github/workflows/companion-images.yml`, native
amd64) and pushed to `ghcr.io/bg-bgi/scout-companion{,-detector}` — the Mac
never cross-builds and nothing travels by USB.

## Requirements

1. `ping 10.1.80.31` works (routed is fine, same subnet not required).
2. **Docker Engine + Compose v2** (engine, not Desktop):
   https://docs.docker.com/engine/install/debian/ — then
   `sudo usermod -aG docker $USER`, re-login.
3. NTP synced (`timedatectl` → "System clock synchronized: yes") — TF and
   message stamps cross machines.
4. Stable address (DHCP reservation); record it as `COMPANION_HOST` in the
   Pi's `.env` and use it for the Foxglove bookmark (`ws://<box>:8766`).

## Install

One-time credentials:

```bash
# 1. Deploy key (on the box):
ssh-keygen -t ed25519 -f ~/.ssh/scout_deploy -N "" -C companion-deploy
cat ~/.ssh/scout_deploy.pub
#    → paste into GitHub: BG-BGI/Scout → Settings → Deploy keys → Add
#      (read-only). Then:
cat >> ~/.ssh/config <<'EOF'
Host github.com
  IdentityFile ~/.ssh/scout_deploy
  IdentitiesOnly yes
EOF

# 2. GHCR login (PAT made on the Mac: Settings → Developer settings → Tokens
#    (classic) → read:packages → Configure SSO → Authorize BG-BGI):
docker login ghcr.io -u <github-username>   # password = the PAT
```

Then:

```bash
git clone git@github.com:BG-BGI/Scout.git && cd Scout/companion
cp .env.example .env                          # PI_IP=10.1.80.31
./update.sh                                   # git pull + compose pull + up -d
```

Every subsequent deploy: commit → push GitHub → CI builds → `./update.sh`
on the box. Never hand-edit tracked files here (deploy-via-git-only, same
rule as the Pi).

## Fallback: USB sneakernet (no-network only)

If GitHub or GHCR is unreachable: on the Mac,
`docker buildx build --platform linux/amd64 --load`, `docker save | gzip`,
carry `*.tar.gz` on the stick, `gunzip -c <file> | docker load` on the box.
⚠ A tarball loads under its save-time tag — retag to what compose expects
(`ghcr.io/bg-bgi/scout-companion:latest` etc.) or compose won't find it.

Bring-up/teardown, sanity checks, and the replay gate: `README.md`.
