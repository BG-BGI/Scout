# ADR-0027: CI builds all images on GitHub runners; deploy/ops are dispatch-only Actions on device self-hosted runners

Status: accepted · Date: 2026-08-25 · Extends ADR-0005 (deploy = git pull)
and the companion GHCR flow from ADR-0022's stack

## Context

Deploying was a ~10-step manual ritual that drifted every time the compose
files did: buildx arm64 images on a Mac, `docker save | gzip | scp` to the Pi,
ssh in, `down -v`, git pull, `docker load`, `build_package`, `up -d`, then ssh
to the companion for `update.sh`. Branch switches (scout-switch.sh) rebuilt
the 13-min librealsense image ON the Pi, which the Mac-build rule existed to
avoid. Nothing tagged Pi images, `:latest` was clobbered by whichever branch
pushed last, and the ADR-0005 stamp-guard `down -v` was tribal knowledge, not
automation.

The repo is public, which makes GitHub's `ubuntu-24.04-arm` hosted runners
free — the one fact that removes the Mac from the loop: `scout:latest` builds
natively arm64 in CI.

## Decision

**Every image is built by GitHub-hosted runners and pushed to GHCR; both
devices only ever `docker compose pull`. Deploys, branch switches, and
container ops are `workflow_dispatch`-only workflows running on self-hosted
runners on the Pi (label `pi`) and companion (label `companion`).**

- **Build workflows** — `pi-images.yml` (arm64, native `ubuntu-24.04-arm`:
  scout, scout-ros-mcp, scout-skills, both observability images),
  `companion-images.yml` (amd64: scout-companion, -detector, -captioner),
  `fleet-status-image.yml` (multi-arch amd64+arm64 — the one image both
  devices pull; two single-arch workflows pushing one tag would clobber each
  other's manifest). Path-filtered on every branch. Build cache in a GHCR
  `:buildcache` ref (the gha cache's 10 GB cap can't hold librealsense).
- **Tags**: `:<sha>` (what deploys pin), `:<branch>`, and `:latest` only from
  main — so no branch push can change what a `${TAG:-latest}` fallback
  resolves to. Compose interpolates `SCOUT_TAG` (Pi `.env`) / `COMPANION_TAG`
  (companion `.env`); the deploy path writes the sha, making a deploy exactly
  reproducible and a rollback "deploy the older commit".
- **`deploy.yml`** resolves the branch pin (repo Actions variables
  `PI_BRANCH` / `COMPANION_BRANCH`, overridable per-run), `workflow_call`s
  the build workflows at that exact sha (push path filters may have skipped
  it), then runs `scripts/deploy-pi.sh` / `companion/update.sh` inside the
  device's own clone (`$SCOUT_REPO` in the runner env) — deploy-via-git-only
  still holds; runners never use a workdir checkout.
- **`deploy-pi.sh`** (successor to scout-switch.sh) automates the ADR-0005
  hazard: it compares the pulled image's `.image_build_id` against the
  `ros_overlay_install` volume's and runs `down -v` on mismatch **or on any
  branch change** (stale `ros_ws_build` pairs with the overlay). It keeps the
  NTP-sync block, sites migration, explore pre-create, and adds a smoke check
  (all `--profile full` services running) that fails the Actions run.
- **`ops.yml`**: ps/logs/restart/stop/start/up/down per device from the
  Actions UI. `down -v` is deliberately not offered. Shares a `deploy`
  concurrency group so ops and deploys serialize.
- **Safety**: a Pi deploy restarts the drivetrain driver, so `deploy.yml`
  refuses `pi`/`both` unless the operator checks the robot-stationary
  confirmation input. Deploys never auto-trigger on push.
- **Public-repo runner hardening**: self-hosted labels appear ONLY in
  dispatch-only workflows (dispatch requires write access, and fork PRs can't
  reach them); repo Actions settings must require approval for outside
  collaborators' workflow runs. See docs/deploy.md.

## Consequences

- Merged code becomes deployable images with zero Mac involvement; a deploy
  or branch switch is one button with the safety checkbox.
- First pull of a new scout image is ~3.2 GB over Wi-Fi (later pulls are
  layer-deduped); first CI build after cache eviction pays the full ~13 min
  librealsense compile.
- scout-switch.sh stays as the offline/build-locally fallback but is no
  longer the deploy path.
- The devices run a GitHub runner agent each (systemd), with `SCOUT_REPO` in
  the runner's `.env`; the Pi needs a one-time `docker login ghcr.io` (same
  read:packages PAT pattern as the companion) unless the GHCR packages are
  made public.
