#!/bin/bash
# deploy-pi.sh <branch> [sha] — the Pi's deploy: pull-from-GHCR successor to
# scout-switch.sh (which stays as the build-locally-on-Pi fallback).
#
# Normally invoked by .github/workflows/deploy.yml on the self-hosted `pi`
# runner (which git-updates the checkout FIRST so this file runs at the new
# revision, and passes SCOUT_PREV_BRANCH). Manual use:
#   SCOUT_REPO=~/Desktop/Scout scripts/deploy-pi.sh companion-v1
#
# ⚠ Restarts the drivetrain driver — never run while the robot is driving.
#
# Wrapped in main() so bash parses the whole file before executing anything:
# run manually, the script git-checkouts the repo it lives in mid-run, and an
# unwrapped script would be re-read from the NEW revision at whatever byte
# offset bash had reached.
set -euo pipefail

main() {
  REPO="${SCOUT_REPO:-$HOME/Desktop/Scout}"
  BRANCH="${1:?usage: deploy-pi.sh <branch> [sha]}"

  cd "$REPO"
  if ! git diff --quiet || ! git diff --cached --quiet; then
    echo "working tree has local changes — commit or stash first (deploy-via-git-only):" >&2
    git status --short >&2
    exit 1
  fi

  # Pre-checkout branch, for the stale-build-cache decision below. The deploy
  # workflow checks out before calling us, so it passes the real value in.
  PREV_BRANCH="${SCOUT_PREV_BRANCH:-$(git rev-parse --abbrev-ref HEAD)}"

  git fetch origin "$BRANCH"
  git checkout -q "$BRANCH"
  # Fast-forward only: never invent merge commits on the robot.
  git merge --ff-only "origin/$BRANCH"

  SHA="${2:-$(git rev-parse HEAD)}"
  if [ "$SHA" != "$(git rev-parse HEAD)" ]; then
    echo "checkout is at $(git rev-parse HEAD) but images were built for $SHA — refusing code/image mismatch" >&2
    exit 1
  fi

  # Every profile, so pull/down see the whole stack (explore, observability,
  # build_package). Which services actually START is decided at `up` below.
  ALL="--profile full --profile observability --profile explore --profile build"

  # Pin every scout image to this commit — compose reads SCOUT_TAG from .env.
  # Upsert only our line; .env may carry SCOUT_PROFILE / COMPANION_HOST.
  touch .env
  { grep -v '^SCOUT_TAG=' .env || true; echo "SCOUT_TAG=$SHA"; } > .env.tmp
  mv .env.tmp .env

  docker compose $ALL pull

  # Stamp guard (ADR-0005): the overlay volume seeds from the image ONCE, and
  # /ros_entrypoint.sh hard-fails every scout service on a stale stamp. If the
  # pulled image carries a new build id — or the branch changed (stale
  # ros_ws_build after a branch switch; build + install must be wiped as a
  # pair) — take the stack down with -v so the volume re-seeds on up.
  IMG_ID=$(docker run --rm --entrypoint cat "ghcr.io/bg-bgi/scout:$SHA" /opt/overlay/.image_build_id 2>/dev/null || echo unknown)
  VOL=$(docker volume ls -q --filter name=ros_overlay_install | head -1)
  VOL_ID=none
  if [ -n "$VOL" ]; then
    VOL_ID=$(docker run --rm -v "$VOL":/stamp --entrypoint cat "ghcr.io/bg-bgi/scout:$SHA" /stamp/.image_build_id 2>/dev/null || echo none)
  fi
  WIPED=0
  if [ "$PREV_BRANCH" != "$BRANCH" ] || [ "$IMG_ID" != "$VOL_ID" ]; then
    echo "== wiping build+install volumes (branch $PREV_BRANCH -> $BRANCH, stamp $VOL_ID -> $IMG_ID)"
    docker compose $ALL down -v --remove-orphans
    WIPED=1
  fi

  # Location sites (ADR-0023), one-time on a pre-sites data layout.
  if [ ! -e sites/active ] && [ -f scripts/migrate_sites.py ]; then
    echo "migrating maps/ + captures/ into sites/default (ADR-0023)..."
    python3 scripts/migrate_sites.py
  fi

  # Seed/refresh the overlay volume and colcon-build scout into it (~2-3 min).
  docker compose --profile build run --rm build_package

  # Pi 5 has no RTC: on a cold boot NTP corrects the clock with a step, and a
  # step landing under a running slam/nav2 poisons the pose graph/costmaps for
  # the session. Block until synced before starting sensor-driving containers.
  echo "waiting for NTP time sync..."
  until [ "$(timedatectl show -p NTPSynchronized --value)" = "yes" ]; do sleep 1; done

  # Stale-project sweep: containers running OUR services under a different
  # compose project name (a pre-`name:`-pin checkout, a runner workdir) are
  # invisible to this project's down/--remove-orphans, duplicate workload and
  # squat on host-network ports (2026-08-27 companion incident: a retired
  # project's foxglove_bridge held :8766 while the real one crash-looped).
  # Removes exactly: same compose service name, different project. PROJECT
  # must match the `name:` pinned in docker-compose.yaml.
  PROJECT=scout
  OURS=$(docker compose $ALL config --services)
  docker ps -a --format '{{.ID}}\t{{.Label "com.docker.compose.project"}}\t{{.Label "com.docker.compose.service"}}' \
  | while IFS="$(printf '\t')" read -r id proj svc; do
      { [ -n "$proj" ] && [ "$proj" != "$PROJECT" ]; } || continue
      echo "$OURS" | grep -qx "$svc" || continue
      echo "== removing stale container from retired project '$proj' (service $svc)"
      docker rm -f "$id"
    done

  # Start everything ungated plus the `full` profile — exactly what this
  # branch's compose defines, no hardcoded service list to drift. explore and
  # observability stay down (start observability via the ops workflow).
  docker compose --profile full up -d --remove-orphans

  # No wipe = up -d recreated nothing, but build_package symlink-installs the
  # scout packages, and a running process only loads new Python at start —
  # restart so the deploy actually takes effect (the wipe path already
  # recreated everything).
  if [ "$WIPED" -eq 0 ]; then
    echo "== no volume wipe — restarting services to load the new install"
    docker compose --profile full restart
  fi

  # Pre-create (NOT start) the profile-gated explore container so
  # scout_skills' explore_start can lazily start it via fleet_status. Created
  # state: no CPU, no node, no motion, no restart policy.
  if docker compose --profile explore config --services 2>/dev/null | grep -qx explore; then
    docker compose --profile explore create explore
  fi

  # Smoke: every service `up` just started must still be running; then a
  # warn-only scan of the robot log past the known startup serial burst.
  sleep 15
  EXPECTED=$(docker compose --profile full config --services)
  RUNNING=$(docker compose --profile full ps --status running --services)
  FAIL=0
  for s in $EXPECTED; do
    [ "$s" = build_package ] && continue
    echo "$RUNNING" | grep -qx "$s" || { echo "NOT RUNNING: $s" >&2; FAIL=1; }
  done
  echo "-- robot log errors (excluding the known startup serial burst):"
  docker compose logs --tail 100 robot 2>/dev/null | grep -iE "error|fatal" | grep -viE "RETRY COUNT EXCEEDED|crc" || echo "   none"
  [ "$FAIL" -eq 0 ] || exit 1

  echo "== deployed $BRANCH @ $(git rev-parse --short HEAD) (images pinned to :$SHA)"
  docker ps --format '{{.Names}}  {{.Status}}' | grep scout || true
}

main "$@"
