#!/usr/bin/env bash
# Companion update — the whole deploy loop on the box side.
# Prereqs (one-time, host-setup.md): deploy-key clone + `docker login ghcr.io`.
#
# Usage:
#   ./update.sh                    # manual: ff-pull current branch, pull, up
#   ./update.sh <branch> [sha]     # deploy.yml: switch branch, pin COMPANION_TAG
#
# With a sha, COMPANION_TAG in .env is pinned so compose pulls the images CI
# built for exactly that commit; without one the previous pin (or :latest) is
# kept — a bare push may not have built :<sha> images (path filters), so only
# the deploy workflow, which guarantees the build, passes it.
#
# Wrapped in main() so bash parses the whole file before executing: git pull
# updates this very file mid-run, and an unwrapped script would be re-read
# from the new revision at whatever byte offset bash had reached.
set -euo pipefail

main() {
  cd "$(dirname "$0")"
  BRANCH="${1:-}"
  SHA="${2:-}"

  if [ -n "$BRANCH" ]; then
    git fetch origin "$BRANCH"
    git checkout -q "$BRANCH"
  fi
  git pull --ff-only

  if [ -n "$SHA" ]; then
    if [ "$SHA" != "$(git rev-parse HEAD)" ]; then
      echo "checkout is at $(git rev-parse HEAD) but images were built for $SHA — refusing code/image mismatch" >&2
      exit 1
    fi
    # Upsert only our line — .env carries PI_IP.
    touch .env
    { grep -v '^COMPANION_TAG=' .env || true; echo "COMPANION_TAG=$SHA"; } > .env.tmp
    mv .env.tmp .env
  fi

  # Location sites (ADR-0023), one-time: seed data/sites/default with the db
  # from the retired rtabmap_db named volume (if any), point active at it.
  if [ ! -e data/sites/active ]; then
    mkdir -p data/sites/default
    if docker volume inspect scout-companion_rtabmap_db >/dev/null 2>&1; then
      echo "migrating rtabmap.db out of the rtabmap_db volume (ADR-0023)..."
      docker run --rm -v scout-companion_rtabmap_db:/src \
        -v "$PWD/data/sites/default":/dst alpine \
        sh -c 'cp /src/rtabmap.db /dst/ 2>/dev/null || true'
    fi
    ln -s default data/sites/active
    echo "sites: active -> default (delete the old volume once verified:"
    echo "  docker volume rm scout-companion_rtabmap_db)"
  fi

  docker compose pull
  docker compose up -d --remove-orphans
  docker image prune -f

  # Smoke: everything compose defines (default profiles) must be running.
  sleep 5
  FAIL=0
  RUNNING=$(docker compose ps --status running --services)
  for s in $(docker compose config --services); do
    echo "$RUNNING" | grep -qx "$s" || { echo "NOT RUNNING: $s" >&2; FAIL=1; }
  done
  echo "== companion on $(git rev-parse --abbrev-ref HEAD) @ $(git rev-parse --short HEAD)${SHA:+ (images pinned to :$SHA)}"
  [ "$FAIL" -eq 0 ]
}

main "$@"
