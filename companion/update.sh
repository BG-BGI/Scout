#!/usr/bin/env bash
# Companion update — the whole deploy loop on the box side.
# Prereqs (one-time, host-setup.md): deploy-key clone + `docker login ghcr.io`.
set -euo pipefail
cd "$(dirname "$0")"
git pull --ff-only

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
