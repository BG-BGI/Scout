#!/bin/bash
# companion-adopt-chris.sh — one-off bootstrap (2026-08-25, ADR-0027 rollout).
# The companion's Scout clone / deploy key / ghcr login lived under root while
# the Actions runner was configured under chris. This moves everything to
# chris so the runner can deploy. Run ON the companion, AS ROOT:
#   curl -fsSL https://raw.githubusercontent.com/BG-BGI/Scout/companion-v1/scripts/companion-adopt-chris.sh | sudo bash
# Idempotent — safe to re-run. Delete from the repo once the box is green.
set -euo pipefail

main() {
  U=chris
  H=/home/$U
  [ "$(id -u)" -eq 0 ] || { echo "run as root (sudo)"; exit 1; }
  id "$U" >/dev/null

  # 1. Move the clone. Running containers keep working through the move (the
  # kernel tracks their bind mounts by inode); the compose up at the end
  # recreates them against the new paths before any restart can dangle.
  if [ -d /root/Scout ] && [ ! -d "$H/Scout" ]; then
    echo "== moving /root/Scout -> $H/Scout"
    mv /root/Scout "$H/Scout"
  fi
  [ -d "$H/Scout/companion" ] || { echo "FATAL: $H/Scout/companion missing"; exit 1; }
  chown -R "$U:$U" "$H/Scout"

  # 2. Deploy key: copy root's ssh config + every identity it references.
  echo "== ssh"
  mkdir -p "$H/.ssh"
  if [ -f /root/.ssh/config ]; then
    cp -n /root/.ssh/config "$H/.ssh/config" || true
    awk 'tolower($1)=="identityfile"{print $2}' /root/.ssh/config | while read -r f; do
      f="${f/#\~\//\/root\/}"
      cp -n "$f" "$H/.ssh/" 2>/dev/null || true
      cp -n "$f.pub" "$H/.ssh/" 2>/dev/null || true
    done
  fi
  # Belt and braces: any bare keys too.
  for k in /root/.ssh/id_*; do [ -e "$k" ] && cp -n "$k" "$H/.ssh/" || true; done
  chown -R "$U:$U" "$H/.ssh"
  chmod 700 "$H/.ssh"
  chmod 600 "$H"/.ssh/* 2>/dev/null || true
  chmod 644 "$H"/.ssh/*.pub 2>/dev/null || true

  # 3. Docker group + ghcr auth for chris.
  echo "== docker"
  usermod -aG docker "$U"
  mkdir -p "$H/.docker"
  [ -f /root/.docker/config.json ] && cp -f /root/.docker/config.json "$H/.docker/config.json"
  chown -R "$U:$U" "$H/.docker"

  # 4. Runner env: point the deploy workflow at the clone.
  ENVF="$H/actions-runner/.env"
  [ -d "$H/actions-runner" ] || { echo "FATAL: $H/actions-runner missing"; exit 1; }
  touch "$ENVF"
  if grep -q '^SCOUT_REPO=' "$ENVF"; then
    sed -i "s|^SCOUT_REPO=.*|SCOUT_REPO=$H/Scout|" "$ENVF"
  else
    echo "SCOUT_REPO=$H/Scout" >> "$ENVF"
  fi
  chown "$U:$U" "$ENVF"

  # 5. Verify chris can do everything the deploy needs (sg applies the fresh
  # docker group without a re-login).
  echo "== verify as $U"
  sudo -u "$U" -H git -C "$H/Scout" fetch origin && echo "GIT-OK"
  sudo -u "$U" -H sg docker -c "docker ps -q >/dev/null" && echo "DOCKER-OK"
  sudo -u "$U" -H sg docker -c "docker pull -q ghcr.io/bg-bgi/scout-companion:latest >/dev/null" && echo "GHCR-OK"

  # 6. Recreate the stack from the new path (compose project name is pinned to
  # scout-companion, so it adopts the running containers and recreates the
  # ones whose bind-mount paths changed).
  echo "== compose up from new path"
  sudo -u "$U" -H sg docker -c "cd $H/Scout/companion && docker compose up -d --remove-orphans"

  # 7. Runner service.
  echo "== runner service"
  cd "$H/actions-runner"
  ./svc.sh install "$U" 2>/dev/null || true   # no-op if already installed
  ./svc.sh start 2>/dev/null || true
  ./svc.sh status | head -5

  echo "== DONE — runner should show Idle at github.com/BG-BGI/Scout/settings/actions/runners"
}

main "$@"
