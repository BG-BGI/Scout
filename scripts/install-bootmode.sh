#!/bin/bash
# install-bootmode.sh — install/refresh the scout-bootpulse + scout-bootmode
# systemd units. Idempotent: rewrites only on diff. Called once manually with
# sudo, then kept current by deploy-pi.sh (sudo -n — warns and skips where
# the runner lacks passwordless sudo).
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="${SCOUT_REPO:-$(dirname "$HERE")}"

if [ "$EUID" -ne 0 ]; then
  exec sudo --preserve-env=SCOUT_REPO "$0" "$@"
fi

changed=0
for unit in scout-bootpulse scout-bootmode; do
  src="$HERE/../systemd/$unit.service"
  dst="/etc/systemd/system/$unit.service"
  tmp=$(mktemp)
  sed "s|@REPO@|$REPO|g" "$src" > "$tmp"
  if ! cmp -s "$tmp" "$dst" 2>/dev/null; then
    install -m 644 "$tmp" "$dst"
    changed=1
  fi
  rm -f "$tmp"
done
[ "$changed" -eq 1 ] && systemctl daemon-reload
systemctl enable scout-bootpulse.service scout-bootmode.service >/dev/null
echo "bootmode units installed/enabled (repo: $REPO)"
