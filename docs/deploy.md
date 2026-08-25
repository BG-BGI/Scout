# Deploy & ops via GitHub Actions (ADR-0027)

Everything after a merge is buttons in the Actions tab. Images build on
GitHub-hosted runners and land in GHCR; the Pi and companion pull them via
their own self-hosted runners. No Mac builds, no `docker save`/scp, no ssh.

## The loop

1. Merge/push. `ci.yml` gates; the image workflows (path-filtered) push
   `ghcr.io/bg-bgi/*:<sha>` + `:<branch>` (+ `:latest` from main only).
2. Actions → **deploy** → Run workflow:
   - `target`: both / pi / companion
   - `branch`: blank = the pins in repo variables `PI_BRANCH` / `COMPANION_BRANCH`
   - `robot_stationary`: **must be checked for pi/both** — the deploy restarts
     the drivetrain driver, and only you can see the robot.
   The run first (re)builds images at the exact commit, so path-filter gaps
   can't deploy stale images, then runs `scripts/deploy-pi.sh` /
   `companion/update.sh` on the devices. deploy-pi.sh owns the ADR-0005 stamp
   guard: image-build-id mismatch or branch change ⇒ automatic `down -v`
   (build + install volumes wiped as a pair) before `build_package` + `up`.
3. Verify per the run's smoke output, then the usual: webui panels, Foxglove
   diagnostics rows, `ops` → logs robot.

## Switching branches

```sh
gh variable set PI_BRANCH -b main -R BG-BGI/Scout
gh variable set COMPANION_BRANCH -b main -R BG-BGI/Scout
```
then dispatch **deploy**. One-off test of a branch: leave the variables alone
and fill the `branch` input. Rollback = deploy the older commit's branch state
(images are sha-pinned, so a re-deploy reproduces it exactly).

## Container ops

Actions → **ops**: ps / logs / restart / stop / start / up / down, optional
service list and profiles. Caveats unchanged from hands-on operation:
`stop`/`down` is a **coast, not a brake**; a nav goal survives its client —
`restart nav2` clears it; `up` with the `explore` profile starts frontier
exploration = motion. `down -v` is not offered — that decision belongs to
deploy-pi.sh's stamp guard.

## One-time device provisioning

On each device (Pi and companion):

1. Repo → Settings → Actions → Runners → New self-hosted runner (Linux,
   arm64 for the Pi / x64 for the companion). Run the download/config
   snippet as the normal user; at `config.sh`, set **labels**: `pi` on the
   Pi, `companion` on the companion.
2. Point it at the existing clone and install as a service:
   ```sh
   echo "SCOUT_REPO=$HOME/Desktop/Scout" >> ~/actions-runner/.env   # Pi path; companion: its clone
   sudo ./svc.sh install $USER && sudo ./svc.sh start
   ```
   The runner user must be in the `docker` group.
3. Pi only: `docker login ghcr.io` once with the read:packages PAT (same
   pattern as companion/host-setup.md) — or make the GHCR packages public
   (repo is public) and skip logins everywhere.

Repo settings (once, admin):

- Variables: `PI_BRANCH`, `COMPANION_BRANCH`.
- Actions → General: "Require approval for all outside collaborators" on
  fork PR runs. Self-hosted labels are used ONLY by dispatch-only workflows
  (deploy, ops) — keep it that way: on a public repo, putting `self-hosted`
  in a `push`/`pull_request` workflow hands the robot to anyone who can get
  a workflow to run.

## Fallbacks

- `scripts/deploy-pi.sh <branch>` by hand over ssh — same path the workflow
  takes (never while the robot is driving).
- `scripts/scout-switch.sh` — the old build-ON-the-Pi flow; offline use only.
- `companion/update.sh` — no args: ff-pull + pull current pins.
- USB sneakernet per companion/host-setup.md if GHCR is unreachable.
