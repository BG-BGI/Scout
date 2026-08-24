#!/usr/bin/env python3
"""Migrate the flat maps/ + captures/ pools into the sites/ layout (ADR-0023).

Run once on the Pi after pulling the sites change, from the repo root:

    python3 scripts/migrate_sites.py

Creates `sites/default/{maps,captures}`, moves everything out of `maps/` and
`captures/` (except .gitkeep) into it, writes `site.json` (default_map = the
sole .posegraph basename if exactly one exists), and points the
`sites/active` symlink at it. Idempotent — a valid `sites/active` means
already migrated and the script exits 0 without touching anything.

Companion box equivalent (documented in companion/docker-compose.yaml):
    mkdir -p data/sites/default && ln -s default data/sites/active
    docker run --rm -v scout-companion_rtabmap_db:/src \
        -v "$PWD/data/sites/default":/dst alpine cp /src/rtabmap.db /dst/
"""

import json
import os
import sys
import time


def main(argv):
    root = argv[1] if len(argv) > 1 else '.'
    sites = os.path.join(root, 'sites')
    active = os.path.join(sites, 'active')

    if os.path.islink(active) and os.path.isdir(active):
        print(f'already migrated: {active} -> {os.readlink(active)}')
        return 0

    default = os.path.join(sites, 'default')
    default_maps = os.path.join(default, 'maps')
    default_caps = os.path.join(default, 'captures')
    os.makedirs(default_maps, exist_ok=True)
    os.makedirs(default_caps, exist_ok=True)

    moved = 0
    for src_dir, dst_dir in ((os.path.join(root, 'maps'), default_maps),
                             (os.path.join(root, 'captures'), default_caps)):
        if not os.path.isdir(src_dir):
            continue
        for name in sorted(os.listdir(src_dir)):
            if name == '.gitkeep':
                continue
            try:
                os.replace(os.path.join(src_dir, name),
                           os.path.join(dst_dir, name))
            except PermissionError:
                # Container-created dirs (captures/*) are root-owned; rename
                # needs write on the source parent. Safe to rerun — already-
                # moved entries are skipped by the loop.
                print(f'permission denied moving {src_dir}/{name} — '
                      'rerun as: sudo python3 scripts/migrate_sites.py',
                      file=sys.stderr)
                return 1
            moved += 1
            print(f'moved {src_dir}/{name} -> {dst_dir}/')

    posegraphs = sorted(f[:-len('.posegraph')]
                        for f in os.listdir(default_maps)
                        if f.endswith('.posegraph'))
    site = {
        'version': 1,
        'display_name': 'default',
        # Exactly one saved map = obviously the site's map; several = the
        # operator picks (webui Save map sets it, or edit site.json by hand).
        'default_map': posegraphs[0] if len(posegraphs) == 1 else None,
        'slam_mode': 'auto',
        'map_start_pose': [0.0, 0.0, 0.0],
        'created': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
    }
    with open(os.path.join(default, 'site.json'), 'w') as f:
        json.dump(site, f, indent=2)

    if os.path.islink(active):
        os.unlink(active)  # broken link from a half-finished run
    os.symlink('default', active)

    print(f"migrated {moved} entries into {default}; active -> default; "
          f"default_map = {site['default_map']!r} "
          f'(posegraphs found: {", ".join(posegraphs) or "none"})')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
