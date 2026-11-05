#!/usr/bin/env python3
"""Apply the automatic standard grid to every saved map under sites/*/maps/*.yaml.

Offline (reads the saved yaml + pgm; no robot, no containers). Idempotent: a
map that already has a grid (auto or structural) is skipped; --refit also
extends existing AUTO grids to the saved map's current extent (a structural
grid is never touched). Uses docker/scout-skills/grid.py, loaded by path.

  scripts/auto_grid_all_maps.py --dry-run
  scripts/auto_grid_all_maps.py --cell-ft 30 --align walls
"""
import argparse
import importlib.util
import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]


def load_grid_module():
    spec = importlib.util.spec_from_file_location(
        "scout_grid", ROOT / "docker" / "scout-skills" / "grid.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def run(sites_dir, cell_ft=30.0, align="walls", dry_run=False, refit=False):
    g = load_grid_module()
    results = []
    for yml in sorted(pathlib.Path(sites_dir).glob("*/maps/*.yaml")):
        name = yml.stem
        if not g.valid_map_name(name) or name == g.SESSION:
            continue
        r = g.apply_auto_files(str(yml), str(yml.with_name(name + ".grid.json")),
                               cell_ft, align, dry_run, refit)
        results.append({"site": yml.parent.parent.name} | r)
    return results


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sites-dir", default=str(ROOT / "sites"))
    ap.add_argument("--cell-ft", type=float, default=30.0)
    ap.add_argument("--align", choices=("walls", "map"), default="walls")
    ap.add_argument("--refit", action="store_true",
                    help="also extend existing AUTO grids to the saved map extent")
    ap.add_argument("--dry-run", action="store_true", help="report only, write nothing")
    a = ap.parse_args(argv)
    results = run(a.sites_dir, a.cell_ft, a.align, a.dry_run, a.refit)
    for r in results:
        print(("[dry-run] " if a.dry_run else "") + json.dumps(r))
    if not results:
        print(f"no saved maps found under {a.sites_dir}/*/maps/*.yaml")
    return 1 if any(r["action"] == "error" for r in results) else 0


if __name__ == "__main__":
    sys.exit(main())
