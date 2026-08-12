#!/usr/bin/env python3
"""Turn a patrol capture run into a Claude-written progress report.

Reads captures/<runstamp>/ (wpNN.jpg + manifest.yaml from patrol_capture),
sends each photo — downscaled to 512 px so a frame costs ~100-200 image
tokens — to Claude Haiku for a per-waypoint annotation, then has Claude Opus
synthesize the run report (optionally against the previous run's report for
change-over-time). Output: report.md inside the run directory.

Runs anywhere with the deps and an API key:
    pip install anthropic pillow pyyaml
    export ANTHROPIC_API_KEY=sk-ant-...
    python3 scripts/patrol_report.py captures/20260812-185125
    python3 scripts/patrol_report.py <run> --compare <previous-run>
    python3 scripts/patrol_report.py <run> --map maps/floorplan.png

Per-frame model is Haiku 4.5 (fast/cheap, fine for descriptions); the
summary uses Claude Opus 5. Cost per 10-waypoint run: a few cents.
"""

import argparse
import base64
import io
import os
import sys

import anthropic
import yaml

FRAME_MODEL = "claude-haiku-4-5"
SUMMARY_MODEL = "claude-opus-5"
LONG_EDGE = 512

FRAME_SYSTEM = """You annotate photos captured by a small ground robot \
documenting construction/interior progress. For each photo, in 2-3 concrete \
sentences: describe the visible state of the space (finishes, framing, MEP, \
furniture/fixtures), any materials or equipment present, and anything \
noteworthy for a site walk (hazards, blockages, incomplete work). No \
preamble, no speculation beyond what is visible."""

SUMMARY_SYSTEM = """You write the progress report for an autonomous robot \
patrol of a construction/interior space. You are given per-waypoint \
annotations (with map coordinates and timestamps) and optionally the \
previous run's report. Produce a markdown report: a 3-5 sentence executive \
summary, then a short per-waypoint section (## Waypoint N) keeping only \
what matters for progress tracking, then, if a previous report was \
provided, a '## Changes since last run' section. Be concrete and terse."""


def downscale_jpeg(path: str, long_edge: int = LONG_EDGE) -> bytes:
    """Return JPEG bytes resized to long_edge; original bytes if PIL absent."""
    try:
        from PIL import Image
    except ImportError:
        print("  (pillow not installed — sending full-size frame)")
        with open(path, "rb") as f:
            return f.read()
    img = Image.open(path)
    img.thumbnail((long_edge, long_edge))
    buf = io.BytesIO()
    img.convert("RGB").save(buf, format="JPEG", quality=80)
    return buf.getvalue()


def annotate_frame(client, jpeg: bytes, caption: str) -> str:
    response = client.messages.create(
        model=FRAME_MODEL,
        max_tokens=300,
        system=FRAME_SYSTEM,
        messages=[{
            "role": "user",
            "content": [
                {"type": "image", "source": {
                    "type": "base64", "media_type": "image/jpeg",
                    "data": base64.standard_b64encode(jpeg).decode()}},
                {"type": "text", "text": caption},
            ],
        }],
    )
    return next(b.text for b in response.content if b.type == "text").strip()


def summarize(client, entries, previous: str | None, map_png: bytes | None) -> str:
    lines = []
    for e in entries:
        lines.append("### Waypoint %d  (map %.1f, %.1f — %s)\n%s" % (
            e["waypoint"], e["x"], e["y"], e["time"], e["annotation"]))
    content = []
    if map_png is not None:
        content.append({"type": "image", "source": {
            "type": "base64", "media_type": "image/png",
            "data": base64.standard_b64encode(map_png).decode()}})
        content.append({"type": "text",
                        "text": "Above: top-down map of the patrol area."})
    prompt = "Per-waypoint annotations from this run:\n\n" + "\n\n".join(lines)
    if previous:
        prompt += "\n\n---\nPrevious run's report:\n\n" + previous
    content.append({"type": "text", "text": prompt})
    response = client.messages.create(
        model=SUMMARY_MODEL,
        max_tokens=4000,
        system=SUMMARY_SYSTEM,
        messages=[{"role": "user", "content": content}],
    )
    if response.stop_reason == "refusal":
        sys.exit("summary refused: %s" % response.stop_details)
    return next(b.text for b in response.content if b.type == "text").strip()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_dir", help="captures/<runstamp> directory")
    ap.add_argument("--compare", help="previous run dir (uses its report.md)")
    ap.add_argument("--map", dest="map_png", help="optional map PNG for context")
    args = ap.parse_args()

    manifest_path = os.path.join(args.run_dir, "manifest.yaml")
    if not os.path.exists(manifest_path):
        sys.exit("no manifest.yaml in %s" % args.run_dir)
    with open(manifest_path) as f:
        manifest = yaml.safe_load(f)

    client = anthropic.Anthropic()  # ANTHROPIC_API_KEY from env

    entries = []
    for cap in manifest.get("captures", []):
        if not cap.get("photo"):
            continue
        photo = os.path.join(args.run_dir, cap["photo"])
        if not os.path.exists(photo):
            continue
        pose = cap.get("pose") or {}
        caption = ("Waypoint %d of the patrol, robot at map position "
                   "(%.1f, %.1f), %s." % (cap["waypoint"],
                                          pose.get("x", 0), pose.get("y", 0),
                                          cap.get("time", "")))
        print("annotating wp%02d…" % cap["waypoint"], flush=True)
        try:
            note = annotate_frame(client, downscale_jpeg(photo), caption)
        except anthropic.APIStatusError as exc:
            note = "(annotation failed: %s)" % exc.message
        entries.append({"waypoint": cap["waypoint"],
                        "x": pose.get("x", 0), "y": pose.get("y", 0),
                        "time": cap.get("time", ""), "annotation": note})
        print("  " + note.replace("\n", " ")[:120])

    if not entries:
        sys.exit("no photos in this run")

    previous = None
    if args.compare:
        prev_path = os.path.join(args.compare, "report.md")
        if os.path.exists(prev_path):
            with open(prev_path) as f:
                previous = f.read()
        else:
            print("warning: %s has no report.md — skipping comparison" % args.compare)

    map_png = None
    if args.map_png and os.path.exists(args.map_png):
        with open(args.map_png, "rb") as f:
            map_png = f.read()

    print("writing summary (%s)…" % SUMMARY_MODEL, flush=True)
    report = summarize(client, entries, previous, map_png)

    out = os.path.join(args.run_dir, "report.md")
    with open(out, "w") as f:
        f.write("# Patrol report — %s\n\n" % os.path.basename(args.run_dir.rstrip("/")))
        f.write(report + "\n\n---\n\n## Raw per-waypoint annotations\n\n")
        for e in entries:
            f.write("**wp%02d** (%.1f, %.1f) — %s\n\n" % (
                e["waypoint"], e["x"], e["y"], e["annotation"]))
    print("report: %s" % out)


if __name__ == "__main__":
    main()
