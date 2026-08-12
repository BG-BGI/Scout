#!/usr/bin/env python3
"""Turn a patrol capture run into an LLM-written progress report.

Reads captures/<runstamp>/ (wpNN.jpg + manifest.yaml from patrol_capture),
sends each photo — downscaled to 512 px so a frame costs ~100-200 image
tokens — to a fast vision model for a per-waypoint annotation, then has a
stronger model synthesize the run report (optionally against the previous
run's report for change-over-time). Output: report.md in the run directory.

Backend: Vercel AI Gateway (OpenAI-compatible REST), one key for any model:
    pip install requests pillow pyyaml
    export AI_GATEWAY_API_KEY=vck_...
    python3 scripts/patrol_report.py captures/20260812-185125
    python3 scripts/patrol_report.py <run> --compare <previous-run>
    python3 scripts/patrol_report.py --list-models        # live catalog
    python3 scripts/patrol_report.py <run> \
        --frame-model google/gemini-2.5-flash \
        --summary-model anthropic/claude-sonnet-4.5

Cost per 10-waypoint run: about a cent with the defaults.
"""

import argparse
import base64
import io
import json
import os
import sys

import requests
import yaml

GATEWAY = os.environ.get("AI_GATEWAY_BASE_URL", "https://ai-gateway.vercel.sh/v1")
FRAME_MODEL = "google/gemini-2.5-flash"
SUMMARY_MODEL = "anthropic/claude-sonnet-4.5"
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


def _key() -> str:
    key = os.environ.get("AI_GATEWAY_API_KEY")
    if not key:
        sys.exit("AI_GATEWAY_API_KEY is not set")
    return key


def chat(model: str, system: str, user_content, max_tokens: int) -> str:
    """One OpenAI-compatible chat completion against the AI Gateway."""
    resp = requests.post(
        GATEWAY + "/chat/completions",
        headers={"Authorization": "Bearer " + _key(),
                 "Content-Type": "application/json"},
        json={
            "model": model,
            "max_tokens": max_tokens,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": user_content},
            ],
        },
        timeout=180,
    )
    if resp.status_code != 200:
        raise RuntimeError("gateway %d for %s: %s"
                           % (resp.status_code, model, resp.text[:300]))
    return resp.json()["choices"][0]["message"]["content"].strip()


def image_part(data: bytes, media: str = "image/jpeg") -> dict:
    return {"type": "image_url", "image_url": {
        "url": "data:%s;base64,%s"
               % (media, base64.standard_b64encode(data).decode())}}


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


def list_models():
    resp = requests.get(GATEWAY + "/models",
                        headers={"Authorization": "Bearer " + _key()},
                        timeout=30)
    resp.raise_for_status()
    for m in sorted(resp.json().get("data", []), key=lambda m: m["id"]):
        print(m["id"])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_dir", nargs="?", help="captures/<runstamp> directory")
    ap.add_argument("--compare", help="previous run dir (uses its report.md)")
    ap.add_argument("--map", dest="map_png", help="optional map PNG for context")
    ap.add_argument("--frame-model", default=FRAME_MODEL)
    ap.add_argument("--summary-model", default=SUMMARY_MODEL)
    ap.add_argument("--list-models", action="store_true",
                    help="print the gateway's model catalog and exit")
    args = ap.parse_args()

    if args.list_models:
        list_models()
        return
    if not args.run_dir:
        ap.error("run_dir is required (or use --list-models)")

    manifest_path = os.path.join(args.run_dir, "manifest.yaml")
    if not os.path.exists(manifest_path):
        sys.exit("no manifest.yaml in %s" % args.run_dir)
    with open(manifest_path) as f:
        manifest = yaml.safe_load(f)

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
        print("annotating wp%02d (%s)…" % (cap["waypoint"], args.frame_model),
              flush=True)
        try:
            note = chat(args.frame_model, FRAME_SYSTEM,
                        [image_part(downscale_jpeg(photo)),
                         {"type": "text", "text": caption}],
                        # generous cap: reasoning models (Gemini 2.5) spend
                        # completion tokens on internal thinking first
                        max_tokens=1500)
        except (RuntimeError, requests.RequestException, KeyError) as exc:
            note = "(annotation failed: %s)" % exc
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

    content = []
    if args.map_png and os.path.exists(args.map_png):
        with open(args.map_png, "rb") as f:
            content.append(image_part(f.read(), "image/png"))
        content.append({"type": "text",
                        "text": "Above: top-down map of the patrol area."})
    lines = ["### Waypoint %d  (map %.1f, %.1f — %s)\n%s" % (
        e["waypoint"], e["x"], e["y"], e["time"], e["annotation"])
        for e in entries]
    prompt = "Per-waypoint annotations from this run:\n\n" + "\n\n".join(lines)
    if previous:
        prompt += "\n\n---\nPrevious run's report:\n\n" + previous
    content.append({"type": "text", "text": prompt})

    print("writing summary (%s)…" % args.summary_model, flush=True)
    report = chat(args.summary_model, SUMMARY_SYSTEM, content, max_tokens=4000)

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
