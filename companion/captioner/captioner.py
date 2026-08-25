#!/usr/bin/env python3
"""Batch anomaly captioner (confined-space inspection F2; WORLDMODEL.md gate 4
repointed to post-run batch — no live CPU-fit risk).

Reads an inspection run's .mcap directly (no ROS anywhere in this image),
captions sampled color frames with Florence-2-base on CPU, keyword-flags
inspection anomalies (standing water, debris, blockage, damage...), and writes
anomalies.json beside the bag. Reviewer opens the .mcap in Foxglove and jumps
to the flagged timestamps.

Usage: captioner.py <run_dir> [--every-s 2.0] [--all-captions]
  <run_dir> = a /captures/inspection/<UTC>/ directory containing *.mcap
Weights are baked into the image at build — runs fully offline.
"""
import argparse
import json
import sys
from io import BytesIO
from pathlib import Path

from PIL import Image

COLOR_TOPIC = "/camera/camera/color/image_raw/compressed"
ODOM_TOPIC = "/odom"

# Flag a caption when any of these appear. Tuned for construction
# confined-space inspection; extend freely — it is a grep, not a model.
ANOMALY_TERMS = (
    "water standing puddle wet flood leak drip "
    "debris rubble trash dirt pile obstruction blocked blockage "
    "crack cracked damage damaged broken collapse hole gap "
    "corrosion rust exposed wire cable pipe burst"
).split()


def _load_model():
    import torch  # deferred: ~seconds of import
    from transformers import AutoModelForCausalLM, AutoProcessor

    model = AutoModelForCausalLM.from_pretrained(
        "microsoft/Florence-2-base", trust_remote_code=True,
        torch_dtype=torch.float32)
    processor = AutoProcessor.from_pretrained(
        "microsoft/Florence-2-base", trust_remote_code=True)
    return model, processor


def _caption(model, processor, img: Image.Image) -> str:
    task = "<MORE_DETAILED_CAPTION>"
    inputs = processor(text=task, images=img, return_tensors="pt")
    ids = model.generate(
        input_ids=inputs["input_ids"], pixel_values=inputs["pixel_values"],
        max_new_tokens=256, num_beams=3, do_sample=False)
    text = processor.batch_decode(ids, skip_special_tokens=False)[0]
    out = processor.post_process_generation(
        text, task=task, image_size=img.size)
    return out.get(task, "").strip()


def _iter_frames(mcap_paths, every_s):
    """Yield (t_sec, PIL image, nearest odom xy) sampled every_s apart.
    Streams the mcap(s); keeps only the latest odom in memory."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    last_odom = None
    next_t = 0.0
    for p in sorted(mcap_paths):
        with open(p, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for _schema, channel, message, msg in reader.iter_decoded_messages(
                    topics=[COLOR_TOPIC, ODOM_TOPIC]):
                t = message.log_time / 1e9
                if channel.topic == ODOM_TOPIC:
                    pos = msg.pose.pose.position
                    last_odom = [round(pos.x, 2), round(pos.y, 2)]
                    continue
                if t < next_t:
                    continue
                next_t = t + every_s
                img = Image.open(BytesIO(bytes(msg.data))).convert("RGB")
                yield t, img, last_odom


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--every-s", type=float, default=2.0)
    ap.add_argument("--all-captions", action="store_true",
                    help="write every caption, not just flagged ones")
    args = ap.parse_args()

    mcaps = list(args.run_dir.glob("**/*.mcap"))
    if not mcaps:
        sys.exit(f"no .mcap under {args.run_dir}")

    print("loading Florence-2-base (CPU)...", flush=True)
    model, processor = _load_model()

    results, flagged = [], 0
    for t, img, odom_xy in _iter_frames(mcaps, args.every_s):
        cap = _caption(model, processor, img)
        low = cap.lower()
        flags = [w for w in ANOMALY_TERMS if w in low]
        if flags:
            flagged += 1
        if flags or args.all_captions:
            results.append({
                "t": round(t, 2),
                "flags": flags,
                "caption": cap,
                "odom_xy": odom_xy,
            })
        print(f"t={t:.1f} {'⚑' if flags else ' '} {cap[:90]}", flush=True)

    out = args.run_dir / "anomalies.json"
    out.write_text(json.dumps({
        "run": args.run_dir.name,
        "sampled_every_s": args.every_s,
        "frames_captioned": len(results) if args.all_captions else None,
        "flagged": flagged,
        "anomaly_terms": ANOMALY_TERMS,
        "results": results,
    }, indent=1))
    print(f"\n{flagged} flagged -> {out}")


if __name__ == "__main__":
    main()
