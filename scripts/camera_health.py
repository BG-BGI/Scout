#!/usr/bin/env python3
"""D455 depth health instrument (KEEPER — recurring maintenance, not a bench rig).

Runs inside the container (pyrealsense2 lives in /usr/lib/python3/dist-packages).
The robot service must NOT hold the camera:

    docker compose stop robot
    docker compose run --rm --no-deps robot python3 /ros_ws/src/scripts/camera_health.py --plane
    docker compose start robot

Modes:
  --plane   (default) Point the camera at a flat wall ~1 m away, keep still.
            30 frames -> central 40% ROI -> least-squares plane fit.
            Reports fill rate, Z-RMS (mm), and subpixel RMS.
            Subpixel < 0.1 = well calibrated; > 0.2 = run camera_selfcal.py.
  --watch   1 Hz live readout of nearest / farthest valid depth (percentile
            filtered). This is the MinZ / MaxZ bench instrument for the
            disparity-shift setting: slide a textured box in until the cloud
            collapses (MinZ), face a wall at ~3 m and read the cutoff (MaxZ).

Both modes load the deployed advanced-mode preset first (--preset, default the
repo's d455_scout_preset.json) so measurements reflect the running config —
including disparity shift. --no-preset measures factory behaviour instead.
"""

import argparse
import sys
import time

import numpy as np
import pyrealsense2 as rs

BASELINE_MM = 95.0  # D455 stereo baseline
GLOBAL_TIME = rs.timestamp_domain.global_time


def start_pipeline(width, height, fps, preset_path):
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    try:
        profile = pipe.start(cfg)
    except RuntimeError as exc:
        sys.exit("Cannot open the camera (%s).\n"
                 "Is the robot service holding it? Run: docker compose stop robot" % exc)
    dev = profile.get_device()
    if preset_path:
        try:
            adv = rs.rs400_advanced_mode(dev)
            if not adv.is_enabled():
                adv.toggle_advanced_mode(True)
                time.sleep(4)  # device re-enumerates
                pipe.stop()
                return start_pipeline(width, height, fps, preset_path)
            with open(preset_path) as f:
                adv.load_json(f.read())
            print("Preset applied: %s" % preset_path)
        except Exception as exc:  # noqa: BLE001 — preset is optional, say so and go on
            print("WARNING: preset not applied (%s) — measuring factory behaviour" % exc)
    return pipe, profile


def get_depth(pipe, depth_scale):
    frames = pipe.wait_for_frames()
    d = frames.get_depth_frame()
    z = np.asanyarray(d.get_data()).astype(np.float32) * depth_scale  # metres
    return z


def plane_mode(pipe, profile, n_frames):
    sensor = profile.get_device().first_depth_sensor()
    scale = sensor.get_depth_scale()
    intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    print("Point at a flat wall ~1 m away, hold still. Capturing %d frames…" % n_frames)
    for _ in range(15):  # warmup / AE settle
        pipe.wait_for_frames()

    h, w = None, None
    fills, rms_mms, subpixes, dists = [], [], [], []
    for _ in range(n_frames):
        z = get_depth(pipe, scale)
        if h is None:
            h, w = z.shape
            r0, r1 = int(h * 0.3), int(h * 0.7)
            c0, c1 = int(w * 0.3), int(w * 0.7)
            us, vs = np.meshgrid(np.arange(c0, c1), np.arange(r0, r1))
        roi = z[r0:r1, c0:c1]
        valid = roi > 0
        fills.append(valid.mean())
        if valid.sum() < 500:
            continue
        zz = roi[valid]
        xx = (us[valid] - intr.ppx) / intr.fx * zz
        yy = (vs[valid] - intr.ppy) / intr.fy * zz
        pts = np.stack([xx, yy, zz], axis=1)
        centroid = pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts - centroid, full_matrices=False)
        normal = vt[2]
        dist = (pts - centroid) @ normal
        rms_m = float(np.sqrt((dist ** 2).mean()))
        zmean = float(zz.mean())
        rms_mms.append(rms_m * 1000.0)
        # disparity(px) = fx * B / z ; d(disp) = fx*B/z^2 * dz
        subpixes.append(rms_m * intr.fx * (BASELINE_MM / 1000.0) / zmean ** 2)
        dists.append(zmean)

    if not rms_mms:
        sys.exit("No usable frames — too many holes. Closer wall, more light, or laser up.")
    fill, rms, sub, dist = map(lambda a: float(np.median(a)), (fills, rms_mms, subpixes, dists))
    print("\nROI %dx%d (central 40%%), median over %d frames at %.2f m:" %
          (c1 - c0, r1 - r0, len(rms_mms), dist))
    print("  fill rate     %.1f %%" % (fill * 100))
    print("  Z-RMS         %.2f mm  (%.2f %% of distance)" % (rms, rms / (dist * 10)))
    print("  subpixel RMS  %.3f" % sub)
    if sub < 0.1:
        print("  VERDICT: well calibrated (< 0.1)")
    elif sub <= 0.2:
        print("  VERDICT: acceptable (0.1-0.2); re-test before recalibrating")
    else:
        print("  VERDICT: RECALIBRATE (> 0.2) — run camera_selfcal.py")


def watch_mode(pipe, profile):
    scale = profile.get_device().first_depth_sensor().get_depth_scale()
    print("1 Hz min/max valid depth (0.5 / 99.5 percentiles). Ctrl-C to stop.")
    try:
        while True:
            t0 = time.monotonic()
            z = get_depth(pipe, scale)
            v = z[z > 0]
            if v.size < 100:
                print("  (no depth)")
            else:
                print("  near %.3f m   far %.3f m   valid %5.1f %%" %
                      (np.percentile(v, 0.5), np.percentile(v, 99.5),
                       100.0 * v.size / z.size))
            time.sleep(max(0.0, 1.0 - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--watch", action="store_true", help="live min/max range readout")
    ap.add_argument("--plane", action="store_true", help="plane-fit RMS test (default)")
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--preset", default="/ros_ws/src/scout/config/d455_scout_preset.json")
    ap.add_argument("--no-preset", action="store_true")
    args = ap.parse_args()

    preset = None if args.no_preset else args.preset
    pipe, profile = start_pipeline(848, 480, 30, preset)
    try:
        if args.watch:
            watch_mode(pipe, profile)
        else:
            plane_mode(pipe, profile, args.frames)
    finally:
        pipe.stop()


if __name__ == "__main__":
    main()
