#!/usr/bin/env python3
"""D455 on-chip self-calibration (KEEPER — recurring maintenance, not a bench rig).

Stereo extrinsic calibration drifts with temperature cycles and knocks; this
runs the camera's built-in self-cal and reports the health score. Flash write
is gated behind an explicit prompt — until confirmed, nothing persists.

    docker compose stop robot
    docker compose run --rm --no-deps robot python3 /ros_ws/src/scripts/camera_selfcal.py
    docker compose start robot

Aim at a scene with texture 0.5-2 m away (NOT a blank wall — self-cal wants
features). Health score: |h| < 0.25 good, 0.25-0.75 degraded but usable,
> 0.75 recalibration was genuinely needed (keep the result).

--tare <mm>: tare calibration against a flat target at a known ground-truth
distance (tape-measured, perpendicular). Only needed if absolute distance is
biased; run plain self-cal first.

IMU flash recalibration is DELIBERATELY not provided: gyro bias is estimated
online (gyro_calibrator -> /imu/data, the only IMU path the EKF consumes),
rs-imu-calibration corrects bias-not-scale anyway, and flash writes sit next
to the documented Motion-Module wedge hazard. Do not add it.
"""

import argparse
import sys
import time

import pyrealsense2 as rs

# Speed-mode profile the on-chip routine expects (per Intel's example).
CAL_PROFILE = (256, 144, 90)
OCC_JSON = '{"calib type": 0, "speed": 3, "scan parameter": 0, "white wall mode": 0}'


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tare", type=int, metavar="MM",
                    help="tare calibration: ground-truth distance to flat target, mm")
    ap.add_argument("--timeout", type=int, default=30000, help="ms")
    args = ap.parse_args()

    pipe = rs.pipeline()
    cfg = rs.config()
    w, h, fps = CAL_PROFILE
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    try:
        profile = pipe.start(cfg)
    except RuntimeError as exc:
        sys.exit("Cannot open the camera (%s).\n"
                 "Is the robot service holding it? Run: docker compose stop robot" % exc)

    try:
        for _ in range(30):  # warm up the stream; occ needs live frames
            pipe.wait_for_frames()
        dev = profile.get_device()
        cal = rs.auto_calibrated_device(dev)

        old_table = cal.get_calibration_table()
        print("Current table: %d bytes. Running %s…" %
              (len(old_table), "tare calibration" if args.tare else "on-chip self-calibration"))

        if args.tare:
            new_table, health = cal.run_tare_calibration(
                args.tare, "", args.timeout)
        else:
            new_table, health = cal.run_on_chip_calibration(
                OCC_JSON, args.timeout)

        try:
            hval = float(health[0]) if hasattr(health, "__len__") else float(health)
        except (TypeError, ValueError):
            hval = float("nan")
        print("Health score: %.4f" % hval)
        a = abs(hval)
        if a < 0.25:
            print("  |h| < 0.25 — optics were already good; new table is a refinement.")
        elif a <= 0.75:
            print("  0.25-0.75 — calibration had drifted; applying helps.")
        else:
            print("  > 0.75 — significant drift; definitely keep the result.")

        cal.set_calibration_table(new_table)  # applied to this session only
        print("New table applied to the RUNNING session (not yet in flash).")

        ans = input("Write to flash? Old table is unrecoverable afterwards. [y/N] ")
        if ans.strip().lower() == "y":
            cal.write_calibration()
            print("Written to flash.")
        else:
            print("NOT written — device reverts to the old table on power cycle.")
    except RuntimeError as exc:
        sys.exit("Calibration failed: %s\n"
                 "Common causes: blank/too-close scene (needs texture 0.5-2 m), "
                 "motion during the run." % exc)
    finally:
        pipe.stop()
        time.sleep(0.5)


if __name__ == "__main__":
    main()
