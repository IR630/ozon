#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""The shift calibration procedure: run it, read it, be stopped by it.

This is the operator-facing half of `src/calibration.py` (path_to_line.md #16,
rig_decision.md §7.1). It takes what a head sees of the BARE BELT, reconstructs
each head's belt plane through the production `world_cloud_from_depth`, and
prints one line per head plus a verdict.

    python3 scripts/calibrate_shift.py runs/frames/bag_3cam
    python3 scripts/calibrate_shift.py                 # every runs/frames/*_3cam

EXIT CODE IS THE POINT. 0 means the rig may run this shift; 1 means it may not,
and the reasons are named per head. A procedure whose output is advisory is the
margin we already have — `SIDE_BELT_MARGIN_M = 8 mm` absorbing drift in silence.

ON RUNNING IT WITH GOODS IN FRAME. The proper calibration frame is bare belt.
A frame with an item on it still measures, because `fit_belt_plane` captures only
+-20 mm around the expected belt height and trims by MAD — the goods sit above
that window. `--bare-belt-check` reports how much of each head's cloud landed in
the capture strip, which is what tells an operator whether the frame was clean
enough to certify a shift with.
"""
import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from src.belt_plane import CAPTURE_M  # noqa: E402
from src.calibration import calibrate_rig, format_report  # noqa: E402
from src.constants import (  # noqa: E402
    CAMERA_SIDE_NEG_Y_POSE_M,
    CAMERA_SIDE_POS_Y_POSE_M,
    CAMERA_TOP_POSE_M,
)
from src.multiview import world_cloud_from_depth  # noqa: E402
from src.perception import BELT_TOP_Z_M, FX, FY, load_depth_png  # noqa: E402

# Every head of the shipped rig, with the pose the reconstruction uses. The top
# head is included on purpose: it is the head the census was taken on, and a
# calibration that certifies only the side heads certifies the wrong rig.
HEADS = (("top", "depth_", CAMERA_TOP_POSE_M),
         ("side_neg_y", "depth_side_neg_y_", CAMERA_SIDE_NEG_Y_POSE_M),
         ("side_pos_y", "depth_side_pos_y_", CAMERA_SIDE_POS_Y_POSE_M))

MAX_RANGE_M = 5.0


def head_frames(dump_dir, frame_index=0):
    """{head: (depth in metres, pose)} for one dumped moment."""
    dump_dir = Path(dump_dir)
    frames = {}
    for head, prefix, pose in HEADS:
        if head == "top":
            paths = sorted(p for p in dump_dir.glob("depth_*.png") if "_side_" not in p.name)
        else:
            paths = sorted(dump_dir.glob(f"{prefix}*.png"))
        if paths:
            path = paths[min(frame_index, len(paths) - 1)]
            frames[head] = (load_depth_png(str(path)), pose)
    return frames


def clouds_of(frames):
    """{head: world cloud (N, 3) m} through the production reconstruction."""
    return {head: world_cloud_from_depth(depth_m, pose, FX, FY, max_range_m=MAX_RANGE_M)
            for head, (depth_m, pose) in frames.items()}


def capture_fractions(clouds):
    """{head: share of the cloud that landed in the belt capture strip}."""
    shares = {}
    for head, pts in clouds.items():
        if not len(pts):
            shares[head] = 0.0
            continue
        near = abs(pts[:, 2] - BELT_TOP_Z_M) <= CAPTURE_M
        shares[head] = float(near.mean())
    return shares


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("dirs", nargs="*",
                        help="dump dirs (default: every runs/frames/*_3cam)")
    parser.add_argument("--frame", type=int, default=0, help="frame index in the dump")
    parser.add_argument("--bare-belt-check", action="store_true",
                        help="also report how much of each cloud fell in the strip")
    args = parser.parse_args(argv)

    dirs = [Path(d) for d in args.dirs] or sorted(
        d for d in (ROOT / "runs" / "frames").glob("*_3cam") if d.is_dir())
    if not dirs:
        print("no dump dirs given and none found under runs/frames/*_3cam")
        return 2

    refused = 0
    for dump_dir in dirs:
        frames = head_frames(dump_dir, args.frame)
        print(f"\n=== {dump_dir.name} (frame {args.frame}) ===")
        if not frames:
            print("REFUSE: no depth frames in this dump")
            refused += 1
            continue

        clouds = clouds_of(frames)
        report = calibrate_rig(clouds)
        print(format_report(report))
        if args.bare_belt_check:
            for head, share in capture_fractions(clouds).items():
                print(f"  {head}: {share * 100:.1f} % of the cloud in the capture strip")
        refused += not report.accepted

    print(f"\n{len(dirs) - refused}/{len(dirs)} dumps accepted")
    return 1 if refused else 0


if __name__ == "__main__":
    raise SystemExit(main())
