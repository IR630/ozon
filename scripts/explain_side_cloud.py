# -*- coding: utf-8 -*-
"""What is actually inside a side head's cloud after the crop — by the numbers.

WHY. The miscalibrated census reads items as 740x505x102 mm against a ~303 mm
truth, on every frame, with the y extent pinned at the belt's own 500 mm width.
Two exact geometric models of the calibration error predict the belt lifts only
2.3-4.0 mm against the 5 mm rejection margin, i.e. it must NOT leak — and it
leaks anyway. Rather than keep refining the model, look at the cloud.

The point of this script is that it uses the SHIPPING functions — the same
backprojection, the same crop, the same nominal poses the node believes — so
what it prints is what the node saw, not a reimplementation that could be wrong
in its own way.

    bash scripts/dump_item_frame.sh bottle /tmp/frames_bottle   # with --side
    python3 scripts/explain_side_cloud.py /tmp/frames_bottle --item 1.9 0.0 \
        --dims 303 94 91 [--png /tmp/side_cloud.png]
"""
import argparse
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))

from src.constants import (  # noqa: E402
    CAMERA_SIDE_NEG_Y_POSE_M,
    CAMERA_SIDE_POS_Y_POSE_M,
)
from src.multiview import crop_to_item, world_cloud_from_depth  # noqa: E402
from src.perception import FX, FY, BELT_TOP_Z_M  # noqa: E402

HEADS = (("depth_side_neg_y", CAMERA_SIDE_NEG_Y_POSE_M),
         ("depth_side_pos_y", CAMERA_SIDE_POS_Y_POSE_M))

# The belt is 0.5 m wide and its top is BELT_TOP_Z_M; anything cropped in that
# lands ON those coordinates is the scene, not the goods. Named here so the
# report says "belt" rather than "some points".
BELT_HALF_WIDTH_M = 0.25


def describe(pts_m, label):
    """One line per head: how much survived the crop and where it sits."""
    if not len(pts_m):
        return f"{label:16} no points survive the crop"
    z_mm = (pts_m[:, 2] - BELT_TOP_Z_M) * 1000.0
    on_belt = np.mean(np.abs(pts_m[:, 1]) <= BELT_HALF_WIDTH_M) * 100.0
    hugging = np.mean(z_mm < 20.0) * 100.0
    return (f"{label:16} {len(pts_m):7d} pts | "
            f"x-span {np.ptp(pts_m[:, 0]) * 1000:6.0f} mm | "
            f"y-span {np.ptp(pts_m[:, 1]) * 1000:6.0f} mm | "
            f"z above belt {z_mm.min():6.1f}..{z_mm.max():6.1f} mm | "
            f"{on_belt:5.1f}% over the belt, {hugging:5.1f}% within 20 mm of it")


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("frames_dir")
    parser.add_argument("--item", nargs=3, type=float, required=True,
                        metavar=("X", "Y", "Z"), help="item position, metres")
    parser.add_argument("--dims", nargs=3, type=float, required=True,
                        metavar=("A", "B", "C"), help="TOP-head dims, mm — these size the crop")
    parser.add_argument("--frame", type=int, default=0)
    parser.add_argument("--png", help="scatter of the surviving points, xy and xz")
    args = parser.parse_args(argv)

    import cv2

    frames = Path(args.frames_dir)
    kept_all = []
    for kind, pose in HEADS:
        path = frames / f"{kind}_{args.frame:03d}.png"
        if not path.exists():
            print(f"{kind:16} MISSING {path} — was the dump run with --side?")
            continue
        depth_m = cv2.imread(str(path), cv2.IMREAD_UNCHANGED).astype(np.float64) / 1000.0
        pts = world_cloud_from_depth(depth_m, pose, FX, FY)
        kept = crop_to_item(pts, args.item, args.dims)
        print(describe(pts, kind + " raw"))
        print(describe(kept, kind + " cropped"))
        kept_all.append(kept)

    if not kept_all:
        return 1
    pts = np.vstack([k for k in kept_all if len(k)]) if any(len(k) for k in kept_all) \
        else np.empty((0, 3))
    print(describe(pts, "BOTH cropped"))
    if len(pts):
        print(f"the fused shadow would therefore be about "
              f"{np.ptp(pts[:, 0]) * 1000:.0f} x {np.ptp(pts[:, 1]) * 1000:.0f} mm")

    if args.png and len(pts):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, (top, side) = plt.subplots(2, 1, figsize=(9, 7))
        top.scatter(pts[:, 0], pts[:, 1], s=0.2)
        top.axhline(BELT_HALF_WIDTH_M, color="r", lw=0.8)
        top.axhline(-BELT_HALF_WIDTH_M, color="r", lw=0.8)
        top.set_title("cropped side points, XY (red = belt edges)")
        side.scatter(pts[:, 0], (pts[:, 2] - BELT_TOP_Z_M) * 1000.0, s=0.2)
        side.axhline(5.0, color="r", lw=0.8)
        side.set_title("same points, height above belt (red = 5 mm rejection margin)")
        fig.tight_layout()
        fig.savefig(args.png, dpi=110)
        print(f"wrote {args.png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
