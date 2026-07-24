#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline: does the SECOND head (arbitration combine) help routing under noise?

The live 2-cam census wedged on infrastructure, but the tolerance frames already
carry a top AND a side depth image per resting pose (runs/tol_frames/*_{top,side}.npy).
This measures the production arbitration on them without Gazebo: top-only verdict vs
top+side fused verdict, clean and under +3 mm additive noise on BOTH heads, scored
against each item's KNOWN category. It answers "1 vs 2 heads under noise with the new
role-arbitration" on real dumped frames.

    python3 scripts/probe_fusion_offline.py [--sigma-mm 3] [--seeds 8]

HONESTY: additive per-pixel Gaussian on rendered depth (the make_noisy_world model),
no correlated error, no no-return regions, no belt-travel/sync between the two frames
(the offline pair is registered by construction — a real rig is not). This is the
GEOMETRIC value of the second head under noise, not field accuracy. The 3rd head is
NOT here (tol_frames is a 2-cam dump); its marginal value stays the occlusion
antagonist result (22/23 vs 19/23) plus the redundancy argument.
"""
import argparse
import glob
import os
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.classification import classify_conservative  # noqa: E402
from src.constants import CAMERA_SIDE_NEG_Y_POSE_M  # noqa: E402
from src.multiview import crop_to_item, fuse_dims_mm, world_cloud_from_depth  # noqa: E402
from src.perception import FX, FY, measure_item  # noqa: E402

EXPECTED = {
    "bottle": "D", "box_300x200x200": "B", "box_400x400x300": "C", "lunchbox": "B",
    "bag": "B", "detergent": "B", "pouf": "C", "pen": "C", "plate": "D",
    "cylinder": "B", "helmet": "B",
}
FRAMES = ROOT / "runs" / "tol_frames"


def _cells():
    out = []
    for top in sorted(glob.glob(str(FRAMES / "*_top.npy"))):
        base = os.path.basename(top)[: -len("_top.npy")]
        side = FRAMES / f"{base}_side.npy"
        slug = re.sub(r"_\d+$", "", base)
        if slug in EXPECTED and side.exists():
            out.append((base, slug, top, str(side)))
    return out


def _verdicts(top_depth, side_depth):
    """(top_only, fused) category, or (None, None) if the top view lost the item."""
    m = measure_item(top_depth)
    if m is None:
        return None, None
    top_only = classify_conservative(m.dims_mm, m.k)
    side_cloud = world_cloud_from_depth(side_depth, CAMERA_SIDE_NEG_Y_POSE_M, FX, FY)
    side_cloud = crop_to_item(side_cloud, m.position_m, m.dims_mm)
    fused_dims = fuse_dims_mm(m.dims_mm, [side_cloud], m.position_m)
    return top_only, classify_conservative(fused_dims, m.k)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sigma-mm", type=float, default=3.0)
    ap.add_argument("--seeds", type=int, default=8)
    args = ap.parse_args()
    cells = _cells()
    if not cells:
        print(f"no top/side pairs under {FRAMES}", file=sys.stderr)
        return 2
    sig = args.sigma_mm / 1000.0

    top_clean = fused_clean = 0
    top_noisy = np.zeros(args.seeds)
    fused_noisy = np.zeros(args.seeds)
    helped, hurt = {}, {}
    for base, slug, top_p, side_p in cells:
        top, side = np.load(top_p), np.load(side_p)
        exp = EXPECTED[slug]
        t0, f0 = _verdicts(top, side)
        top_clean += (t0 == exp)
        fused_clean += (f0 == exp)
        for s in range(args.seeds):
            rng = np.random.default_rng((hash(base) & 0xFFFF) * 100 + s)
            tn = top.copy()
            tn[top > 0] += rng.normal(0, sig, int((top > 0).sum()))
            sn = side.copy()
            sn[side > 0] += rng.normal(0, sig, int((side > 0).sum()))
            t, f = _verdicts(tn, sn)
            top_noisy[s] += (t == exp)
            fused_noisy[s] += (f == exp)
            if t != exp and f == exp:
                helped[slug] = helped.get(slug, 0) + 1
            if t == exp and f != exp:
                hurt[slug] = hurt.get(slug, 0) + 1

    n = len(cells)
    print(f"{n} cells, sigma={args.sigma_mm} mm, {args.seeds} noise draws/cell\n")
    print("                 clean      noisy(mean)")
    print(f"  top-only    {top_clean:3d}/{n}      {top_noisy.mean():5.1f}/{n}")
    print(f"  top+side    {fused_clean:3d}/{n}      {fused_noisy.mean():5.1f}/{n}")
    print(f"\nunder noise, side head HELPED: {sum(helped.values())} "
          f"({', '.join(f'{k}:{v}' for k, v in sorted(helped.items(), key=lambda x: -x[1])) or 'none'})")
    print(f"under noise, side head HURT:   {sum(hurt.values())} "
          f"({', '.join(f'{k}:{v}' for k, v in sorted(hurt.items(), key=lambda x: -x[1])) or 'none'})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
