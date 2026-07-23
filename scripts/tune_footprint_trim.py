#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tune the footprint/body-OBB trim by ROUTING correctness, clean and under noise.

The robust footprint + body-OBB trim (docs/experiments.md 2026-07-24) rejects the
depth-noise outliers that sent the helmet to C, but a trim also shaves real edge
pixels on a sparse frame, so its level must be set by the WHOLE item set, not by the
helmet alone. This is the fast offline gate before the ~2 h Gazebo census: 33
pre-rendered resting-pose depth frames (runs/tol_frames/*_top.npy), each measured
through the PRODUCTION measure_item + classify_conservative, scored against the item's
KNOWN category. trim=0.0 reproduces the pre-change hull baseline, so the sweep carries
its own control.

    python3 scripts/tune_footprint_trim.py
    python3 scripts/tune_footprint_trim.py --sigma-mm 3 --seeds 8

Honesty: these frames are clean renders + ADDITIVE per-pixel Gaussian noise (the
make_noisy_world model), no correlated error, no no-return regions, no belt-travel or
sync. It measures the ESTIMATOR's noise robustness, not field accuracy — the Gazebo
census remains the legitimising gate for routing and organizer tolerance.
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

import src.perception as perception  # noqa: E402
from src.classification import classify_conservative  # noqa: E402

# Known category per item (docs/md/models.md, confirmed by census routing).
EXPECTED = {
    "bottle": "D", "box_300x200x200": "B", "box_400x400x300": "C", "lunchbox": "B",
    "bag": "B", "detergent": "B", "pouf": "C", "pen": "C", "plate": "D",
    "cylinder": "B", "helmet": "B",
}

FRAMES_DIR = ROOT / "runs" / "tol_frames"
SWEEP = [0.0, 0.25, 0.5, 1.0, 2.0]


def _cells():
    out = []
    for path in sorted(glob.glob(str(FRAMES_DIR / "*_top.npy"))):
        base = os.path.basename(path)[: -len("_top.npy")]
        slug = re.sub(r"_\d+$", "", base)
        if slug in EXPECTED:
            out.append((base, slug, path))
    return out


def _verdict(depth_m):
    m = perception.measure_item(depth_m)
    if m is None:
        return None
    return classify_conservative(m.dims_mm, m.k)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sigma-mm", type=float, default=3.0, help="depth noise sigma, mm")
    ap.add_argument("--seeds", type=int, default=8, help="noise draws per cell")
    ap.add_argument("--trims", type=str, default=None,
                    help="comma-separated trim%% list to sweep (default: built-in)")
    args = ap.parse_args()
    sweep = [float(t) for t in args.trims.split(",")] if args.trims else SWEEP

    cells = _cells()
    if not cells:
        print(f"no *_top.npy under {FRAMES_DIR}", file=sys.stderr)
        return 2
    print(f"{len(cells)} cells, sigma={args.sigma_mm} mm, {args.seeds} noise draws/cell\n")
    print(f"{'trim%':>6} | {'clean':>7} | {'noisy(mean)':>11} | worst-flipped cells")
    print("-" * 70)

    saved = perception.FOOTPRINT_TRIM_PCT
    try:
        for trim in sweep:
            perception.FOOTPRINT_TRIM_PCT = trim
            clean_ok = 0
            noisy_ok = np.zeros(args.seeds)
            flippers = {}
            for base, slug, path in cells:
                depth = np.load(path)
                exp = EXPECTED[slug]
                if _verdict(depth) == exp:
                    clean_ok += 1
                mask = depth > 0
                for s in range(args.seeds):
                    rng = np.random.default_rng((hash(base) & 0xFFFF) * 100 + s)
                    noisy = depth.copy()
                    noisy[mask] += rng.normal(0, args.sigma_mm / 1000.0, int(mask.sum()))
                    if _verdict(noisy) == exp:
                        noisy_ok[s] += 1
                    else:
                        flippers[slug] = flippers.get(slug, 0) + 1
            worst = sorted(flippers.items(), key=lambda kv: -kv[1])[:5]
            worst_s = ", ".join(f"{k}:{v}" for k, v in worst) or "none"
            print(f"{trim:6.2f} | {clean_ok:3d}/{len(cells)} | "
                  f"{noisy_ok.mean():5.1f}/{len(cells)} | {worst_s}")
    finally:
        perception.FOOTPRINT_TRIM_PCT = saved
    return 0


if __name__ == "__main__":
    sys.exit(main())
