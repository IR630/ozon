#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""How often does sensor noise FLIP the CATEGORY of a pose sitting on the K edge?

WHY THIS EXISTS. The flatness gate keeps a thick round body in B by clamping its
silhouette K to exactly `ROUND_K_THRESHOLD`, and the category rule then tests that
same boundary strictly. Offline renders of all 165 census poses put 21 of them
exactly on it (`docs/experiments.md` 23.07). "Exactly on the boundary" is a
statement about arithmetic; this probe asks the PRODUCT question instead — at
depth noise a real sensor produces, how many draws send a B item to D?

WHAT IT MEASURES AND WHAT IT DOES NOT:

  * Verdict stability of ONE STILL FRAME per pose under per-pixel gaussian noise,
    through the production rule (`classify_conservative`). No belt, no crop, no
    tracker, no frame aggregation — the contour votes over ~13 frames and would
    smooth some of this. The number is an upper bound on per-frame fragility,
    not a mis-sort rate of the system.
  * A draw where the item is not detected at all counts as a WRONG outcome, not
    as a skip: on a line a lost item is not a neutral event.
  * Ten draws per (pose, sigma) give an order of magnitude, not a probability.

    python3 scripts/probe_verdict_flip.py                 # helmet and bag, 5 seeds
    python3 scripts/probe_verdict_flip.py --trials 40 --sigmas 0.001,0.002
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from probe_sensor_noise import add_depth_noise  # noqa: E402
from render_depth import load_mesh, render_depth  # noqa: E402
from spawn_orientations import orientation_quat  # noqa: E402

from src.classification import classify_conservative  # noqa: E402
from src.constants import ROUND_K_THRESHOLD  # noqa: E402
from src.perception import measure_item  # noqa: E402

# The two items whose reference category is decided by K rather than by size, and
# whose poses land on the clamp. Pouf also clamps, but its 489 mm settles the
# verdict as C before K is ever consulted, so noise cannot flip it.
EDGE_ITEMS = (("helmet", 10, "B"), ("bag", 4, "B"))
SIGMAS_FRAC = (0.0005, 0.001, 0.002)
DEFAULT_TRIALS = 10


def flips_under_noise(depth_m, expect, sigma, trials, rng):
    """How many of `trials` noisy draws leave the reference category."""
    return sum(flips_by_category(depth_m, expect, sigma, trials, rng).values())


def flips_by_category(depth_m, expect, sigma, trials, rng):
    """Wrong outcomes of `trials` noisy draws, split by WHERE they went.

    The split is the whole point. A pose can leave B two different ways and they
    have different causes: to D means the roundness rule fired (K crossed 0.8),
    to C means noise inflated a dimension past a sorter bound. Reporting only a
    flip COUNT hides which, and the first offline run of this probe was read as
    "the K clamp is fragile" when the helmet was in fact failing on SIZE — its
    297 mm width against the 320 mm bound, inflated to ~354 mm by the noise.
    """
    out = {"C": 0, "D": 0, "lost": 0}
    for _ in range(trials):
        m = measure_item(add_depth_noise(depth_m, sigma, rng))
        if m is None:
            out["lost"] += 1
            continue
        verdict = classify_conservative(m.dims_mm, m.k)
        if verdict != expect:
            out[verdict] = out.get(verdict, 0) + 1
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--trials", type=int, default=DEFAULT_TRIALS)
    parser.add_argument("--seeds", type=int, default=5)
    parser.add_argument("--sigmas", default=None,
                        help="comma-separated fractions of range")
    args = parser.parse_args(argv)
    sigmas = ([float(s) for s in args.sigmas.split(",")] if args.sigmas
              else list(SIGMAS_FRAC))

    print(f"trials={args.trials} per (pose, sigma); a lost detection counts as a flip")
    print(f"only poses whose clean K == {ROUND_K_THRESHOLD} exactly are shown\n")
    totals = {sigma: {"C": 0, "D": 0, "lost": 0, "drawn": 0} for sigma in sigmas}
    for slug, idx, expect in EDGE_ITEMS:
        mesh = load_mesh(slug)
        for seed in range(args.seeds):
            for oi in range(3):
                depth = render_depth(mesh, orientation_quat(seed, idx, oi))
                clean = measure_item(depth)
                if clean is None or clean.k != ROUND_K_THRESHOLD:
                    continue
                row = []
                for sigma in sigmas:
                    rng = np.random.default_rng([seed, idx, oi, int(sigma * 1e6)])
                    split = flips_by_category(depth, expect, sigma, args.trials, rng)
                    for key in ("C", "D", "lost"):
                        totals[sigma][key] += split.get(key, 0)
                    totals[sigma]["drawn"] += args.trials
                    n = sum(split.values())
                    row.append(f"{100 * sigma:.2f}%: {n}/{args.trials}"
                               f"(C{split['C']} D{split['D']} lost{split['lost']})")
                print(f"{slug} seed{seed} oi{oi} -> " + " | ".join(row))
    print("\ntotals over the clamped poses (WHERE the wrong outcomes went):")
    for sigma, t in totals.items():
        wrong = t["C"] + t["D"] + t["lost"]
        share = 100.0 * wrong / t["drawn"] if t["drawn"] else 0.0
        print(f"  sigma {100 * sigma:.2f}% of range: {wrong}/{t['drawn']} ({share:.0f} %) — "
              f"to C {t['C']} (size bound), to D {t['D']} (roundness), lost {t['lost']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
