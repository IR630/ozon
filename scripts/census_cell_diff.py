#!/usr/bin/env python3
"""Compare two censuses CELL BY CELL, on parsed values rather than log text.

The merge gate for a branch that touched perception asks "did any cell move?".
A `diff` of the two log trees cannot answer it: the branch changed the log
FORMAT (`classifier_node.py` prints `k=%.6f` where the baseline printed `%.3f`),
so every classifier line differs by construction and a text diff reports 100 %
of cells changed while proving nothing.

So compare what the cell MEANS: the terminal verdict, the routed category, the
measured dimensions and K — all read through `triage_matrix.parse_cell`, whose
regexes are already digit-count agnostic. One parser for the whole project
(CLAUDE.md), no second definition of what a cell is.

The verdict/category equality is the PASS criterion. Dimensions are reported,
not enforced: Gazebo is non-deterministic at a fixed seed (docs/experiments.md,
22.07), so a millimetre-level dims delta is the stand's own noise, and demanding
bit-exact dimensions would fail the branch for physics it does not control.

Usage (from repo root):
    python3 scripts/census_cell_diff.py runs/a2sweep_..._seed0 runs/lrcensus_seed0
"""

from __future__ import annotations

import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from triage_matrix import parse_cell  # noqa: E402


def read_census(logdir):
    """{(slug, orient): Cell} for every episode log in one census dir."""
    cells = {}
    for path in sorted(glob.glob(os.path.join(logdir, "matrix_*.log"))):
        cell = parse_cell(path)
        cells[(cell.slug, cell.orient)] = cell
    return cells


def _dims_delta_mm(before, after):
    """Largest per-axis difference in mm, or None if either side has no dims."""
    if before is None or after is None:
        return None
    return max(abs(a - b) for a, b in zip(after, before))


def compare(baseline_dir, candidate_dir):
    """(moved, dims_rows, missing) — the cells that changed and by how much."""
    baseline = read_census(baseline_dir)
    candidate = read_census(candidate_dir)

    moved, dims_rows, missing = [], [], []
    for key in sorted(baseline.keys() | candidate.keys()):
        before, after = baseline.get(key), candidate.get(key)
        if before is None or after is None:
            missing.append((key, "candidate" if after is None else "baseline"))
            continue
        if before.verdict != after.verdict or before.category != after.category:
            moved.append((key, before, after))
        delta = _dims_delta_mm(before.dims_mm, after.dims_mm)
        if delta is not None:
            dims_rows.append((key, before.dims_mm, after.dims_mm, delta))
    return moved, dims_rows, missing


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("baseline", help="census log dir taken as the reference")
    ap.add_argument("candidate", help="census log dir under test")
    args = ap.parse_args()

    moved, dims_rows, missing = compare(args.baseline, args.candidate)
    print(f"=== cell diff: {args.baseline} -> {args.candidate} ===")

    for (slug, oi), side in missing:
        print(f"  MISSING in {side}: {slug} oi={oi}")

    if moved:
        print(f"  MOVED CELLS: {len(moved)}")
        for (slug, oi), before, after in moved:
            print(f"    {slug} oi={oi}: "
                  f"{before.verdict}/{before.category} -> {after.verdict}/{after.category}")
    else:
        print("  MOVED CELLS: 0 — every shared cell kept its verdict and category")

    if dims_rows:
        worst = max(dims_rows, key=lambda row: row[3])
        (slug, oi), before_dims, after_dims, delta = worst
        print(f"  dims compared on {len(dims_rows)} cells; "
              f"largest per-axis delta {delta} mm on {slug} oi={oi} "
              f"({'x'.join(map(str, before_dims))} -> {'x'.join(map(str, after_dims))})")
    else:
        print("  dims compared on 0 cells (no perception line on either side)")

    return 1 if (moved or missing) else 0


if __name__ == "__main__":
    raise SystemExit(main())
