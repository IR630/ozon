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
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from triage_matrix import PERCEPTION_RE, parse_cell  # noqa: E402


def median_dims_mm(path):
    """Per-axis MEDIAN of every perception line in one cell log, or None.

    NOT the last line, which is an arbitrary late frame rather than a summary.

    READ THIS BEFORE TRUSTING ANY DIMS NUMBER OUT OF A CENSUS LOG. The episode
    log does not contain the frames the routing was computed from.
    `run_skeleton.sh` kills the launch the moment the verdict is in, which
    truncates the nodes' stdout, so a cell whose classifier line reports `n=16`
    can leave exactly ONE perception line behind — and which one survived is a
    flush artefact. Measured case, `bottle` oi=2 on seed 3: baseline kept a
    single 200x90x87 line, the branch a single 303x89x88 one, both with the item
    at x~2.0 and both routed D. That 103 mm is a difference between two
    arbitrarily surviving frames, NOT between two measurements of the same thing.

    The median is still the right reduction — it is what `ItemAggregator` applies
    to route the item, and on cells where several lines did survive it removes
    the tail. It cannot repair a cell that flushed only one line, which is
    precisely why dims are reported here and never gated.
    """
    with open(path, encoding="utf-8", errors="replace") as handle:
        text = handle.read()
    dims = [(int(m["w"]), int(m["h"]), int(m["d"])) for m in PERCEPTION_RE.finditer(text)]
    if not dims:
        return None
    return tuple(int(statistics.median(axis)) for axis in zip(*dims))


def read_census(logdir):
    """{(slug, orient): (Cell, median dims)} for every episode log in one dir."""
    cells = {}
    for path in sorted(glob.glob(os.path.join(logdir, "matrix_*.log"))):
        cell = parse_cell(path)
        cells[(cell.slug, cell.orient)] = (cell, median_dims_mm(path))
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
        before_entry, after_entry = baseline.get(key), candidate.get(key)
        if before_entry is None or after_entry is None:
            missing.append((key, "candidate" if after_entry is None else "baseline"))
            continue
        before, before_dims = before_entry
        after, after_dims = after_entry
        if before.verdict != after.verdict or before.category != after.category:
            moved.append((key, before, after))
        delta = _dims_delta_mm(before_dims, after_dims)
        if delta is not None:
            dims_rows.append((key, before_dims, after_dims, delta))
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
        deltas = sorted(row[3] for row in dims_rows)
        print(f"  dims (per-cell MEDIAN over frames) compared on {len(dims_rows)} cells; "
              f"median delta {deltas[len(deltas) // 2]} mm, "
              f"largest {delta} mm on {slug} oi={oi} "
              f"({'x'.join(map(str, before_dims))} -> {'x'.join(map(str, after_dims))})")
    else:
        print("  dims compared on 0 cells (no perception line on either side)")

    return 1 if (moved or missing) else 0


if __name__ == "__main__":
    raise SystemExit(main())
