#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Does a cell take LONGER on this branch than on the baseline?

The merge gate hit five cells that died on run_matrix.sh's 180 s wall-clock cap
where the 22.07 baseline had none. Two of them coincide with load this session
created, but "coincides" is not a cause, and a branch that boots slower would
produce exactly the same symptom for a real reason.

So measure the thing directly: `run_skeleton.sh` prints, per cell,
    stages: bringup 18.1s, feed 2.7s, transit+verdict 9.2s
    ... cycle 30.0s from launch
Compare the distributions between two censuses. A branch that costs seconds of
bringup shows up here; contention shows up as a heavy tail with an unchanged
median.

Usage:
    python3 scripts/census_timing_diff.py runs/a2sweep_..._seed0 runs/lrcensus_seed0
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import statistics

STAGES_RE = re.compile(
    r"^\s+stages: bringup (?P<bringup>[\d.]+)s, feed (?P<feed>[\d.]+)s, "
    r"transit\+verdict (?P<transit>[\d.]+)s", re.MULTILINE)
CYCLE_RE = re.compile(r"cycle (?P<cycle>[\d.]+)s from launch")


def timings(logdir):
    """{(slug, orient): {stage: seconds}} for every cell that reported stages."""
    out = {}
    for path in sorted(glob.glob(os.path.join(logdir, "matrix_*.log"))):
        name = os.path.basename(path)
        m = re.match(r"matrix_(?P<slug>.+)_(?P<oi>\d+)\.log$", name)
        if not m:
            continue
        with open(path, encoding="utf-8", errors="replace") as handle:
            text = handle.read()
        stages = STAGES_RE.search(text)
        cycle = CYCLE_RE.search(text)
        # `stages:` is instrumentation this branch added; main does not print it.
        # Requiring it silently dropped EVERY main cell and reported "no cells",
        # which reads exactly like a failed control. The cycle line is on both.
        if not stages and not cycle:
            continue
        row = {k: float(v) for k, v in stages.groupdict().items()} if stages else {}
        if cycle:
            row["cycle"] = float(cycle["cycle"])
        out[(m["slug"], int(m["oi"]))] = row
    return out


def _summarise(label, values):
    if not values:
        return f"  {label:<10} (no cells reported)"
    values = sorted(values)
    return (f"  {label:<10} n={len(values):<4} median {statistics.median(values):6.1f}s"
            f"  p90 {values[int(0.9 * (len(values) - 1))]:6.1f}s"
            f"  max {values[-1]:6.1f}s")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("baseline")
    ap.add_argument("candidate")
    args = ap.parse_args(argv)

    before, after = timings(args.baseline), timings(args.candidate)
    print(f"=== timing: {args.baseline} -> {args.candidate} ===")
    for stage in ("bringup", "feed", "transit", "cycle"):
        print(f" {stage}:")
        print(_summarise("baseline", [row[stage] for row in before.values() if stage in row]))
        print(_summarise("branch", [row[stage] for row in after.values() if stage in row]))

    shared = sorted(before.keys() & after.keys())
    deltas = [(after[k]["cycle"] - before[k]["cycle"], k)
              for k in shared if "cycle" in before[k] and "cycle" in after[k]]
    if deltas:
        deltas.sort()
        median_delta = statistics.median(d for d, _ in deltas)
        print(f" per-cell cycle delta on {len(deltas)} shared cells: "
              f"median {median_delta:+.1f}s, "
              f"worst {deltas[-1][0]:+.1f}s on {deltas[-1][1][0]} oi={deltas[-1][1][1]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
