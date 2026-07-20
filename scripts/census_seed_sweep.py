#!/usr/bin/env python3
"""Aggregate several single-seed censuses into one honest routing distribution.

`run_matrix.sh <seed>` scores ONE seed; the stability-repeat lesson
(docs/decisions.md 2026-07-16) is that a headline "33/33" on seed 0 can hide a
seed-sensitive cell (Pouf oi0/oi1 diverged: 31/33 on a seed-0 replay vs 11/11 on
seed 1). The jury changes input parameters and re-runs, so the defensible claim
is a distribution across seeds, not one seed's pass.

This reads the per-seed `runs/matrix_*` dirs a sweep produced, reuses
triage_matrix's cell parser + root-cause diagnosis (no second parser), and
reports: overall routing N/(11 x N x seeds), which cells are NON-DETERMINISTIC
(not unanimously PASS across the seeds that ran them), and the classification-vs-
execution split of every failure.

Produce the input dirs with, from repo root in WSL/Docker:
    for s in 0 1 2 3 4; do bash scripts/run_matrix.sh $s 3; done
    python3 scripts/census_seed_sweep.py            # globs runs/matrix_*_seed*
    python3 scripts/census_seed_sweep.py runs/matrix_A runs/matrix_B  # explicit
"""

from __future__ import annotations

import glob
import os
import re
import sys
from collections import defaultdict
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from triage_matrix import diagnose, parse_cell  # noqa: E402

# Root causes that are OUR rule/perception rather than the mechanism/physics —
# the same split triage_matrix prints, kept identical on purpose.
CLASSIFICATION_CAUSES = {"misroute", "no_detect", "no_classification"}


@dataclass(frozen=True)
class Record:
    """One diagnosed cell from one census run (trial).

    `trial` is the run identity (the run_matrix.sh dir), NOT the seed: two runs
    of the SAME seed are distinct trials, so a cell that passes in one and fails
    in the other is correctly flagged — that Gazebo non-determinism at a fixed
    seed is exactly the Pouf signal this tool exists to surface. Keying by seed
    instead would let a later same-seed run silently overwrite an earlier
    failure.
    """

    trial: str
    slug: str
    orient: int
    cause: str  # 'ok' or a triage root cause
    expected: str | None


@dataclass(frozen=True)
class SweepSummary:
    routed: int
    total: int
    nondeterministic: list  # (slug, orient, {seed: cause})
    class_failures: int
    exec_failures: int


def _seed_of(logdir: str) -> str:
    """The seed label of a run_matrix.sh dir (…_seed<N>), or the dir name."""
    m = re.search(r"seed(\w+)$", os.path.basename(logdir.rstrip("/\\")))
    return m[1] if m else os.path.basename(logdir.rstrip("/\\"))


def summarize(records) -> SweepSummary:
    """Cross-seed routing distribution from diagnosed Records.

    A cell is (slug, orient). It is NON-DETERMINISTIC when the trials that ran it
    did not all pass — that is the honest signal the single-seed census cannot
    give. Failures are counted once per (trial, cell), split classification vs
    execution exactly as triage_matrix does.
    """
    by_cell = defaultdict(dict)  # (slug, orient) -> {trial: cause}
    routed = total = 0
    class_failures = exec_failures = 0
    for r in records:
        by_cell[(r.slug, r.orient)][r.trial] = r.cause
        total += 1
        if r.cause == "ok":
            routed += 1
        elif r.cause in CLASSIFICATION_CAUSES:
            class_failures += 1
        else:
            exec_failures += 1

    nondeterministic = []
    for (slug, orient), trial_cause in sorted(by_cell.items()):
        if any(cause != "ok" for cause in trial_cause.values()):
            nondeterministic.append((slug, orient, trial_cause))

    return SweepSummary(routed, total, nondeterministic, class_failures, exec_failures)


def _records_from_dirs(logdirs) -> list[Record]:
    records = []
    for logdir in logdirs:
        trial = os.path.basename(logdir.rstrip("/\\"))
        for path in sorted(glob.glob(os.path.join(logdir, "matrix_*.log"))):
            cell = diagnose(parse_cell(path))
            records.append(Record(trial, cell.slug, cell.orient, cell.cause, cell.expected))
    return records


def main(argv) -> int:
    logdirs = argv or sorted(
        d for d in glob.glob("runs/matrix_*seed*") if os.path.isdir(d)
    )
    if not logdirs:
        print("no census dirs — run scripts/run_matrix.sh for a few seeds first")
        return 1

    seeds = sorted({_seed_of(d) for d in logdirs})
    print(f"seeds: {', '.join(seeds)}  ({len(logdirs)} census dirs)")
    records = _records_from_dirs(logdirs)
    if not records:
        print("no matrix_*.log cells found in the given dirs")
        return 1

    s = summarize(records)
    print(f"=== routing across seeds: {s.routed}/{s.total} ===")
    print(f"classification failures: {s.class_failures}   execution failures: {s.exec_failures}\n")

    if not s.nondeterministic:
        print("every cell passed on every trial that ran it (no seed sensitivity found)")
    else:
        print(f"NON-DETERMINISTIC / failing cells ({len(s.nondeterministic)}):")
        for slug, orient, trial_cause in s.nondeterministic:
            detail = " ".join(f"{trial}:{cause}" for trial, cause in sorted(trial_cause.items()))
            print(f"  {slug} oi={orient}: {detail}")
    # Non-zero exit iff not every cell passed on every trial — a clean gate signal.
    return 0 if s.routed == s.total else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
