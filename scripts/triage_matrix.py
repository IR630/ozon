#!/usr/bin/env python3
"""Triage a routing-matrix census: each cell -> root cause, not just PASS/FAIL.

Day 6 (P3+P4) asks "where a route is wrong: perception or rule?" — and the
three censuses so far were each triaged by hand, re-reading 33 episode logs.
This does it from the logs alone.

The expected zone is NOT duplicated here: run_skeleton.sh already prints it in
the verdict line ("bottle -> D: PASS (pose ...)"), so the run_matrix.sh
SLUGS/ZONES table stays the single source of truth (CLAUDE.md).

Usage (from repo root, after scripts/run_matrix.sh):
    python3 scripts/triage_matrix.py                 # reads /tmp/matrix_*.log
    python3 scripts/triage_matrix.py --logdir /tmp
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import subprocess
from collections import Counter
from dataclasses import dataclass

# The item is spawned at x=-1.5 and the camera window opens near x=1.5. An item
# whose final pose never crossed this line never reached the camera at all — it
# jammed on the infeed, which is an EXECUTION failure, not a perception one.
CAMERA_REACH_X = 1.0
# Belt surface sits at z~0.45; the floor patches are at z<0.25 (run_skeleton.sh).
BELT_Z_MIN = 0.35

VERDICT_RE = re.compile(
    r"^(?P<slug>\S+) -> (?P<zone>[BCD]): (?P<verdict>PASS|FAIL) "
    r"\(pose x=(?P<x>\S+) y=(?P<y>\S+) z=(?P<z>\S+), cycle (?P<cycle>[\d.]+)s"
)
PERCEPTION_RE = re.compile(
    r"\[perception\]: item \d+: (?P<w>\d+)x(?P<h>\d+)x(?P<d>\d+) mm K=(?P<k>[\d.]+)"
)
CLASSIFIER_RE = re.compile(
    r"\[classifier\]: item \d+: (?P<cat>[BCD]) \(k=(?P<k>[\d.]+), conf=(?P<conf>[\d.]+)"
)
# The controller's own decision line ("item 1: D — firing pusher_d in 0.46s",
# "item 1: B — rides to belt end"). It is the route the item ACTUALLY took, and
# it survives when the classifier's line is lost: killing the launch on verdict
# truncates node stdout, and cells have been seen with the controller line but
# no classifier line (bottle oi=1, seed-0 diverter census) — triaging those as
# "never classified" would blame perception for a mechanism failure.
CONTROLLER_RE = re.compile(r"\[controller\]: item \d+: (?P<cat>[BCD])\b")
FIRED_RE = re.compile(r"pusher_(?P<side>[cd]) FIRED")


@dataclass
class Cell:
    """One census episode, reduced to what decides its root cause."""

    slug: str
    orient: int
    mtime: float  # last write to the log — identifies the cell still in flight
    expected: str | None  # None when the episode was killed before its verdict
    verdict: str  # PASS | FAIL | TIMEOUT
    category: str | None  # what the classifier decided
    k: float | None
    dims_mm: tuple[int, int, int] | None
    fired: str | None  # 'c' | 'd' | None
    pose: tuple[float, float, float] | None
    cycle_s: float | None
    cause: str = ""
    detail: str = ""


def parse_cell(path: str) -> Cell:
    """Reduce one episode log to a Cell (no root cause yet)."""
    name = os.path.basename(path)
    m = re.match(r"matrix_(?P<slug>.+)_(?P<oi>\d+)\.log$", name)
    if not m:
        raise ValueError(f"not a matrix cell log: {name}")

    text = open(path, encoding="utf-8", errors="replace").read()

    def last(pattern):
        found = None
        for found in pattern.finditer(text):  # the last match wins
            pass
        return found

    verdict_m = last(VERDICT_RE)
    perception_m = last(PERCEPTION_RE)
    classifier_m = last(CLASSIFIER_RE)
    controller_m = last(CONTROLLER_RE)
    fired_m = last(FIRED_RE)

    return Cell(
        slug=m["slug"],
        orient=int(m["oi"]),
        mtime=os.path.getmtime(path),
        expected=verdict_m["zone"] if verdict_m else None,
        # No verdict line = the cell was killed by the per-cell timeout (physics
        # wedged): run_skeleton.sh always prints one otherwise. The single
        # exception — the cell still in flight — is re-labelled in main().
        verdict=verdict_m["verdict"] if verdict_m else "TIMEOUT",
        # The controller's route beats the classifier's line (see CONTROLLER_RE).
        category=(controller_m or classifier_m)["cat"] if (controller_m or classifier_m) else None,
        k=float(classifier_m["k"]) if classifier_m else None,
        dims_mm=(
            (int(perception_m["w"]), int(perception_m["h"]), int(perception_m["d"]))
            if perception_m
            else None
        ),
        fired=fired_m["side"] if fired_m else None,
        pose=(
            (float(verdict_m["x"]), float(verdict_m["y"]), float(verdict_m["z"]))
            if verdict_m
            else None
        ),
        cycle_s=float(verdict_m["cycle"]) if verdict_m else None,
    )


def diagnose(cell: Cell) -> Cell:
    """Assign the root cause. Order matters: earlier stages mask later ones."""
    if cell.verdict == "PASS":
        cell.cause = "ok"
        return cell

    if cell.verdict == "RUNNING":
        cell.cause = "in_progress"
        cell.detail = "episode still running (log written just now)"
        return cell

    if cell.verdict == "TIMEOUT":
        cell.cause = "physics_wedge"
        cell.detail = "episode killed by per-cell timeout (no verdict line)"
        return cell

    x, y, z = cell.pose if cell.pose else (float("nan"),) * 3

    # Stage 1 — did the item even reach the camera?
    if cell.dims_mm is None:
        if x < CAMERA_REACH_X:
            cell.cause = "feed_jam"
            cell.detail = f"never reached the camera (final x={x:.2f})"
        else:
            cell.cause = "no_detect"
            cell.detail = f"passed the camera (x={x:.2f}) but perception saw nothing"
        return cell

    # Stage 2 — was the category right? (measurement exists, so the rule ran)
    if cell.category is None:
        cell.cause = "no_classification"
        cell.detail = "measured but never classified"
        return cell
    if cell.category != cell.expected:
        cell.cause = "misroute"
        cell.detail = (
            f"classified {cell.category}, expected {cell.expected} "
            f"(k={cell.k}, dims={cell.dims_mm})"
        )
        return cell

    # Stage 3 — category right, item in the wrong place: the mechanism failed.
    if cell.expected == "B":
        cell.cause = "false_divert"
        cell.detail = f"B item left the belt (pose x={x:.2f} y={y:.2f} z={z:.2f})"
    elif cell.fired is None:
        cell.cause = "no_fire"
        cell.detail = f"classified {cell.category} but the mechanism never fired"
    elif z >= BELT_Z_MIN:
        cell.cause = "mech_miss"
        cell.detail = f"fired, but the item stayed at belt height (z={z:.2f}, x={x:.2f})"
    else:
        cell.cause = "mech_overshoot"
        cell.detail = f"fired, item off the belt but outside the zone window (x={x:.2f} y={y:.2f})"
    return cell


def census_in_flight() -> bool:
    """Is a matrix episode running right now? (Gazebo alive = a cell in flight.)"""
    try:
        return subprocess.run(["pgrep", "-f", "ign gazebo"], capture_output=True).returncode == 0
    except OSError:  # no pgrep (e.g. triaging a copied logdir on Windows)
        return False


def latest_run() -> str | None:
    """The newest runs/matrix_* directory — what run_matrix.sh just wrote."""
    runs = glob.glob("runs/matrix_*")
    return max(runs, key=os.path.getmtime) if runs else None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--logdir",
        default=None,
        help="census log dir (default: the newest runs/matrix_*)",
    )
    args = ap.parse_args()

    logdir = args.logdir or latest_run()
    if not logdir:
        print("no runs/matrix_* found — run scripts/run_matrix.sh first")
        return 1

    paths = sorted(glob.glob(os.path.join(logdir, "matrix_*.log")))
    if not paths:
        print(f"no matrix_*.log in {logdir}")
        return 1
    print(f"census: {logdir}")

    parsed = [parse_cell(p) for p in paths]
    # Triage is useful mid-census. run_matrix.sh runs cells strictly one after
    # another, so while Gazebo is alive the newest log is the episode still in
    # flight — it has no verdict line YET and must not be scored as a wedge.
    if census_in_flight():
        max(parsed, key=lambda c: c.mtime).verdict = "RUNNING"

    everything = [diagnose(c) for c in parsed]
    running = [c for c in everything if c.cause == "in_progress"]
    cells = [c for c in everything if c.cause != "in_progress"]
    passed = sum(c.cause == "ok" for c in cells)

    print(f"=== triage: routing {passed}/{len(cells)} ===\n")
    for c in sorted(cells, key=lambda c: (c.cause != "ok", c.slug, c.orient)):
        if c.cause == "ok":
            continue
        print(f"  {c.slug} oi={c.orient} -> {c.expected}: {c.cause}")
        print(f"      {c.detail}")
    for c in running:
        print(f"  {c.slug} oi={c.orient}: still running — not scored")

    print("\n=== failures by root cause ===")
    causes = Counter(c.cause for c in cells if c.cause != "ok")
    # Classification failures are OUR rule/perception; the rest is the mechanism
    # and the physics — the split the day-6 decision is read against.
    perception_cls = {"misroute", "no_detect", "no_classification"}
    for cause, n in causes.most_common():
        blame = "classification" if cause in perception_cls else "execution"
        print(f"  {cause:16s} {n:2d}  [{blame}]")
    cls_n = sum(n for c, n in causes.items() if c in perception_cls)
    exe_n = sum(causes.values()) - cls_n
    print(f"\n  classification: {cls_n}   execution: {exe_n}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
