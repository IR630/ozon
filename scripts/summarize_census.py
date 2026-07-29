#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""One census directory -> one row: routing misses and rc=124 timeouts, SEPARATELY.

WHY THE SPLIT IS THE WHOLE POINT. `rc=124` is a wall-clock cap on the cell, not a
verdict: its log is the single word `Terminated` (11 bytes), so no category was ever
produced. Adding it to routing misses produces a number that measures the MACHINE
and reads as a number about the OPTICS — the mistake that cost this project three
false conclusions in one day (docs/decisions.md 28.07, and the 18/33 that turned out
to be segmentation cost). This script refuses to produce that sum.

    python3 scripts/summarize_census.py runs/miscal_20260729_*/
    python3 scripts/summarize_census.py --markdown runs/miscal_*/[123]cam*

Reads the durable `rc=` status files run_matrix.sh writes per cell, so a cell that
never reached a verdict is distinguishable from one that reached a wrong one.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

VERDICT_RE = re.compile(
    r"^(?P<slug>\S+) -> (?P<zone>[BCD]): (?P<verdict>PASS|FAIL) ", re.MULTILINE)


def scan(logdir):
    """(passes, routing_misses, timeouts, other, total) for one census directory.

    A cell whose .status has not been written yet is STILL RUNNING, not broken:
    run_matrix.sh writes the log first and the status when the cell ends. Scoring
    it as an error would make every mid-census read look like a failing rig.
    """
    logdir = Path(logdir)
    passes = misses = timeouts = other = 0
    cells = sorted(logdir.glob("matrix_*.log"))
    detail = []
    running = 0
    for log in cells:
        status = log.with_suffix(".status")
        if not status.is_file():
            running += 1
            continue
        match = re.fullmatch(r"rc=(\d+)", status.read_text(encoding="utf-8").strip())
        rc = int(match.group(1)) if match else None
        text = log.read_text(encoding="utf-8", errors="replace")
        verdict = VERDICT_RE.search(text)
        name = log.stem.replace("matrix_", "")
        if rc == 0 and verdict and verdict.group("verdict") == "PASS":
            passes += 1
        elif rc == 124 and verdict is None:
            timeouts += 1
            detail.append((name, "rc=124 TIMEOUT — no verdict was produced"))
        elif verdict and verdict.group("verdict") == "FAIL":
            misses += 1
            detail.append((name, "routing miss"))
        else:
            other += 1
            detail.append((name, f"rc={rc}, no terminal verdict (harness/runner error)"))
    if running:
        detail.append((f"({running} cells)", "still running — not scored"))
    return passes, misses, timeouts, other, len(cells) - running, detail


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("dirs", nargs="+")
    parser.add_argument("--markdown", action="store_true")
    parser.add_argument("--detail", action="store_true", help="name every non-PASS cell")
    args = parser.parse_args(argv)

    rows = []
    for d in args.dirs:
        path = Path(d)
        if not path.is_dir():
            continue
        passes, misses, timeouts, other, total, detail = scan(path)
        rows.append((path.name, passes, misses, timeouts, other, total, detail))
    if not rows:
        print("no census directories found", file=sys.stderr)
        return 1

    if args.markdown:
        print("| стойка | routing | промахи маршрутизации | rc=124 | прочее |")
        print("|---|---|---|---|---|")
        for name, passes, misses, timeouts, other, total, _ in rows:
            print(f"| {name} | {passes}/{total} | {misses} | {timeouts} | {other} |")
    else:
        print(f"{'census':22}{'routing':>10}{'misses':>9}{'rc=124':>9}{'other':>8}")
        for name, passes, misses, timeouts, other, total, _ in rows:
            print(f"{name:22}{f'{passes}/{total}':>10}{misses:>9}{timeouts:>9}{other:>8}")

    if args.detail:
        for name, _p, _m, _t, _o, _total, detail in rows:
            if detail:
                print(f"\n--- {name} ---")
                for cell, why in detail:
                    print(f"  {cell:32} {why}")

    print("\nrc=124 is a stopwatch on the machine, NOT a routing verdict — the two "
          "columns\nmust not be added. A timed-out cell produced no category at all.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
