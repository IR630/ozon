#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Which census cells did the RULER move, and which did physics move?

Day 11 replaced the episode verdict's measuring point: it used to score the model
ORIGIN (the default pose's bottom, which Gazebo rotates the body about) and now
scores the GOODS. Every cell log therefore carries two verdicts — the body-scored
one that counts, and the legacy origin-scored one printed alongside it for exactly
one census (scripts/zone_verdict.py, in_zone_legacy).

That pair is what lets the report separate two things the team has been conflating.
Cells that flip between the two rulers WITHIN one run moved because of the ruler.
Cells that flip between two RUNS of the same ruler moved because of physics — the
"+-2-3 cells of noise" the milestone metric has always carried. Without this split,
a re-census just produces another number and the jury has to take our word for what
it means.

Usage:
    python3 scripts/census_ruler_diff.py runs/census_body_1 [runs/census_body_2 ...]
"""
import glob
import os
import re
import sys

from triage_matrix import parse_cell

# run_skeleton.sh prints the body-scored verdict on the cell line, and the legacy
# origin-scored one under it (that second line exists only from day 11 on).
LEGACY_RE = re.compile(
    r"^\s+legacy origin-scored verdict: (?P<legacy>PASS|FAIL)", re.MULTILINE
)
TERMINAL_VERDICTS = {"PASS", "FAIL"}


def read_cell(path):
    """(zone, body verdict, legacy verdict) of one cell log; None where absent."""
    cell = parse_cell(path)
    text = open(path, encoding="utf-8", errors="replace").read()
    lg = LEGACY_RE.search(text)
    return (
        cell.expected,
        cell.verdict,
        lg["legacy"] if lg else None,
    )


def read_census(logdir):
    """{cell name: (zone, body, legacy)} for one census directory."""
    cells = {}
    for path in sorted(glob.glob(os.path.join(logdir, "matrix_*.log"))):
        name = os.path.basename(path)[len("matrix_"):-len(".log")]
        cells[name] = read_cell(path)
    return cells


def main():
    dirs = sys.argv[1:]
    if not dirs:
        sys.exit("usage: census_ruler_diff.py <census logdir> [<census logdir> ...]")

    censuses = [(d, read_census(d)) for d in dirs]
    for d, cells in censuses:
        if not cells:
            sys.exit(f"no matrix_*.log cells in {d}")

    print(f"{'cell':30} " + " ".join(f"{os.path.basename(d):>22}" for d, _ in censuses))
    print(f"{'':30} " + " ".join(f"{'body / legacy':>22}" for _ in censuses))
    print("-" * (30 + 23 * len(censuses)))

    ruler_moved, physics_moved = [], []
    for name in sorted(censuses[0][1]):
        row = []
        for _, cells in censuses:
            zone, body, legacy = cells.get(name, (None, "-", None))
            row.append(f"{body} / {legacy or '-'}")
        print(f"{name:30} " + " ".join(f"{c:>22}" for c in row))

        for _, cells in censuses:
            zone, body, legacy = cells.get(name, (None, None, None))
            if (legacy and body in TERMINAL_VERDICTS and body != legacy
                    and name not in ruler_moved):
                ruler_moved.append(name)
        bodies = {
            body
            for _, cells in censuses
            if (body := cells.get(name, (None, None, None))[1]) in TERMINAL_VERDICTS
        }
        if len(bodies) > 1:
            physics_moved.append(name)

    print("-" * (30 + 23 * len(censuses)))
    for d, cells in censuses:
        body_pass = sum(1 for z, b, lg in cells.values() if b == "PASS")
        legacy_pass = sum(1 for z, b, lg in cells.values() if lg == "PASS")
        n = len(cells)
        print(f"{os.path.basename(d):22}  body {body_pass}/{n}   legacy {legacy_pass}/{n}")

    print(f"\nMOVED BY THE RULER (body != legacy in a run): {len(ruler_moved)}")
    for name in ruler_moved:
        print(f"    {name}")
    if len(censuses) > 1:
        print(f"\nMOVED BY PHYSICS (body verdict differs BETWEEN runs): {len(physics_moved)}")
        for name in physics_moved:
            print(f"    {name}")


if __name__ == "__main__":
    main()
