# -*- coding: utf-8 -*-
"""Cell wall-clock parsing (scripts/census_timing_diff.py).

This tool answered the merge gate's open question — whether the branch made a
cell slower — so its failure mode matters as much as its result. It had one, and
it was silent: requiring the `stages:` line dropped every cell of a census taken
on main (which does not print that line) and reported "no cells", which reads
exactly like a control run that failed to execute. Two runs were diagnosed as
"main's code does not work" before that was spotted. The first test below is that
bug.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from census_timing_diff import timings  # noqa: E402

VERDICT = ("bottle -> D: PASS (pose x=3.87 y=-1.31 z=0.07, "
           "cycle {cycle}s from launch)")
STAGES = "  stages: bringup {bringup}s, feed 2.7s, transit+verdict 9.2s"


def write_cell(logdir, slug, oi, lines):
    logdir.mkdir(exist_ok=True)
    (logdir / f"matrix_{slug}_{oi}.log").write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_a_census_without_the_stages_line_still_reports_its_cycle(tmp_path):
    """main does not print `stages:`; dropping those cells faked a failed control."""
    write_cell(tmp_path, "bottle", 0, [VERDICT.format(cycle="34.6")])

    rows = timings(str(tmp_path))

    assert list(rows) == [("bottle", 0)]
    assert rows[("bottle", 0)]["cycle"] == 34.6
    assert "bringup" not in rows[("bottle", 0)]


def test_stage_times_are_read_when_the_branch_did_print_them(tmp_path):
    write_cell(tmp_path, "bottle", 1, [
        VERDICT.format(cycle="30.0"),
        STAGES.format(bringup="18.1"),
    ])

    row = timings(str(tmp_path))[("bottle", 1)]

    assert row == {"bringup": 18.1, "feed": 2.7, "transit": 9.2, "cycle": 30.0}


def test_a_cell_that_never_reached_its_verdict_is_not_counted_as_fast(tmp_path):
    """A capped cell leaves an EMPTY log — it must not enter the distribution."""
    write_cell(tmp_path, "box_400x400x300", 0, [])

    assert timings(str(tmp_path)) == {}


def test_a_file_that_is_not_a_cell_log_is_ignored(tmp_path):
    (tmp_path / "summary.log").write_text("=== matrix ... 33/33 ===\n", encoding="utf-8")
    write_cell(tmp_path, "bottle", 2, [VERDICT.format(cycle="29.9")])

    assert list(timings(str(tmp_path))) == [("bottle", 2)]
