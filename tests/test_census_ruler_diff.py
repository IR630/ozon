# -*- coding: utf-8 -*-
"""Separating "the ruler moved this cell" from "physics moved this cell".

The day-11 re-census is only worth running if it can attribute each moved cell. A cell
that flips between the two rulers inside ONE run moved because of the measuring point;
a cell that flips between two RUNS of the same ruler moved because of contact physics —
the +-2-3 cells of noise the metric has always carried.
"""
import sys

import pytest

from census_ruler_diff import main, read_cell, read_census


def write_cell(d, slug, oi, verdict, legacy=None, zone="C"):
    lines = [f"{slug} -> {zone}: {verdict} (pose x=3.6 y=0.5 z=0.28, cycle 30.1s from launch)",
             "  resting rpy: r=1.8 p=-0.4 y=-2.1",
             "  body: centre x=3.4 y=0.6 z=0.23 | lowest z=+0.011 | origin-to-body dz=-0.05"]
    if legacy:
        lines.append(f"  legacy origin-scored verdict: {legacy} (body-scored: {verdict})")
    (d / f"matrix_{slug}_{oi}.log").write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_status(d, slug, oi, rc):
    (d / f"matrix_{slug}_{oi}.status").write_text(f"rc={rc}\n", encoding="utf-8")


def test_a_cell_the_ruler_moved_is_read_as_both_verdicts(tmp_path):
    # pouf oi=1: delivered to the cage, but the origin-scored gate called it a failure.
    write_cell(tmp_path, "pouf", 1, "PASS", legacy="FAIL")
    assert read_cell(str(tmp_path / "matrix_pouf_1.log")) == ("C", "PASS", "FAIL")


def test_a_pre_day_11_log_has_no_legacy_line_and_says_so(tmp_path):
    write_cell(tmp_path, "bottle", 0, "PASS", legacy=None, zone="D")
    assert read_cell(str(tmp_path / "matrix_bottle_0.log")) == ("D", "PASS", None)


def test_a_legacy_unfinished_cell_has_unknown_termination(tmp_path):
    (tmp_path / "matrix_pouf_2.log").write_text("boot spam only\n", encoding="utf-8")
    assert read_cell(str(tmp_path / "matrix_pouf_2.log")) == (
        None,
        "UNKNOWN_TERMINATION",
        None,
    )


@pytest.mark.parametrize(
    ("rc", "expected"),
    [(124, "TIMEOUT"), (7, "RUNNER_ERROR"), (137, "RUNNER_ERROR")],
)
def test_status_distinguishes_timeout_from_runner_failure(tmp_path, rc, expected):
    path = tmp_path / "matrix_pouf_2.log"
    path.write_text("boot spam only\n", encoding="utf-8")
    write_status(tmp_path, "pouf", 2, rc)

    assert read_cell(str(path)) == (None, expected, None)


def test_terminal_verdict_must_agree_with_runner_status(tmp_path):
    write_cell(tmp_path, "pouf", 1, "PASS", legacy="FAIL")
    write_status(tmp_path, "pouf", 1, 1)

    assert read_cell(str(tmp_path / "matrix_pouf_1.log")) == (
        "C",
        "HARNESS_ERROR",
        "FAIL",
    )


def test_a_census_directory_is_keyed_by_cell(tmp_path):
    write_cell(tmp_path, "pouf", 1, "PASS", legacy="FAIL")
    write_cell(tmp_path, "bag", 1, "PASS", legacy="PASS", zone="B")
    assert read_census(str(tmp_path)) == {
        "pouf_1": ("C", "PASS", "FAIL"),
        "bag_1": ("B", "PASS", "PASS"),
    }


def test_runner_failure_is_not_reported_as_physics_movement(
    tmp_path, monkeypatch, capsys
):
    first, second = tmp_path / "first", tmp_path / "second"
    first.mkdir()
    second.mkdir()
    write_cell(first, "pouf", 1, "PASS", legacy="PASS")
    write_status(first, "pouf", 1, 0)
    (second / "matrix_pouf_1.log").write_text("runner crashed\n", encoding="utf-8")
    write_status(second, "pouf", 1, 7)
    monkeypatch.setattr(sys, "argv", ["census_ruler_diff.py", str(first), str(second)])

    main()

    output = capsys.readouterr().out
    assert "MOVED BY THE RULER (body != legacy in a run): 0" in output
    assert "MOVED BY PHYSICS (body verdict differs BETWEEN runs): 0" in output


def test_harness_error_with_legacy_line_is_not_ruler_movement(
    tmp_path, monkeypatch, capsys
):
    write_cell(tmp_path, "pouf", 1, "PASS", legacy="FAIL")
    write_status(tmp_path, "pouf", 1, 1)
    monkeypatch.setattr(sys, "argv", ["census_ruler_diff.py", str(tmp_path)])

    main()

    assert "MOVED BY THE RULER (body != legacy in a run): 0" in capsys.readouterr().out


def test_pass_to_fail_is_still_reported_as_physics_movement(tmp_path, monkeypatch, capsys):
    first, second = tmp_path / "first", tmp_path / "second"
    first.mkdir()
    second.mkdir()
    write_cell(first, "pouf", 1, "PASS")
    write_status(first, "pouf", 1, 0)
    write_cell(second, "pouf", 1, "FAIL")
    write_status(second, "pouf", 1, 1)
    monkeypatch.setattr(sys, "argv", ["census_ruler_diff.py", str(first), str(second)])

    main()

    assert "MOVED BY PHYSICS (body verdict differs BETWEEN runs): 1" in capsys.readouterr().out
