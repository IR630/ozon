# -*- coding: utf-8 -*-
"""Cell-by-cell census comparison (scripts/census_cell_diff.py).

The merge gate of feat/line-readiness hangs on this tool: the branch changed the
classifier log format (`k=%.6f` against the baseline's `%.3f`), so a text diff of
two censuses reports every cell as changed and answers nothing. These lock the
property that makes the gate meaningful — a re-formatted but identically routed
cell must read as UNCHANGED, and a genuinely re-routed one must read as moved.
"""
from census_cell_diff import compare

PERCEPTION = "[python3-2] [INFO] [178.0] [perception]: item 1: {dims} mm K={k} at (1.76, 0.00){tail}"
CLASSIFIER = "[python3-3] [INFO] [178.0] [classifier]: item 1: {cat} (k={k}, conf=0.99, n=9)"


def verdict(slug, zone, v):
    return f"{slug} -> {zone}: {v} (pose x=3.74 y=-0.58 z=0.058, cycle 25.8s from launch)"


def write_cell(logdir, slug, oi, lines, rc=0):
    logdir.mkdir(exist_ok=True)
    (logdir / f"matrix_{slug}_{oi}.log").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (logdir / f"matrix_{slug}_{oi}.status").write_text(f"rc={rc}\n", encoding="utf-8")


def baseline_cell(logdir, slug="bottle", oi=0, cat="D", dims="298x102x92"):
    write_cell(logdir, slug, oi, [
        verdict(slug, "D", "PASS"),
        PERCEPTION.format(dims=dims, k="1.00", tail=""),
        CLASSIFIER.format(cat=cat, k="1.000"),
    ])


def test_a_reformatted_but_identically_routed_cell_does_not_read_as_moved(tmp_path):
    """The exact trap the gate would fall into: k=%.3f -> %.6f, heads=N added."""
    before, after = tmp_path / "before", tmp_path / "after"
    baseline_cell(before)
    write_cell(after, "bottle", 0, [
        verdict("bottle", "D", "PASS"),
        # Same cell, branch formatting: six decimals on k and the heads= suffix.
        PERCEPTION.format(dims="298x102x92", k="1.00", tail=" heads=1"),
        CLASSIFIER.format(cat="D", k="1.000000"),
    ])

    moved, dims_rows, missing = compare(str(before), str(after))

    assert moved == []
    assert missing == []
    assert dims_rows[0][3] == 0  # per-axis delta, mm


def test_a_re_routed_cell_is_reported_as_moved(tmp_path):
    before, after = tmp_path / "before", tmp_path / "after"
    baseline_cell(before)
    write_cell(after, "bottle", 0, [
        verdict("bottle", "D", "FAIL"),
        PERCEPTION.format(dims="298x102x92", k="1.00", tail=" heads=1"),
        CLASSIFIER.format(cat="B", k="0.700000"),
    ], rc=1)

    moved, _, _ = compare(str(before), str(after))

    assert [key for key, _, _ in moved] == [("bottle", 0)]
    _, was, now = moved[0]
    assert (was.verdict, was.category) == ("PASS", "D")
    assert (now.verdict, now.category) == ("FAIL", "B")


def test_millimetre_noise_is_reported_but_not_counted_as_a_moved_cell(tmp_path):
    """Gazebo is non-deterministic at a fixed seed — dims drift is not a regression."""
    before, after = tmp_path / "before", tmp_path / "after"
    baseline_cell(before, slug="helmet", cat="B", dims="357x303x280")
    write_cell(after, "helmet", 0, [
        verdict("helmet", "D", "PASS"),
        PERCEPTION.format(dims="354x305x282", k="0.78", tail=" heads=1"),
        CLASSIFIER.format(cat="B", k="0.733000"),
    ])

    moved, dims_rows, _ = compare(str(before), str(after))

    assert moved == []
    assert dims_rows[0][3] == 3


def test_dims_are_compared_on_the_median_frame_not_the_last_one(tmp_path):
    """Routing uses the median over frames, so the comparison must too.

    `ItemAggregator` routes on the median of the frames it saw; the last line in
    a log is just the last one that happened to flush. Where several lines
    survived the truncation, comparing medians removes a tail that comparing last
    lines would report as a regression.
    """
    before, after = tmp_path / "before", tmp_path / "after"
    write_cell(before, "bottle", 2, [
        verdict("bottle", "D", "PASS"),
        PERCEPTION.format(dims="303x90x88", k="0.25", tail=""),
        PERCEPTION.format(dims="302x90x87", k="0.25", tail=""),
        PERCEPTION.format(dims="200x90x87", k="0.44", tail=""),  # truncated, last
        CLASSIFIER.format(cat="D", k="0.250"),
    ])
    write_cell(after, "bottle", 2, [
        verdict("bottle", "D", "PASS"),
        PERCEPTION.format(dims="303x89x88", k="0.25", tail=" heads=1"),
        PERCEPTION.format(dims="302x90x87", k="0.25", tail=" heads=1"),
        PERCEPTION.format(dims="303x89x88", k="0.25", tail=" heads=1"),  # whole, last
        CLASSIFIER.format(cat="D", k="0.248500"),
    ])

    moved, dims_rows, _ = compare(str(before), str(after))

    assert moved == []
    # Last-frame comparison would have said 103 mm; the medians differ by 1.
    assert dims_rows[0][1] == (302, 90, 87)
    assert dims_rows[0][2] == (303, 89, 88)
    assert dims_rows[0][3] == 1


def test_a_cell_missing_from_one_side_is_named_instead_of_silently_dropped(tmp_path):
    """An interrupted sweep must not pass the gate by comparing fewer cells."""
    before, after = tmp_path / "before", tmp_path / "after"
    baseline_cell(before)
    baseline_cell(before, slug="pen", oi=2, cat="C")
    baseline_cell(after)

    moved, _, missing = compare(str(before), str(after))

    assert moved == []
    assert missing == [(("pen", 2), "candidate")]
