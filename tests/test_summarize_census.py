# -*- coding: utf-8 -*-
"""Routing misses and rc=124 must never be summed, and a running cell is not a failure.

The sum is the specific error this project has already made: 15 cells that never
produced a category were read as "the extra head harms" (docs/decisions.md 28.07).
`rc=124` measures how busy the machine was; a routing miss measures the contour.
Their sum means nothing, so the summariser must keep them in separate columns.
"""
from scripts.summarize_census import scan


def _cell(logdir, name, log_text, rc=None):
    (logdir / f"matrix_{name}.log").write_text(log_text, encoding="utf-8")
    if rc is not None:
        (logdir / f"matrix_{name}.status").write_text(f"rc={rc}\n", encoding="utf-8")


PASS_LOG = "plate -> D: PASS (pose x=4.1 y=-1.2 z=0.02, cycle 14.1s from launch)\n"
FAIL_LOG = "pen -> C: FAIL (pose x=3.0 y=0.1 z=0.02, cycle 20.0s from launch)\n"
# What a capped cell actually leaves behind: one word, no verdict line at all.
TIMEOUT_LOG = "Terminated\n"


def test_the_three_outcomes_land_in_three_columns(tmp_path):
    _cell(tmp_path, "plate_0", PASS_LOG, rc=0)
    _cell(tmp_path, "pen_0", FAIL_LOG, rc=1)
    _cell(tmp_path, "bottle_1", TIMEOUT_LOG, rc=124)
    passes, misses, timeouts, other, total, _detail = scan(tmp_path)
    assert (passes, misses, timeouts, other, total) == (1, 1, 1, 0, 3)


def test_a_timeout_is_not_counted_as_a_routing_miss(tmp_path):
    """The whole point. A cell that produced no category cannot have mis-routed."""
    for i in range(5):
        _cell(tmp_path, f"bottle_{i}", TIMEOUT_LOG, rc=124)
    passes, misses, timeouts, _other, total, _detail = scan(tmp_path)
    assert misses == 0, "timeouts must not be scored as routing misses"
    assert timeouts == 5
    assert passes == 0 and total == 5


def test_a_cell_without_a_status_is_still_running_not_broken(tmp_path):
    """run_matrix.sh writes the log first; reading mid-census must not invent failures."""
    _cell(tmp_path, "plate_0", PASS_LOG, rc=0)
    _cell(tmp_path, "pen_0", "")           # no .status yet
    passes, misses, timeouts, other, total, _detail = scan(tmp_path)
    assert (passes, misses, timeouts, other) == (1, 0, 0, 0)
    assert total == 1, "an unfinished cell must not be scored at all"


def test_a_runner_error_is_its_own_column(tmp_path):
    """rc=1 with no verdict is a harness failure, not a wrong route.

    `lunchbox oi=2` failed exactly this way ("belt never reached full speed") and
    was correctly called infrastructure rather than a rig defect.
    """
    _cell(tmp_path, "lunchbox_2", "ABORT: belt never reached full speed\n", rc=1)
    passes, misses, timeouts, other, total, _detail = scan(tmp_path)
    assert (passes, misses, timeouts, other, total) == (0, 0, 0, 1, 1)
