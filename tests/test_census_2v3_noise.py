# -*- coding: utf-8 -*-
"""The rig census must budget the harness for the rig it is measuring.

`run_skeleton.sh` waits `SOFT_START_TRIES` x 0.5 s (30 s by default) for the
controller, and `run_matrix.sh` caps a cell at `CELL_TIMEOUT` (180 s). Both defaults
are sized for the ONE-head world. Measured bring-up on one tree: 1 head ~18 s,
2 heads ~29 s, 3 heads ~40 s — so at the defaults every three-head cell aborts with
"belt never reached full speed" before it measures anything, and the census reports
a broken rig when it is really holding a stopwatch.

That failure already happened twice in this project under two different names, so
the budgets are locked here rather than left to whoever runs the script next.
"""
import re
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "census_2v3_noise.sh"
TEXT = SCRIPT.read_text(encoding="utf-8")

# The three-head cycle is 74-94 s and its bring-up ~40 s; anything at or below the
# one-head defaults cannot measure that rig at all.
MIN_SOFT_START_TRIES = 120      # x 0.5 s = 60 s, twice the measured 3-head bring-up
MIN_CELL_TIMEOUT_S = 300        # comfortably past the 74-94 s cycle plus bring-up


@pytest.mark.parametrize("name,minimum", [
    ("SOFT_START_TRIES", MIN_SOFT_START_TRIES),
    ("CELL_TIMEOUT", MIN_CELL_TIMEOUT_S),
])
def test_rig_census_raises_the_one_head_harness_budgets(name, minimum):
    match = re.search(rf"^export {name}=\$\{{{name}:-(\d+)\}}", TEXT, re.MULTILINE)
    assert match, f"{SCRIPT.name} must set {name} — the shipped default is one-head sized"
    assert int(match.group(1)) >= minimum, (
        f"{name}={match.group(1)} is below {minimum}: a three-head cell would be "
        "capped by the harness before it measures anything")


def test_the_budgets_stay_overridable():
    """`${VAR:-default}`, not `VAR=default`: a caller must still be able to pin them."""
    for name in ("SOFT_START_TRIES", "CELL_TIMEOUT"):
        assert f"export {name}=${{{name}:-" in TEXT, (
            f"{name} must keep the :- form so a run can pin its own budget")


def test_the_stream_path_honours_the_same_soft_start_knob():
    """run_stream.sh must not hardcode the one-head wait.

    It did, as a literal `seq 1 60`, and the three-head stream suite therefore
    reported 0/6 episodes with "belt never reached full speed" while the rig was
    fine — the same failure the census hit, on the path that measures THROUGHPUT,
    which is the number the jury actually scores.
    """
    stream = (SCRIPT.parent / "run_stream.sh").read_text(encoding="utf-8")
    assert 'seq 1 "${SOFT_START_TRIES:-' in stream, (
        "run_stream.sh must take SOFT_START_TRIES; a hardcoded wait cannot measure "
        "a rig that boots slower than the shipped one")
