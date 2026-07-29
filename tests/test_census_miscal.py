# -*- coding: utf-8 -*-
"""The drift census must ask BOTH calibration budgets, and keep a control that cannot drift.

Two failures are locked here, and both would produce a census that reads as a
verdict about fusion while measuring something else.

1. ONE BUDGET DECIDES NOTHING. The offline matrix (docs/experiments.md 26.07) has
   the rig order INVERT between calibration columns: 3A leads two heads 92 % against
   82 % at the typical budget and trails 82-83 % against 86 % in the worst one. A
   script that ran only the default budget would answer the easy column and read as
   "the rig is stable across calibration".
2. A CONTROL THAT DRIFTS IS NOT A CONTROL. The decision rule asks whether fusion
   under drift is worse than TOP-ONLY. The drift is baked into the side heads only,
   so the one-head rig is invariant by construction — it has to be in the table, and
   it has to be the clean world, not a miscalibrated one.
"""
import re
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "census_miscal.sh"
TEXT = SCRIPT.read_text(encoding="utf-8")
# Shell line continuations joined, so a generator call split across lines is one
# string to match against. Without this the budget assertions below pass on the
# first line alone and never see the flags they exist to check.
FLAT = re.sub(r"\\\n\s*", " ", TEXT)


def _generator_calls():
    return re.findall(r"python3 scripts/make_miscal_world\.py[^\n]*", FLAT)

# Same harness budgets as the noise census: the three-head bring-up outruns the
# shipped one-head defaults and every cell would abort before measuring anything.
MIN_SOFT_START_TRIES = 120
MIN_CELL_TIMEOUT_S = 300


@pytest.mark.parametrize("name,minimum", [
    ("SOFT_START_TRIES", MIN_SOFT_START_TRIES),
    ("CELL_TIMEOUT", MIN_CELL_TIMEOUT_S),
])
def test_it_raises_the_one_head_harness_budgets(name, minimum):
    match = re.search(rf"^export {name}=\$\{{{name}:-(\d+)\}}", TEXT, re.MULTILINE)
    assert match, f"{SCRIPT.name} must set {name} — the shipped default is one-head sized"
    assert int(match.group(1)) >= minimum


def test_both_calibration_budgets_are_run():
    """The typical column AND the worst one — the matrix disagrees across them."""
    assert re.search(r"TYPICAL_MM=2\.0", TEXT), "typical budget must be 2 mm"
    assert re.search(r"TYPICAL_DEG=0\.2", TEXT), "typical budget must be 0.2 deg"
    assert re.search(r"WORST_MM=3\.0", TEXT), "worst budget must be 3 mm"
    assert re.search(r"WORST_DEG=0\.3", TEXT), "worst budget must be 0.3 deg"


def test_every_generated_world_gets_its_budget_on_the_command_line():
    """A generator call without --shift-mm silently writes the DEFAULT error.

    That is the failure that would make the 'worst' rigs a second copy of the
    typical ones, and the census would then report the rig as budget-insensitive.
    """
    generated = _generator_calls()
    assert len(generated) == 4, f"expected four generated worlds, got {len(generated)}"
    for call in generated:
        assert "--shift-mm" in call and "--tilt-deg" in call, (
            f"generator call without an explicit budget writes the default:\n{call}")


def test_the_worst_rigs_are_generated_from_the_worst_budget():
    for call in _generator_calls():
        if "_worst.sdf" in call:
            assert "$WORST_MM" in call and "$WORST_DEG" in call, call
        else:
            assert "$TYPICAL_MM" in call and "$TYPICAL_DEG" in call, call


def test_all_three_rigs_are_measured():
    for tag in ("1cam", "2cam_typical", "2cam_worst", "3cam_typical", "3cam_worst"):
        assert tag in TEXT, f"rig {tag} missing from the census"


def test_the_one_head_control_runs_the_CLEAN_world():
    """It is the line that does not bend; miscalibrating it would remove the baseline.

    (make_miscal_world.py would in fact ABORT on the one-head world — it carries no
    side head — so this also guards against a config row that silently never ran.)
    """
    control = [line for line in TEXT.splitlines() if "1cam" in line and "sim/worlds" in line]
    assert control, "no config row for the one-head control"
    assert "cell_diverter.sdf" in control[0] and "miscal" not in control[0], control[0]
    assert "bridge.yaml" in control[0], control[0]


def test_generated_worlds_are_validated_before_the_census_starts():
    """33 cells failing one by one on an unparseable world reads as a broken rig."""
    assert "ign sdf -k" in TEXT, "generated worlds must be validated (scripts/check_sdf.sh)"


def test_each_rig_keeps_its_own_node_log():
    """heads=N must be verifiable PER RIG.

    A shared NODE_LOG leaves only the last rig's, and a rig that quietly lost a side
    head is indistinguishable from one that fused correctly.
    """
    assert re.search(r'NODE_LOG="\$OUT/\$tag\.node\.log"', TEXT), (
        "each rig needs its own NODE_LOG or heads=N cannot be attributed")
