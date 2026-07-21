# -*- coding: utf-8 -*-
"""The diagnosis must compare the two rigs, not measure one of them twice.

Its entire value is the DIFFERENCE between a perfectly registered rig and one
off by the calibration budget, so the two worlds, the three-head bridge and the
side-head flag are pinned here. A run that quietly dumped the same world twice —
or the one-camera world, which is what a dropped WORLD seam falls back to —
would print two identical clouds and read as "calibration changes nothing".
"""
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "diagnose_side_clouds.sh"


def test_both_rigs_are_dumped():
    text = SCRIPT.read_text(encoding="utf-8")

    assert "cell_diverter_3cam.sdf" in text, "the clean rig is the control"
    assert "cell_diverter_3cam_miscal.sdf" in text, "the miscalibrated rig is the case"
    assert "for TAG in clean miscal" in text


def test_the_miscalibrated_world_is_generated_when_missing():
    """It is gitignored on purpose (a derived world would drift), so a fresh
    checkout must build it rather than silently dump the clean rig twice."""
    text = SCRIPT.read_text(encoding="utf-8")

    assert "make_miscal_world.py" in text


def test_the_three_head_bridge_and_the_side_heads_are_both_switched_on():
    """Either one missing gives empty side clouds — the same output an
    everything-is-fine rig would give."""
    text = SCRIPT.read_text(encoding="utf-8")

    assert "BRIDGE_CONFIG=sim/bridge_3cam.yaml" in text
    assert "SIDE=1" in text
