# -*- coding: utf-8 -*-
"""The live re-ranking must segment the top head the way production does.

This probe exists to stop an offline ranking from reaching the kernel unchecked,
so its own segmentation has to be the shipped one. One specific way it would lie
quietly: borrowing the side heads' 8 mm floor for the top head. That floor is
correct for a grazing view and wrong for a downward one, and it would silently
drop the pen — 9 mm over the belt, and the item the whole C/B border hangs on —
turning "the rule handles the pen" into "the rule never saw the pen".

Skipped when the rig dumps are not in the checkout (`runs/` is gitignored), same
guard as tests/test_probe_depth_dropout.py.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from probe_fusion_live import CONFIGS, top_item_points  # noqa: E402

from src.constants import SIDE_BELT_MARGIN_M  # noqa: E402
from src.perception import BELT_DEPTH_M, BELT_TOP_Z_M, MASK_MARGIN_M  # noqa: E402


def _top_frame(height_mm, h=120, w=160):
    """Synthetic top frame: bare belt with one slab standing `height_mm` over it."""
    depth = np.full((h, w), BELT_DEPTH_M, dtype=float)
    depth[40:80, 50:110] = BELT_DEPTH_M - height_mm / 1000.0
    return depth


def test_a_bare_belt_yields_no_goods():
    """The belt must not become the item, or every rule is ranked on the belt."""
    assert top_item_points(np.full((120, 160), BELT_DEPTH_M)) is None


def test_the_top_head_keeps_its_own_5_mm_margin_and_so_keeps_the_pen():
    """A body between the two margins must survive: it is the pen's whole case.

    The side heads use SIDE_BELT_MARGIN_M = 8 mm because a grazing view lifts the
    belt under a pointing error. Applying that to the top head would discard
    anything under 8 mm — including the 9 mm pen, which sits just above it and
    decides the C/B border.
    """
    # The pen stands 9 mm over the belt: well clear of the top head's 5 mm floor
    # and barely clear of the side heads' 8 mm one. That gap is the whole reason
    # the two margins are different constants.
    assert MASK_MARGIN_M * 1000.0 < 9.0
    assert SIDE_BELT_MARGIN_M * 1000.0 < 9.0
    assert MASK_MARGIN_M < SIDE_BELT_MARGIN_M

    between = top_item_points(_top_frame(7.0))
    assert between is not None, "a body above the 5 mm mask margin was dropped"
    assert len(between) > 100

    below = top_item_points(_top_frame(3.0))
    assert below is None, "a body under the mask margin became goods"


def test_the_points_come_back_in_world_metres_above_the_belt():
    """A backprojection that lands under the belt would poison every rule's height."""
    pts = top_item_points(_top_frame(80.0))
    assert pts.shape[1] == 3
    heights_mm = (pts[:, 2] - BELT_TOP_Z_M) * 1000.0
    assert heights_mm.min() > 0.0
    assert heights_mm.max() == pytest.approx(80.0, abs=2.0)


def test_the_configs_compared_are_two_heads_against_three():
    """The ranking is only meaningful if the rig column means what it says."""
    assert [n for _name, n in CONFIGS] == [1, 2]
    assert "2" in CONFIGS[0][0] and "3A" in CONFIGS[1][0]
