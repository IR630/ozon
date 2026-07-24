# -*- coding: utf-8 -*-
"""A side head's own housing inside the top view is a SILENT, TOTAL failure.

`docs/plan-two-cameras.md` §1.3-бис lists it as the one rig failure with no test:
the housing lands in the top camera's frame, joins the item's mask, the merged
component now touches the frame border, and `_find_item` rejects it — on every
frame the two overlap. Nothing raises, nothing logs a warning; perception simply
reports no item and the cell routes on a default.

The metric clearance of the housing to the cone is already asserted in
tests/test_three_camera_layout.py. What is asserted HERE is the other two links
of the same chain: that the shipped worlds still mount the heads where that
clearance was computed, and that the failure really is silent when they do not.
"""
import re

import numpy as np
import pytest

from scripts.make_miscal_world import heads_present
from scripts.plot_three_camera_layout import BODY_W_M, SIDE_Y_M, SIDE_Z_M
from src.constants import (
    CAMERA_SIDE_NEG_Y_POSE_M,
    CAMERA_SIDE_POS_Y_POSE_M,
    CAMERA_TOP_POSE_M,
)
from src.perception import (
    BELT_DEPTH_M,
    BELT_TOP_Z_M,
    CAMERA_Y_M,
    CAMERA_Z_M,
    FX,
    IMG_W,
    measure_item,
)
from scripts.make_miscal_world import SRC_WORLD

TWO_CAM_WORLD = SRC_WORLD.parent / "cell_diverter_2cam.sdf"
_POSE_BY_NAME = {"camera": CAMERA_TOP_POSE_M,
                 "camera_side_neg_y": CAMERA_SIDE_NEG_Y_POSE_M,
                 "camera_side_pos_y": CAMERA_SIDE_POS_Y_POSE_M}


def _pose(sdf_text, model_name):
    match = re.search(
        r'<model name="%s">\s*\n\s*<static>true</static>\s*\n\s*<pose>([^<]+)</pose>'
        % model_name, sdf_text)
    assert match, "no pose for %s" % model_name
    return [float(v) for v in match.group(1).split()]


# --- link 1: the shipped worlds mount the heads where the clearance was computed


@pytest.mark.parametrize("world", (SRC_WORLD, TWO_CAM_WORLD))
def test_every_head_in_the_shipped_world_sits_where_the_constants_say(world):
    """The SDF comment asks not to edit these poses here alone; this enforces it.

    A head edited in the world and not in `src/constants.py` is not merely a
    duplicated constant: `src/multiview.py` projects the side cloud with the
    CONSTANT, so the run would measure a rig nobody owns — and the clearance
    argument of test_three_camera_layout.py would be computed for the wrong pose.
    """
    text = world.read_text(encoding="utf-8")
    for name in ("camera",) + tuple(n for n, _sign in heads_present(text)):
        x, y, z = _pose(text, name)[:3]
        assert (x, y, z) == pytest.approx(_POSE_BY_NAME[name][0]), name


# --- link 2: at the shipped height the housing is out of frame, at belt level in


def _housing_column_px(z_m):
    """Image column of the -y housing's NEAR edge seen by the top head.

    Pinhole, principal point at the image centre: a point at world y projects to
    IMG_W / 2 + FX * (y - CAMERA_Y_M) / (CAMERA_Z_M - z).
    """
    near_edge_y_m = -(SIDE_Y_M - BODY_W_M / 2.0)
    return IMG_W / 2.0 + FX * (near_edge_y_m - CAMERA_Y_M) / (CAMERA_Z_M - z_m)


def test_the_housing_is_out_of_frame_at_its_mounting_height():
    assert _housing_column_px(SIDE_Z_M) < 0, "side housing projects INTO the top frame"


def test_the_same_housing_lowered_to_the_belt_is_in_frame():
    """Why the mounting HEIGHT is load-bearing and |y| alone is not.

    Same head, same y, dropped to belt level: the cone narrows and the housing
    that cleared it by 400 px now projects inside the image. This is the
    counterfactual the next two tests feed a frame with.
    """
    col = _housing_column_px(BELT_TOP_Z_M)
    assert 0 <= col < IMG_W, f"expected an in-frame column, got {col:.0f}"


# --- link 3: once it IS in frame, the failure is silent and it does not pass


_ITEM_TOP_M = BELT_DEPTH_M - 0.10      # a 100 mm tall item on the belt
_HOUSING_TOP_M = BELT_DEPTH_M - 0.30   # the housing hangs above the belt plane


_HOUSING_COL = int(_housing_column_px(BELT_TOP_Z_M))
# The item rides the belt edge NEAREST that head, one pixel clear of the housing:
# columns are y (across the belt), rows are x (along it), so a travelling item
# keeps this column offset and only its rows change.
_ITEM_COLS = slice(_HOUSING_COL + 1, _HOUSING_COL + 101)


def _frame(item_rows, with_housing):
    """Top-view depth frame: empty belt, one item, optionally the -y housing.

    The housing occupies the columns from the image edge out to its in-frame
    projection — the geometry the previous test computes, not a made-up
    rectangle — and a band of rows, because its x-extent is 90 mm too.
    """
    depth = np.full((480, IMG_W), BELT_DEPTH_M)
    depth[item_rows, _ITEM_COLS] = _ITEM_TOP_M
    if with_housing:
        depth[230:270, :_HOUSING_COL + 1] = _HOUSING_TOP_M
    return depth


def test_the_item_alone_is_measured_normally():
    """The control: in this very spot, without the housing, the item measures.

    Without it the next test would prove only that something in the frame is
    unmeasurable, not that the HOUSING is what makes it so.
    """
    assert measure_item(_frame(slice(200, 300), with_housing=False)) is not None


def test_an_item_beside_the_in_frame_housing_is_lost_without_a_word():
    """No exception, no partial result — just None, which the node reads as
    'no item on the belt'. That silence is the whole failure mode."""
    assert measure_item(_frame(slice(200, 300), with_housing=True)) is None


def test_the_loss_persists_frame_after_frame_so_waiting_does_not_help():
    """`_find_item` returns None for a partially visible item because the NEXT
    frame usually fixes it — the item rides out of the border and is measured.
    Against a STATIC housing that retry buys nothing: the item travels along the
    belt, its column offset never changes, and every frame of the measurement
    window is lost the same way."""
    travel_px = 4  # 1 m/s belt at 15 Hz over ~2.7 mm/px: ~25 px, but the mask
    for step in range(3):  # only has to stay merged, so a slow crawl is enough
        rows = slice(200 + step * travel_px, 300 + step * travel_px)
        assert measure_item(_frame(rows, with_housing=True)) is None, f"frame {step}"
