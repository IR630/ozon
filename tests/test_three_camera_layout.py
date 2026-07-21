# -*- coding: utf-8 -*-
"""The layout figure must agree with the geometry it claims to draw.

The brief names a side camera body inside the top head's cone as one of three
SILENT failures (it lands in the mask, _find_item returns None every frame), so
the clearance is asserted here rather than eyeballed off the picture.
"""
import numpy as np
import pytest

from scripts.plot_three_camera_layout import BODY_W_M, SIDE_Y_M, SIDE_Z_M
from src.constants import (
    CAMERA_RIG_POSES_M,
    CAMERA_SIDE_NEG_Y_POSE_M,
    CAMERA_SIDE_POS_Y_POSE_M,
    CAMERA_TOP_POSE_M,
)
from src.perception import BELT_TOP_Z_M, CAMERA_X_M, CAMERA_Y_M, CAMERA_Z_M, HFOV_RAD


def _cone_half_width_m(z_m):
    return (CAMERA_Z_M - z_m) * np.tan(HFOV_RAD / 2.0)


def test_side_camera_bodies_clear_the_top_cone():
    """At their mounting height the bodies must sit outside the cone."""
    nearest_edge = SIDE_Y_M - BODY_W_M / 2
    assert nearest_edge > _cone_half_width_m(SIDE_Z_M), "side body inside the top cone"


def test_clearance_is_governed_by_mounting_height_not_by_y():
    """The margin the arithmetic reports is not a property of |y| alone.

    Lowering the same head to belt level nearly puts it in frame, which is the
    part a single 'cleared by 234 mm' number hides — and the reason the layout
    constraint is recorded as a height constraint.
    """
    nearest_edge = SIDE_Y_M - BODY_W_M / 2
    at_mount = nearest_edge - _cone_half_width_m(SIDE_Z_M)
    at_belt = nearest_edge - _cone_half_width_m(BELT_TOP_Z_M)
    assert at_mount > 0.15, f"expected a comfortable margin, got {at_mount * 1000:.0f} mm"
    assert at_belt < 0.02, f"belt-level margin should be marginal, got {at_belt * 1000:.0f} mm"


def test_figure_renders():
    cv2 = pytest.importorskip("cv2")
    assert cv2 is not None
    import tempfile
    from pathlib import Path

    from scripts.plot_three_camera_layout import main

    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "layout.png"
        assert main([str(out)]) == 0
        assert out.stat().st_size > 5000


def test_rig_top_head_matches_the_calibration_perception_actually_uses():
    """The rig table and perception.py must describe the SAME top camera.

    perception.py carries the offline calibration of the saved frames and the rig
    table is the single source for all three heads; duplicating a domain constant
    is a bug per CLAUDE.md, so the duplication is turned into an invariant that
    fails loudly the moment the two drift apart.
    """
    (x, y, z), _look_at = CAMERA_TOP_POSE_M
    assert (x, y, z) == (CAMERA_X_M, CAMERA_Y_M, CAMERA_Z_M)


def test_rig_side_heads_are_mounted_where_the_drawing_says():
    """The figure and the constants must not diverge — the clearance argument is
    only valid for the pose it was computed at."""
    for pose, sign in ((CAMERA_SIDE_NEG_Y_POSE_M, -1), (CAMERA_SIDE_POS_Y_POSE_M, +1)):
        (_x, y, z), _look_at = pose
        assert y == pytest.approx(sign * SIDE_Y_M)
        assert z == pytest.approx(SIDE_Z_M)


def test_every_rig_head_clears_the_top_cone():
    """Whatever heads the rig grows, none of their bodies may sit in the top view."""
    for (x, y, z), _look_at in CAMERA_RIG_POSES_M[1:]:
        assert abs(y) - BODY_W_M / 2 > _cone_half_width_m(z), f"head at {(x, y, z)} in frame"
