# -*- coding: utf-8 -*-
"""Multi-head fusion: the three ways the brief says this fails SILENTLY.

Sync skew, a lost head, and K leaking off the top view are all failures that
produce plausible numbers rather than an exception, so each gets a test that
would catch it by value.
"""
import numpy as np
import pytest

from src.constants import (
    BELT_SPEED_M_S,
    CAMERA_SIDE_NEG_Y_POSE_M,
    CAMERA_TOP_POSE_M,
)
from src.multiview import (
    camera_axes,
    compensate_belt_travel,
    crop_to_item,
    fuse_dims_mm,
    world_cloud_from_depth,
)
from src.perception import BELT_TOP_Z_M


def test_top_head_axes_look_straight_down():
    """A downward view is the degenerate case for a naive up-vector; the frame
    must stay orthonormal there rather than collapse."""
    right, down, forward = camera_axes(CAMERA_TOP_POSE_M)
    assert forward == pytest.approx([0.0, 0.0, -1.0], abs=1e-9)
    for a, b in ((right, down), (right, forward), (down, forward)):
        assert float(np.dot(a, b)) == pytest.approx(0.0, abs=1e-9)
        assert np.linalg.norm(a) == pytest.approx(1.0)


def test_side_head_axes_are_orthonormal_and_face_the_belt():
    right, down, forward = camera_axes(CAMERA_SIDE_NEG_Y_POSE_M)
    assert forward[1] > 0.9, "the -y head must look toward +y"
    assert forward[2] < 0.0, "and slightly downward"
    for a, b in ((right, down), (right, forward), (down, forward)):
        assert float(np.dot(a, b)) == pytest.approx(0.0, abs=1e-9)


def test_backprojection_puts_the_belt_where_the_belt_is():
    """A flat frame at the belt distance from the top head must land on z=0.4."""
    fx = fy = 552.5
    depth = np.full((48, 64), 1.5)
    pts = world_cloud_from_depth(depth, CAMERA_TOP_POSE_M, fx, fy)
    assert len(pts) == 48 * 64
    assert pts[:, 2] == pytest.approx(BELT_TOP_Z_M, abs=1e-9)


def test_zero_and_invalid_depth_do_not_become_a_blob_at_the_lens():
    """Dropped returns must be discarded, not projected to the camera origin,
    where they would read as a solid object right at the sensor."""
    fx = fy = 552.5
    depth = np.full((16, 16), 1.5)
    depth[0, 0] = 0.0
    depth[0, 1] = np.nan
    depth[0, 2] = np.inf
    pts = world_cloud_from_depth(depth, CAMERA_TOP_POSE_M, fx, fy)
    assert len(pts) == 16 * 16 - 3
    assert np.all(np.isfinite(pts))


def test_belt_travel_compensation_cancels_a_full_frame_of_skew():
    """66.7 ms at 1 m/s is 66.7 mm — 13x the 5 mm budget. It must come back."""
    pts = np.zeros((10, 3))
    dt = 1.0 / 15.0
    moved = compensate_belt_travel(pts, dt)
    assert moved[:, 0] == pytest.approx(dt * BELT_SPEED_M_S)
    assert float(moved[:, 0].mean()) == pytest.approx(0.0667, abs=0.0005)
    # and it is a pure translation along the belt
    assert moved[:, 1:] == pytest.approx(pts[:, 1:])


def test_uncompensated_skew_would_widen_the_item_by_the_travel():
    """States the defect the compensation exists for, so a regression is visible
    as a number rather than as a slightly-off measurement."""
    box = np.array([[1.5, 0.0, 0.45], [1.6, 0.0, 0.45]])
    skewed = compensate_belt_travel(box, 1.0 / 15.0)
    spread_uncompensated = np.ptp(np.vstack([box, box + [0.0667, 0, 0]])[:, 0])
    spread_compensated = np.ptp(np.vstack([box, skewed - [0.0667, 0, 0]])[:, 0])
    assert spread_uncompensated == pytest.approx(0.1667, abs=0.001)
    assert spread_compensated == pytest.approx(0.1, abs=0.001)


def test_crop_drops_the_belt_and_keeps_the_item():
    pts = np.array([
        [1.5, 0.0, BELT_TOP_Z_M],          # belt plane -> out
        [1.5, 0.0, BELT_TOP_Z_M + 0.05],   # item -> in
        [4.0, 0.0, BELT_TOP_Z_M + 0.05],   # far downstream -> out
    ])
    kept = crop_to_item(pts, (1.5, 0.0, 0.45), [100.0, 100.0, 100.0])
    assert len(kept) == 1
    assert kept[0][0] == pytest.approx(1.5)


def test_a_lost_head_degrades_to_the_top_measurement_instead_of_crashing():
    """Brief boundary: a missing head must degrade the node, not kill it."""
    top = [300.0, 200.0, 100.0]
    assert fuse_dims_mm(top, [], (1.5, 0.0, 0.45)) == top
    assert fuse_dims_mm(top, [None], (1.5, 0.0, 0.45)) == top
    assert fuse_dims_mm(top, [np.empty((0, 3))], (1.5, 0.0, 0.45)) == top


def test_a_degenerate_side_cloud_falls_back_to_the_top_measurement():
    """Three collinear points cannot form a hull; that must read as 'no help',
    not as an exception on the live belt."""
    top = [300.0, 200.0, 100.0]
    line = np.array([[1.5, 0.0, 0.45], [1.5, 0.001, 0.45], [1.5, 0.002, 0.45],
                     [1.5, 0.003, 0.45]])
    assert fuse_dims_mm(top, [line], (1.5, 0.0, 0.45)) == top


def test_side_heads_may_only_add_hidden_extent_never_carve_the_top_view_away():
    """The top head saw the item unoccluded from above. A partially visible flank
    must not shrink a correct measurement below what was directly observed."""
    top = [300.0, 200.0, 100.0]
    tiny = np.array([[1.5, 0.0, 0.45], [1.51, 0.0, 0.45],
                     [1.5, 0.01, 0.45], [1.5, 0.0, 0.46]])
    fused = fuse_dims_mm(top, [tiny], (1.5, 0.0, 0.45))
    assert fused[0] >= top[0] - 1e-6
    assert fused[1] >= top[1] - 1e-6


def test_stale_side_frame_policy_is_tighter_than_the_error_it_would_admit():
    """A dropped head is safe; a silently stale one is not.

    Compensation fixes the belt TRANSLATION only — whatever rotated or settled
    between the two frames stays wrong. The cutoff must therefore sit at a lag
    whose uncorrectable residual is still recognisable as an error, not at one
    that quietly doubles the item.
    """
    from src.constants import CAMERA_FRAME_PERIOD_S, CAMERA_SIDE_STALE_FRAMES

    worst_lag_s = CAMERA_SIDE_STALE_FRAMES * CAMERA_FRAME_PERIOD_S
    worst_travel_mm = worst_lag_s * BELT_SPEED_M_S * 1000.0
    assert worst_travel_mm == pytest.approx(133.3, abs=1.0)
    assert CAMERA_SIDE_STALE_FRAMES >= 1.0, "must tolerate one full period of jitter"
    assert CAMERA_SIDE_STALE_FRAMES <= 3.0, "beyond this the frame is another item"
