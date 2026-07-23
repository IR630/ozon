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


def test_the_belt_a_miscalibrated_head_reconstructs_is_rejected():
    """The census this margin was raised for, pinned by its measured number.

    At the +-2 mm / 0.2 deg calibration budget the belt reconstructs 5.00-5.23 mm
    above itself (measured on dumped frames, scripts/diagnose_side_clouds.sh). The
    shipped 5 mm floor let it through, the crop then admitted two strips of belt
    edge across its whole window, and a 303 mm bottle read 740x505 mm.
    """
    from src.constants import SIDE_BELT_MARGIN_M

    worst_leak_m = BELT_TOP_Z_M + 0.00523
    belt = np.array([[1.4, 0.22, worst_leak_m], [1.9, -0.22, worst_leak_m]])
    assert len(crop_to_item(belt, (1.5, 0.0, BELT_TOP_Z_M), [303.0, 94.0, 91.0])) == 0
    assert SIDE_BELT_MARGIN_M > 0.00523, "the margin no longer clears the measured leak"


def test_the_margin_still_leaves_the_thinnest_item_visible_to_the_side_heads():
    """And the other side of the trade: the 9 mm pen is WHY the heads are there.

    A floor set above the pen would make the rig degrade to one camera on exactly
    the item that motivated the third.
    """
    from src.constants import MIN_DIMS_MM, SIDE_BELT_MARGIN_M

    assert SIDE_BELT_MARGIN_M * 1000.0 < 9.0 < MIN_DIMS_MM[0]
    pen_top = np.array([[1.5, 0.0, BELT_TOP_Z_M + 0.009]])
    assert len(crop_to_item(pen_top, (1.5, 0.0, BELT_TOP_Z_M), [148.0, 13.0, 9.0])) == 1


def test_the_grazing_margin_is_deliberately_not_the_top_view_margin():
    """Reusing the top head's 5 mm here is the defect, so the two are pinned apart.

    A downward view moves the belt SIDEWAYS under a pointing error; a grazing
    view tilts the plane about the lens and lifts it by the error times the
    range. Same budget, different geometry, different floor.
    """
    from src.constants import SIDE_BELT_MARGIN_M
    from src.perception import MASK_MARGIN_M

    assert SIDE_BELT_MARGIN_M > MASK_MARGIN_M, "the grazing floor collapsed back onto the top one"
    # 2 mm of translation plus 0.2 deg over the longest range the crop admits
    bound_m = 0.002 + np.radians(0.2) * 1.28
    assert SIDE_BELT_MARGIN_M >= bound_m, f"below the {bound_m * 1000:.1f} mm calibration bound"


def test_a_side_head_raises_the_height_the_top_under_read():
    """Arbitration's whole job: the side head reveals the hidden vertical extent.

    The top read the helmet dome 100 mm tall (its visible cap); a side head that sees
    the body reach 150 mm above the belt must lift the height to 150, leaving the two
    lateral dims — which the top owns — untouched.
    """
    top = [300.0, 200.0, 100.0]                        # top_height = 100 mm at z=0.45
    rng = np.random.default_rng(0)
    # A realistic side cloud (thousands of points) reaching 150 mm above the belt,
    # plus a few 200 mm strays the 99.5th percentile must reject (< 0.5 % of points)
    # so a single noisy return does not set the dimension.
    n = 2000
    side = np.column_stack([
        np.full(n, 1.5), rng.uniform(-0.05, 0.05, n),
        BELT_TOP_Z_M + rng.uniform(0.0, 0.150, n),
    ])
    side[:3, 2] = BELT_TOP_Z_M + 0.200
    fused = fuse_dims_mm(top, [side], (1.5, 0.0, 0.45))
    assert fused[0] == pytest.approx(300.0)
    assert fused[1] == pytest.approx(200.0)
    assert fused[2] == pytest.approx(150.0, abs=6.0)   # ~150, not 100 and not 200
    assert fused[2] > 100.0


def test_too_few_side_points_degrade_rather_than_read_a_dim_off_a_sliver():
    """A handful of grazing points is a sliver of flank, not a measurement."""
    top = [300.0, 200.0, 100.0]
    sliver = np.column_stack([np.full(10, 1.5), np.zeros(10),
                              np.full(10, BELT_TOP_Z_M + 0.150)])
    assert fuse_dims_mm(top, [sliver], (1.5, 0.0, 0.45)) == top


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
