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
    """The belt floor must not swallow the thinnest item's points.

    HISTORICAL NOTE, because this docstring used to claim the opposite and the
    claim was wrong: it read "the 9 mm pen is WHY the heads are there". It is not.
    The top head measures the Pen correctly at 9 mm; a side head resolves only
    3-4 mm per pixel at this range and is the head that gets it wrong (see
    SIDE_HEIGHT_MIN_GAIN_MM and the two tests below). What this test still pins is
    narrower and true: the grazing floor is a BELT filter, so it must sit under
    the thinnest item rather than clipping it away.
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


def _pen_side_cloud(height_above_belt_m, n=400, seed=0):
    """A side head's view of the Pen's flank: plenty of points, almost no height.

    148 mm long and 9-11 mm tall, which is the whole trap — the point COUNT clears
    SIDE_MIN_POINTS easily while the height is 2-3 pixels for this head.
    """
    rng = np.random.default_rng(seed)
    return np.column_stack([
        rng.uniform(1.426, 1.574, n),                       # 148 mm of length
        rng.uniform(-0.0065, 0.0065, n),                    # 13 mm of width
        BELT_TOP_Z_M + rng.uniform(0.0, height_above_belt_m, n),
    ])


def test_the_pen_keeps_its_size_protection_when_a_side_head_noises_upward():
    """The measured regression, pinned where it actually happened.

    Identical cell, seed 1: one head read 147x11x9 and the 9 mm under MIN_DIMS_MM
    made the item "small" -> C, correct. The rig with side heads read 147x11x11,
    the item stopped being small, and SHAPE then decided the verdict -> D, a miss.
    The fusion measured nothing worse: it lifted the item out from under the size
    rule, and 2 mm of lift is 0.6 of a pixel for this head.

    The pre-existing pin for this item feeds classify() a ready 9 mm, so it passes
    either way and could not catch this. This one starts from the DIMS.
    """
    from src.classification import CATEGORY_C, classify
    from src.constants import MIN_DIMS_MM

    top = [147.0, 11.0, 9.0]
    top_z = BELT_TOP_Z_M + 0.0045                           # top_height = 9 mm
    side = _pen_side_cloud(0.011)                           # side would read ~11 mm
    assert len(side) > 30, "the point-count guard must NOT be what saves this"

    fused = fuse_dims_mm(top, [side], (1.5, 0.0, top_z))
    assert min(fused) == pytest.approx(9.0), f"the 9 mm was inflated to {min(fused)}"
    assert min(fused) < MIN_DIMS_MM[0], "the item lost its size protection"
    assert classify(fused, 1.0) == CATEGORY_C, "shape decided a verdict size owned"


def test_fusion_never_lifts_a_dimension_across_the_small_item_threshold():
    """The structural invariant behind that miss, stated once for any item.

    `fuse_dims_mm` raises height and never lowers it, so without a gate it can walk
    a sub-threshold dimension over MIN_DIMS_MM and silently move the verdict from
    the size rule to the shape rule. Swept across the whole approach to the
    threshold so a future change to the gate cannot re-open it at one value.
    """
    from src.constants import MIN_DIMS_MM

    floor_mm = MIN_DIMS_MM[0]
    for true_h_mm in (5.0, 6.0, 7.0, 8.0, 9.0, 9.5):
        top = [147.0, 11.0, true_h_mm]
        top_z = BELT_TOP_Z_M + true_h_mm / 2000.0
        # A side head reading anywhere up to just over the threshold
        for side_h_mm in (true_h_mm + 0.5, true_h_mm + 2.0, floor_mm + 0.5):
            fused = fuse_dims_mm(top, [_pen_side_cloud(side_h_mm / 1000.0)],
                                 (1.5, 0.0, top_z))
            assert min(fused) < floor_mm, (
                f"true {true_h_mm} mm + side {side_h_mm} mm -> {min(fused)} mm, "
                "over the small-item floor")


def test_the_boundary_refusal_costs_a_genuinely_taller_item_and_that_is_chosen():
    """The price of the refusal above, pinned so it stays visible rather than lost.

    An item truly ~10.5 mm tall that the top head under-read to 5 mm keeps its
    "small" classification and routes C. That is a real misroute on a catalogue
    that contained such an item. Ours does not: the Pen at 9 mm is the only body
    near the floor and the next thinnest is 91 mm. The trade is deliberate —
    protecting the Pen is worth an error no item in this catalogue can commit.
    """
    from src.constants import MIN_DIMS_MM

    top = [147.0, 11.0, 5.0]
    top_z = BELT_TOP_Z_M + 0.0025
    fused = fuse_dims_mm(top, [_pen_side_cloud(0.0105)], (1.5, 0.0, top_z))
    assert fused == top, "the refusal is not in force"
    assert min(fused) < MIN_DIMS_MM[0]


def test_a_gain_below_the_head_resolution_is_not_evidence_of_hidden_height():
    """The gate itself: one pixel is 3.4 mm by geometry, 4.2 mm on a real frame.

    A gain under that is noise, not a hidden dome, and must degrade BIT-EXACT to
    the top reading. The Helmet's real gain is tens of mm and is pinned above.
    """
    from src.constants import SIDE_HEIGHT_MIN_GAIN_MM

    assert SIDE_HEIGHT_MIN_GAIN_MM >= 4.2, "below the measured pixel size"
    top = [300.0, 200.0, 100.0]
    top_z = BELT_TOP_Z_M + 0.050                            # top_height = 100 mm
    rng = np.random.default_rng(1)
    n = 2000
    barely = np.column_stack([
        np.full(n, 1.5), rng.uniform(-0.05, 0.05, n),
        BELT_TOP_Z_M + rng.uniform(0.0, 0.100 + (SIDE_HEIGHT_MIN_GAIN_MM - 1.0) / 1000.0, n),
    ])
    assert fuse_dims_mm(top, [barely], (1.5, 0.0, top_z)) == top


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
