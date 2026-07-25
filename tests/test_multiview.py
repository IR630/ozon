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


def test_fusion_can_only_inflate_never_reduce_the_top_measurement():
    """The general form of the floor at src/multiview.py:141, over all three dims.

    That line returns a component-wise max against sorted(top_dims), so the sign
    of the change a side head can make is fixed by construction: never down. The
    neighbouring hand-made case pins one flank on two components; this one sweeps
    seeded flanks — narrow, wide, tall, several at once — over all three, which is
    what lets the report call the sign a theorem rather than a measurement.
    """
    rng = np.random.default_rng(20260725)
    top = [303.0, 94.0, 91.0]          # bottle, as production hands it over: desc
    floor = sorted(top, reverse=True)
    for _ in range(40):
        clouds = []
        for _head in range(int(rng.integers(1, 4))):
            n = int(rng.integers(4, 80))
            spread = float(rng.uniform(0.002, 0.25))
            clouds.append(np.column_stack([
                1.5 + rng.normal(0.0, spread, n),
                rng.normal(0.0, spread, n),
                BELT_TOP_Z_M + rng.uniform(0.001, 0.30, n),
            ]))
        fused = fuse_dims_mm(top, clouds, (1.5, 0.0, 0.45))
        assert all(f >= t - 1e-6 for f, t in zip(fused, floor)), \
            f"{fused} fell below the top head's {floor}"


def test_fusion_cannot_undo_an_overmeasurement_by_the_top_head():
    """The cost side of that floor, and the reason a third head cannot buy points.

    Same line, src/multiview.py:141, read the other way: if the TOP head is the
    one that is wrong high, no side view can pull the number back — the max keeps
    the overmeasurement. The case is real: a prone bottle, top-view width 102 mm
    against an intrinsic 91 mm (the hull rolls over its near-vertical flanks), and
    a side head that sees the true 91 mm has nothing to squeeze with.

    Why this test decides a rig question rather than describing one: both misses
    left in the 163/165 census are an overmeasurement and a K sitting on its
    threshold, and both belong to the TOP head. This test says heads cannot fix
    either, so a three-head multi-seed census could not have moved 163/165 — which
    is why that census (5-10 h of Gazebo) was deliberately not run.
    """
    truth = [303.0, 91.0, 91.0]
    top_overmeasured = [303.0, 102.0, 102.0]
    # a side cloud generous beyond what one head can see: the WHOLE 91 mm section
    # of the lying bottle, so the squeeze fails on the floor and not on bad input
    radius_m = 0.0455
    phi = np.linspace(0.0, 2.0 * np.pi, 48, endpoint=False)
    gx, gp = np.meshgrid(np.linspace(1.5 - 0.1515, 1.5 + 0.1515, 16), phi)
    honest_flank = np.column_stack([
        gx.ravel(),
        radius_m * np.cos(gp).ravel(),
        BELT_TOP_Z_M + radius_m + radius_m * np.sin(gp).ravel()])
    # that cloud alone does resolve the true body, so the fusion path really runs
    on_its_own = fuse_dims_mm([10.0, 10.0, 10.0], [honest_flank], (1.5, 0.0, 0.45))
    assert on_its_own[1] == pytest.approx(truth[1], abs=0.5)
    fused = fuse_dims_mm(top_overmeasured, [honest_flank], (1.5, 0.0, 0.45))
    assert fused[1] >= 102.0 - 1e-6, "the side head must not be able to squeeze"
    assert fused[1] > truth[1], "and so the 11 mm overmeasurement survives fusion"


def test_the_relief_gate_is_always_open_on_a_grazing_side_cloud():
    """The tilted-OBB guard is calibrated on TOP views and cannot fire on side ones.

    BODY_OBB_MIN_RELIEF (src/perception.py:559) is a RELATIVE gate: p95-p05 of the
    cloud's height must reach half the item's height or no tilted box is allowed.
    Its calibration populations are named at the constant — "boxes/detergent ~0.05
    vs dome-family >=0.9" — and both are relief as the TOP head sees it, where a
    box lid is flat and scores ~0.

    src/multiview.py:133-136 feeds the same function a SIDE cloud. A grazing head
    sees the vertical wall, so the cloud spans nearly the whole body height and the
    ratio is ~1 for the very box the gate exists to protect: on the side path the
    only safeguard against tilting a minimum-volume box through a hidden body is
    open unconditionally. That is the mechanism behind the measured regression of
    naive fusion (23/33 in tolerance against 27/33 for the top head alone), which
    until now stood in the report as a number without a cause.
    """
    from src.constants import SIDE_BELT_MARGIN_M
    from src.perception import BODY_OBB_MIN_RELIEF, _body_obb_dims_mm, _obb_dims_px

    box_mm = (200.0, 120.0, 80.0)      # one carton, seen twice
    xs_m = np.linspace(1.5 - 0.100, 1.5 + 0.100, 40)

    def gate_verdict(pts):
        """Resolve the cloud exactly as fuse_dims_mm does, and report the ratio."""
        from scipy.spatial import ConvexHull

        heights_m = pts[:, 2] - BELT_TOP_Z_M
        dz_mm = max(float(heights_m.max()) * 1000.0, min(box_mm))
        hull = ConvexHull(pts[:, :2] * 1000.0)
        long_mm, short_mm, _dir = _obb_dims_px((pts[:, :2] * 1000.0)[hull.vertices])
        shadow = sorted([float(long_mm), float(short_mm), dz_mm], reverse=True)
        relief_mm = float(np.percentile(heights_m, 95.0)
                          - np.percentile(heights_m, 5.0)) * 1000.0
        body = _body_obb_dims_mm(
            xs=pts[:, 0], ys=pts[:, 1], depth_col_m=np.ones(len(pts)),
            heights_m=heights_m, fx=1.0, fy=1.0, cx=0.0, cy=0.0,
            legacy_dims_mm=tuple(shadow), dz_mm=dz_mm, px_pad_mm=0.0)
        return body, relief_mm / dz_mm

    # the grazing view: the -y wall from the crop floor up, plus the sliver of lid
    # a downward-tilted head catches over the top edge
    gx, gz = np.meshgrid(xs_m, np.linspace(BELT_TOP_Z_M + SIDE_BELT_MARGIN_M,
                                           BELT_TOP_Z_M + 0.080, 24))
    wall = np.column_stack([gx.ravel(), np.full(gx.size, -0.060), gz.ravel()])
    lx, ly = np.meshgrid(xs_m, np.linspace(-0.060, -0.030, 6))
    lid_sliver = np.column_stack([lx.ravel(), ly.ravel(),
                                  np.full(lx.size, BELT_TOP_Z_M + 0.080)])
    body_side, ratio_side = gate_verdict(np.vstack([wall, lid_sliver]))

    # the same carton from above: the lid, flat, which is what the gate was tuned on
    tx, ty = np.meshgrid(xs_m, np.linspace(-0.060, 0.060, 24))
    lid = np.column_stack([tx.ravel(), ty.ravel(),
                           np.full(tx.size, BELT_TOP_Z_M + 0.080)])
    body_top, ratio_top = gate_verdict(lid)

    assert body_top is None, "the gate must still block a tilted box on a flat lid"
    assert ratio_top < BODY_OBB_MIN_RELIEF
    assert body_side is not None, "the gate no longer opens on a wall — retune this test"
    assert ratio_side > BODY_OBB_MIN_RELIEF
    # and not by a hair: the side view is on the far side of the gate, always
    assert ratio_side > 0.8
    # what the open gate then licenses, in numbers: 200x72x30 against a shadow of
    # 200x80x30 on a carton 80 mm tall. The "no candidate thinner than the measured
    # height" correction (src/perception.py:641) guards only the axis along the
    # winning facet normal; on a grazing cloud that axis is horizontal (the 200 mm
    # length here), so the height lands on an unguarded in-plane axis and comes back
    # short by exactly the crop floor the side head is not allowed to see under.
    assert body_side == pytest.approx([200.0, 72.0, 30.0], abs=0.5)
    assert min(box_mm) - body_side[1] == pytest.approx(SIDE_BELT_MARGIN_M * 1000.0,
                                                      abs=0.5)


def test_adding_a_second_side_head_is_not_guaranteed_monotone():
    """The floor is against the TOP dims, not against the answer with fewer heads.

    src/multiview.py:141 compares with `top_dims_mm`, so nothing in the module
    orders fuse(top, {s1}) against fuse(top, {s1, s2}) — a dimension may DROP when
    a head is added. The witness below is the cheapest of the three channels:
    _obb_dims_px minimises AREA (src/perception.py:540), and area is what grows
    monotonically under more points, not the individual sides. Both clouds sit at
    one height, so the relief gate is shut for both and the whole difference is in
    that rectangle: 100x4 mm alone, 78x77 mm once the second head contributes, and
    the long side falls 100 -> 78 mm while the area rises 400 -> 6000 mm^2.

    (The other two channels are structural too and are not exercised here: the
    percentile relief gate can be switched shut by added points, and the 1500-point
    decimator at src/perception.py:600 resamples, so the point that held a dimension
    need not survive a larger cloud.) The consequence for the report: the SIGN of
    the change against one head is a theorem, but "9.0 -> 9.4 -> 10.2 mm on the pen
    at 1/2/3 heads" is an empirical curve on our catalogue, not a law.
    """
    top = [10.0, 10.0, 10.0]           # the smallest admissible item: floor out of the way
    z = BELT_TOP_Z_M + 0.030

    def flat(xy_mm):
        return np.array([[1.5 + x / 1000.0, y / 1000.0, z] for x, y in xy_mm])

    s1 = flat([(0, 0), (100, 0), (50, 2), (50, -2)])
    s2 = flat([(50, 60), (49, 59), (51, 59), (50, 58)])
    one_head = fuse_dims_mm(top, [s1], (1.5, 0.0, 0.45))
    two_heads = fuse_dims_mm(top, [s1, s2], (1.5, 0.0, 0.45))
    assert one_head[0] == pytest.approx(99.9, abs=0.1)
    assert two_heads[0] == pytest.approx(78.1, abs=0.1)
    assert two_heads[0] < one_head[0], "no counterexample: re-derive the report's claim"
    # both still clear the top head's floor — T1 holds even where monotonicity does not
    for fused in (one_head, two_heads):
        assert all(f >= t - 1e-6 for f, t in zip(fused, sorted(top, reverse=True)))
