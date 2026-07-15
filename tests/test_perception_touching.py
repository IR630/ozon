# -*- coding: utf-8 -*-
"""Touching-mask splitting: two products whose silhouettes touch form one
connected component, which measure_items() must split back into one measurement
per item (PLAN.md section 5, "close placement"). Synthetic frames only — the
math is analytic, so these run everywhere (numpy+scipy). The real Gazebo
counterpart is scripts/smoke_multi_item.sh with a reduced spawn interval.

Baseline before this split (documented limitation): a touching pair fused into
one hull and reported a single oversized blob; the tests below lock that it now
yields two correct measurements, and — just as important — that a single item is
NEVER split into phantoms, at any yaw.
"""
import numpy as np
import pytest

from src.classification import classify_conservative
from src.perception import _split_touching, measure_item, measure_items


def _corner_touch(belt=1.5, top=1.3):
    """Two 80x80 px boxes touching only at one diagonal corner (8-connected, no
    body overlap): the sharpest neck. Each -> (79+1)*1.3/500*1000 = 208 mm."""
    depth = np.full((480, 640), belt)
    depth[100:180, 200:280] = top
    depth[180:260, 280:360] = top
    return depth


def _offset_edge_touch(belt=1.5, top=1.3):
    """Two 100x100 px boxes touching along a partial vertical edge, offset 30 px
    (a shallow neck). Each -> (99+1)*1.3/500*1000 = 260 mm. The straight
    Euclidean split inflates the long side here; the geodesic grassfire keeps it
    at 260 (docs/experiments.md)."""
    depth = np.full((480, 640), belt)
    depth[150:250, 200:300] = top
    depth[180:280, 300:400] = top
    return depth


def _unequal_offset_edge_touch(belt=1.5, top=1.3):
    """A 50 px and an 80 px box sharing a 30 px offset edge."""
    depth = np.full((480, 640), belt)
    depth[175:225, 270:320] = top
    depth[195:275, 320:400] = top
    return depth


def _rotated_rect(l_px, w_px, angle_deg, top=1.3, belt=1.5):
    depth = np.full((480, 640), belt)
    yy, xx = np.mgrid[0:480, 0:640]
    a = np.deg2rad(angle_deg)
    dx, dy = xx - 320.0, yy - 240.0
    xr = np.cos(a) * dx + np.sin(a) * dy
    yr = -np.sin(a) * dx + np.cos(a) * dy
    depth[(np.abs(xr) <= l_px / 2) & (np.abs(yr) <= w_px / 2)] = top
    return depth


def _three_lobed_single_mask():
    """One concave product whose silhouette has three prominent EDT peaks.

    A binary silhouette cannot prove whether three lobes are three touching
    products or one irregular product.  Inventing three measurements is the
    unsafe outcome: the single product's dimensions and category are then
    computed from only its largest fragment.
    """
    mask = np.zeros((240, 300), dtype=bool)
    mask[80:140, 30:90] = True
    mask[30:90, 120:180] = True
    mask[100:160, 210:270] = True
    mask[80:120, 80:220] = True
    return mask


def _thin_bent_single_mask():
    """One thin bent product with two thick ends and two prominent EDT peaks."""
    yy, xx = np.mgrid[:260, :340]
    mask = ((xx - 60) ** 2 + (yy - 50) ** 2 <= 20 ** 2)
    mask |= ((xx - 280) ** 2 + (yy - 210) ** 2 <= 20 ** 2)
    mask[46:55, 60:281] = True
    mask[50:211, 276:285] = True
    return mask


def _long_tie_rod_single_mask():
    """One long relief body with two wide ends and a narrow middle waist.

    This is the binary analogue of the real Cylinder stream frames: its two
    prominent EDT peaks are parts of ONE 10:1 product, not two touching items.
    """
    mask = np.zeros((260, 120), dtype=bool)
    mask[20:40, 50:70] = True
    mask[40:200, 58:62] = True
    mask[200:220, 50:70] = True
    return mask


def test_corner_touch_splits_into_two_correct_items():
    # go/no-go: two touching boxes -> two measurements, each its OWN 208x208x200,
    # never one fused ~400 mm blob spanning both.
    items = measure_items(_corner_touch(), belt_depth_m=1.5, fx=500.0, fy=500.0)
    assert len(items) == 2
    assert items[0].position_m[0] < items[1].position_m[0]   # sorted, distinct
    for m in items:
        assert m.dims_mm == pytest.approx([208.0, 208.0, 200.0], abs=15.0)


def test_offset_edge_touch_keeps_each_true_footprint():
    # the geodesic split must cut along the neck, so each body keeps its 260 mm
    # footprint; a straight Euclidean cut would inflate the long side past 300.
    items = measure_items(_offset_edge_touch(), belt_depth_m=1.5, fx=500.0, fy=500.0)
    assert len(items) == 2
    for m in items:
        assert m.dims_mm == pytest.approx([260.0, 260.0, 200.0], abs=20.0)


def test_unequal_offset_touch_keeps_both_products():
    """The optional split keeps two IDs/categories on a harder asymmetric neck.

    Grassfire does not recover each exact rectangle on this wide offset contact;
    that geometric limitation is preferable to fusing both into one 338 mm blob.
    """
    items = measure_items(_unequal_offset_edge_touch(), belt_depth_m=1.5,
                          fx=500.0, fy=500.0)

    assert len(items) == 2
    assert all(max(item.dims_mm) < 260.0 for item in items)
    assert all(classify_conservative(item.dims_mm, item.k) == "B" for item in items)


def test_touching_pair_is_one_connected_component():
    # guards that the fixtures actually exercise the SPLIT (one component in, two
    # out) and not merely two already-disconnected blobs.
    from scipy.ndimage import label

    for depth in (_corner_touch(), _offset_edge_touch()):
        fg = (depth > 0) & (depth < 1.5 - 0.005)
        assert label(fg, structure=np.ones((3, 3), dtype=int))[1] == 1


@pytest.mark.parametrize("angle", [0, 20, 45, 70])
def test_single_rotated_item_is_never_oversplit(angle):
    # THE regression guard: a single convex item at any yaw stays ONE measurement.
    # A rotated rectangle rasterizes into a ridge of many equal EDT maxima, which
    # a naive peak count splits into phantoms (caught at 20/70 deg); the h-maxima
    # prominence test and the solidity fast path must both hold it together.
    items = measure_items(_rotated_rect(200, 120, angle), belt_depth_m=1.5,
                          fx=500.0, fy=500.0)
    assert len(items) == 1
    long_mm = (200 + 1) * 1.3 / 500.0 * 1000.0
    assert items[0].dims_mm[0] == pytest.approx(long_mm, rel=0.04)


def test_ambiguous_three_peak_single_is_not_split_into_phantoms():
    mask = _three_lobed_single_mask()

    pieces = _split_touching(mask)

    assert len(pieces) == 1
    assert np.array_equal(pieces[0], mask)


def test_thin_bent_two_peak_single_is_not_split_into_phantoms():
    mask = _thin_bent_single_mask()

    pieces = _split_touching(mask)

    assert len(pieces) == 1
    assert np.array_equal(pieces[0], mask)


def test_long_two_peak_single_is_not_split_into_phantoms():
    """Regression for Cylinder in a dense B stream (one body became 3 IDs)."""
    mask = _long_tie_rod_single_mask()

    pieces = _split_touching(mask)

    assert len(pieces) == 1
    assert np.array_equal(pieces[0], mask)


def test_split_partitions_the_mask_exactly():
    # every input pixel lands in exactly one sub-mask: no pixels invented, lost,
    # or double-counted along the seam.
    depth = _corner_touch()
    mask = (depth > 0) & (depth < 1.5 - 0.005)
    pieces = _split_touching(mask)
    assert len(pieces) == 2
    union = np.zeros_like(mask)
    overlap = np.zeros_like(mask)
    for piece in pieces:
        overlap |= union & piece
        union |= piece
    assert not overlap.any()
    assert np.array_equal(union, mask)


def test_convex_single_item_is_not_split():
    # a lone axis-aligned box (solidity 1.0) returns unchanged from the splitter.
    depth = np.full((480, 640), 1.5)
    depth[100:200, 150:250] = 1.3
    mask = (depth > 0) & (depth < 1.5 - 0.005)
    assert len(_split_touching(mask)) == 1


def test_tiny_depth_speck_is_filtered_before_touch_split():
    """Regression for the seed-1 stream: noise must not reach scipy ConvexHull."""
    depth = np.full((480, 640), 1.5)
    depth[100, 100] = 1.3
    depth[180:260, 280:360] = 1.3

    items = measure_items(depth, belt_depth_m=1.5, fx=500.0, fy=500.0)

    assert len(items) == 1
    assert items[0].dims_mm == pytest.approx([208.0, 208.0, 200.0], abs=15.0)


def test_partial_item_at_frame_edge_is_ignored_without_crashing():
    """Regression for the seed-1 stream, on the exact geometry from its log: the
    first visible row of an entering item (y=479, x=268..374) is a 1-D component
    far above _MIN_ITEM_PX, so it reaches _split_touching, where a 1-D point
    cloud is what Qhull rejected. It must come back unsplit and be dropped as a
    zero-height, frame-edge blob — no crash, no phantom item.
    """
    depth = np.full((480, 640), 1.5)
    depth[479, 268:375] = 1.3

    assert measure_items(depth, belt_depth_m=1.5, fx=500.0, fy=500.0) == []


def test_diagonal_one_pixel_speck_is_rejected_without_crashing():
    """A rank-1 component may span both axes but is not a measurable body."""
    depth = np.full((480, 640), 1.5)
    diagonal = np.arange(100, 130)
    depth[diagonal, diagonal] = 1.3

    assert measure_item(depth, belt_depth_m=1.5, fx=500.0, fy=500.0) is None
    assert measure_items(depth, belt_depth_m=1.5, fx=500.0, fy=500.0) == []


def test_qhull_failure_falls_back_to_one_component(monkeypatch):
    """Touch splitting is optional: a degenerate hull must not kill perception."""
    import scipy.spatial

    mask = (_corner_touch() < 1.5)

    def fail_hull(_points):
        raise scipy.spatial.QhullError("synthetic degenerate component")

    monkeypatch.setattr(scipy.spatial, "ConvexHull", fail_hull)

    pieces = _split_touching(mask)

    assert len(pieces) == 1
    assert np.array_equal(pieces[0], mask)
