# -*- coding: utf-8 -*-
"""The side-camera probe must not flatter a second view by construction."""
import numpy as np
import pytest

from scripts.probe_side_camera import (
    BELT_TOP_Z_M,
    SIDE_CAM_POS_M,
    SIDE_CAM_TARGET_M,
    TOP_CAM_POS_M,
    TOP_CAM_TARGET_M,
    cloud_dims_mm,
    seeded_quats,
    visible_points,
)


def _box_surface(size_m, centre_xy_m, n=250):
    """Dense surface samples of an axis-aligned box resting on the belt.

    `n` must keep the sample spacing FINER than a pixel footprint (~2.4 mm at
    belt depth), or the near face is full of holes and the z-buffer legitimately
    reports the far one through them -- a sampling artifact, not occlusion.
    """
    sx, sy, sz = size_m
    lin = np.linspace(0.0, 1.0, n)
    a, b = np.meshgrid(lin, lin)
    a, b = a.ravel(), b.ravel()
    faces = [
        np.column_stack([a * sx, b * sy, np.zeros_like(a)]),          # bottom
        np.column_stack([a * sx, b * sy, np.full_like(a, sz)]),       # top
        np.column_stack([a * sx, np.zeros_like(a), b * sz]),          # -y flank
        np.column_stack([a * sx, np.full_like(a, sy), b * sz]),       # +y flank
        np.column_stack([np.zeros_like(a), a * sy, b * sz]),          # -x flank
        np.column_stack([np.full_like(a, sx), a * sy, b * sz]),       # +x flank
    ]
    pts = np.vstack(faces)
    pts[:, 0] += centre_xy_m[0] - sx / 2
    pts[:, 1] += centre_xy_m[1] - sy / 2
    pts[:, 2] += BELT_TOP_Z_M
    return pts


def test_top_view_cannot_see_the_underside():
    """The z-buffer is the occlusion model: a top camera keeps no bottom-face point."""
    pts = _box_surface((0.3, 0.2, 0.2), (TOP_CAM_POS_M[0], TOP_CAM_POS_M[1]))
    seen = visible_points(pts, TOP_CAM_POS_M, TOP_CAM_TARGET_M)
    assert len(seen) > 0
    # every visible point sits at (or above) the box top, never on the belt-level base
    assert seen[:, 2].min() > BELT_TOP_Z_M + 0.15


def test_side_view_keeps_the_near_flank_only():
    """A -Y side camera must not see through the body to the +Y flank."""
    pts = _box_surface((0.3, 0.2, 0.2), (TOP_CAM_POS_M[0], 0.0))
    seen = visible_points(pts, SIDE_CAM_POS_M, SIDE_CAM_TARGET_M)
    assert len(seen) > 0
    # The camera sits above the box top, so the TOP face is legitimately in view
    # and it reaches y = +0.1. Only the far vertical FLANK must stay hidden.
    far_flank = (seen[:, 1] > 0.09) & (seen[:, 2] < BELT_TOP_Z_M + 0.19)
    assert not far_flank.any(), "side camera saw through the item to the far flank"


def test_fused_cloud_measures_a_known_box():
    """Ground truth the probe must reproduce, or its verdict means nothing."""
    size_m = (0.3, 0.2, 0.2)
    pts = _box_surface(size_m, (TOP_CAM_POS_M[0], 0.0))
    top = visible_points(pts, TOP_CAM_POS_M, TOP_CAM_TARGET_M)
    side = visible_points(pts, SIDE_CAM_POS_M, SIDE_CAM_TARGET_M)
    dims = cloud_dims_mm(np.vstack([top, side]))
    assert dims == sorted(dims, reverse=True)
    for measured, truth_m in zip(dims, sorted(size_m, reverse=True)):
        assert measured == pytest.approx(truth_m * 1000.0, abs=12.0)


def test_empty_cloud_is_reported_not_guessed():
    assert cloud_dims_mm(np.zeros((0, 3))) is None


def test_poses_are_reproducible_from_the_seed():
    """Karpathy #5: the sweep must repeat exactly from its seed."""
    assert seeded_quats(4) == seeded_quats(4)
    assert seeded_quats(4)[0] == (0.0, 0.0, 0.0, 1.0)
    assert len(seeded_quats(4)) == 4
    for quat in seeded_quats(4):
        assert np.linalg.norm(quat) == pytest.approx(1.0)
