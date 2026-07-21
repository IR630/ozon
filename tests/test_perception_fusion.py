# -*- coding: utf-8 -*-
"""Two-camera fusion (feat/two-cameras): the side head refines DIMENSIONS only.

Everything the branch promises is pinned here against the frozen single-camera path:

  1. Belt-motion compensation actually undoes the inter-frame travel (66.7 mm at 1 m/s
     is 13x the tolerance; unioning uncompensated clouds smears it into the length).
  2. Fusing a known box's two visible faces recovers its true dimensions.
  3. No side stream -> bit-identical to main. This is the branch's hard availability
     contract (a dropped side camera must not perturb a single number), tested three
     ways: None, an empty cloud, and the default argument.
  4. The side head never touches K. K stays top-only and yaw-invariant, so a fused
     frame reports the SAME K as the top-only frame (a side view sees an end-on circle
     on every lying body and would wrongly route it to D).
  5. Monotone fusion: the side view may only GROW a dimension (reveal the hidden lower
     section of a lying body), never shrink one — a smaller fused box is the min-volume
     search tilting on a soft/round cloud, not a measurement. Plus a sparse-point guard,
     because a thin side cloud (the 9 mm pen) blows the OBB up instead of constraining it.
"""
import numpy as np

from scripts.render_depth import load_mesh, render_depth
from scripts.probe_side_camera import place_on_belt, visible_points

from src.classification import within_measurement_tolerance
from src.perception import (
    BELT_TOP_Z_M,
    FX,
    FY,
    IMG_H,
    IMG_W,
    SIDE_CAMERA_POS_M,
    SIDE_CAMERA_TARGET_M,
    _camera_basis,
    backproject_depth_to_world,
    compensate_belt_motion,
    fuse_dims_over_cloud_mm,
    measure_items,
    select_side_item_points,
    side_cloud_from_frame,
)


def _dome_quat(index=4):
    """A reproducible helmet tumble; index 4 is a dome pose the top view busts to C."""
    rng = np.random.default_rng(0)
    quats = [(0.0, 0.0, 0.0, 1.0)] + [tuple(q / np.linalg.norm(q)) for q in rng.normal(size=(6, 4))]
    return quats[index]


def _top_and_side(slug, quat):
    """(top depth frame, side world cloud) for `slug` in `quat`, same world placement."""
    mesh = load_mesh(slug)
    depth = render_depth(mesh, quat)
    world = place_on_belt(mesh, quat)
    side = visible_points(world, SIDE_CAMERA_POS_M, SIDE_CAMERA_TARGET_M)
    return depth, side


def test_compensate_belt_motion_undoes_travel():
    pts = np.array([[1.50, 0.0, 0.45], [1.52, 0.1, 0.50]])
    travelled = pts.copy()
    travelled[:, 0] += 1.0 * 0.0667  # item rode 66.7 mm in the 66.7 ms between frames
    back = compensate_belt_motion(travelled, dt_s=0.0667, belt_speed_m_s=1.0)
    assert np.allclose(back, pts, atol=1e-9)
    assert compensate_belt_motion(np.empty((0, 3)), dt_s=0.05).shape == (0, 3)


def test_backproject_inverts_the_side_projection():
    """A world point projected into the side frame and back must return to itself.

    Guards the node's geometry: depth-frame -> world uses the SAME pinhole convention
    the fusion tests project with, so the two never drift. Rasterizing to integer
    pixels costs sub-pixel error (~1.6 mm/px at 0.9 m), hence the 5 mm tolerance.
    """
    forward, right, up = _camera_basis(SIDE_CAMERA_POS_M, SIDE_CAMERA_TARGET_M)
    cam = np.asarray(SIDE_CAMERA_POS_M, dtype=float)
    world = np.array([[1.50, 0.02, 0.55], [1.42, -0.05, 0.48], [1.58, 0.08, 0.62]])
    for p in world:
        rel = p - cam
        z = rel @ forward
        u = int(round(IMG_W / 2.0 + (rel @ right) * FX / z))
        v = int(round(IMG_H / 2.0 - (rel @ up) * FY / z))
        depth = np.zeros((IMG_H, IMG_W))
        depth[v, u] = z
        got = backproject_depth_to_world(depth, SIDE_CAMERA_POS_M, SIDE_CAMERA_TARGET_M, FX, FY)
        assert len(got) == 1 and np.linalg.norm(got[0] - p) < 0.005, (p, got)


def _rasterize_side(world_pts):
    """A sparse side depth frame (z-buffered) of world points, for the node pipeline."""
    forward, right, up = _camera_basis(SIDE_CAMERA_POS_M, SIDE_CAMERA_TARGET_M)
    cam = np.asarray(SIDE_CAMERA_POS_M, dtype=float)
    depth = np.zeros((IMG_H, IMG_W))
    for p in world_pts:
        rel = p - cam
        z = rel @ forward
        u = int(round(IMG_W / 2.0 + (rel @ right) * FX / z))
        v = int(round(IMG_H / 2.0 - (rel @ up) * FY / z))
        if 0 <= u < IMG_W and 0 <= v < IMG_H and (depth[v, u] == 0 or z < depth[v, u]):
            depth[v, u] = z
    return depth


def test_side_cloud_from_frame_recovers_flank_and_compensates_motion():
    """The node's side pipeline end to end: frame -> world flank -> motion-registered."""
    world = np.random.default_rng(1).uniform([1.45, -0.10, 0.42], [1.55, -0.05, 0.55], size=(200, 3))
    depth = _rasterize_side(world)
    at_top = side_cloud_from_frame(depth, 0.0, FX, FY)
    assert len(at_top) > 50 and abs(at_top[:, 0].mean() - 1.5) < 0.02
    # a side frame stamped 66.7 ms after the top frame: its item rode +66.7 mm, so the
    # compensated cloud sits 66.7 mm back in x from the uncompensated one.
    compensated = side_cloud_from_frame(depth, 0.0667, FX, FY)
    assert abs(compensated[:, 0].mean() - (at_top[:, 0].mean() - 0.0667)) < 1e-6


def test_select_side_item_points_keeps_flank_drops_background():
    item = np.random.default_rng(0).uniform([1.45, -0.1, 0.42], [1.55, 0.1, 0.55], size=(50, 3))
    background = np.array([[3.0, 0.0, 0.45],      # far down-belt wall
                           [1.5, 1.2, 0.30],      # far guide, outside |y| gate
                           [1.5, 0.0, 0.39],      # below the belt band
                           [1.5, 0.0, 1.20]])     # gantry, above the band
    kept = select_side_item_points(np.vstack([item, background]), item_x_m=1.5)
    assert len(kept) == len(item), "must keep the flank and drop every background point"
    assert select_side_item_points(np.empty((0, 3)), item_x_m=1.5).shape == (0, 3)


def test_fuse_synthetic_box_recovers_true_dims():
    """A 300x200x100 box: its top face gives x,y; its -Y flank gives the hidden z."""
    xs = np.linspace(1.35, 1.65, 40)      # 300 mm along x, centered at 1.5
    ys = np.linspace(-0.10, 0.10, 30)     # 200 mm along y
    zs = np.linspace(BELT_TOP_Z_M, BELT_TOP_Z_M + 0.10, 20)  # 100 mm tall
    gx, gy = np.meshgrid(xs, ys)
    top = np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, BELT_TOP_Z_M + 0.10)])
    gx2, gz = np.meshgrid(xs, zs)
    flank = np.column_stack([gx2.ravel(), np.full(gx2.size, -0.10), gz.ravel()])
    dims = fuse_dims_over_cloud_mm(np.vstack([top, flank]))
    assert within_measurement_tolerance(dims, (300.0, 200.0, 100.0)), dims


def test_none_side_is_bit_identical_to_main():
    depth, side = _top_and_side("helmet", _dome_quat())
    base = measure_items(depth)
    assert base, "the rendered helmet frame must measure"
    for absent in (None, np.empty((0, 3))):
        got = measure_items(depth, side_points_world_m=absent)
        assert len(got) == len(base)
        for a, b in zip(base, got):
            assert a.dims_mm == b.dims_mm and a.k == b.k and a.position_m == b.position_m


def test_fusion_leaves_K_top_only():
    depth, side = _top_and_side("helmet", _dome_quat())
    top = measure_items(depth)[0]
    fused = measure_items(depth, side_points_world_m=side)[0]
    assert fused.k == top.k, "the side head must not change K"
    assert fused.dims_mm != top.dims_mm, "but it must change the dimensions"


def test_monotone_fusion_never_shrinks_below_top_only():
    """The core guarantee: a second view may only REVEAL extent, never remove it.

    Naive union let the min-volume box tilt on a soft/round cloud and UNDER-read a
    dimension the top view had right (bag height 148 vs 170, helmet 247 vs 282), which
    measured a NET REGRESSION on live frames (23/33 vs top-only 27/33). Monotone fusion
    takes the per-axis max against top-only, so every fused dim is >= its top-only value
    — recovering under-read dims (lying-body lower section) without the shrink regressions
    (measured 29/33, docs/experiments.md).
    """
    depth, side = _top_and_side("helmet", _dome_quat())
    top = np.sort(measure_items(depth)[0].dims_mm)[::-1]
    fused = np.sort(measure_items(depth, side_points_world_m=side)[0].dims_mm)[::-1]
    assert np.all(fused >= top - 1e-9), (top, fused)


def test_sparse_side_cloud_keeps_top_only():
    """A side cloud below SIDE_MIN_POINTS (the thin pen) corrupts the OBB; ignore it."""
    from src.perception import SIDE_MIN_POINTS

    depth, side = _top_and_side("helmet", _dome_quat())
    assert len(side) >= SIDE_MIN_POINTS, "helmet flank should be dense"
    top = measure_items(depth)[0]
    sparse = measure_items(depth, side_points_world_m=side[: SIDE_MIN_POINTS - 1])[0]
    assert sparse.dims_mm == top.dims_mm, "too-sparse side must fall back to top-only"
