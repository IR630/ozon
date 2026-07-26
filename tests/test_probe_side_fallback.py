# -*- coding: utf-8 -*-
"""Independent side-head detection: the ways it would lie quietly.

The gate this probe serves decides whether a rig can have redundancy at all, so
the probe's own failure modes are the risk. Five of them get a test by value:

  * the raster index silently pairing a component with the WRONG world points —
    everything downstream would still print plausible millimetres;
  * the border rule quietly missing, so the fallback "rescues" exactly the
    partial views the top head discards on purpose (`src/perception.py:348`);
  * the prism admitting bare belt, which would turn the belt itself into goods;
  * the two-opposing-flanks claim — the one mechanism by which a THIRD head can
    beat a second one under an independent detector — being assumed, not shown;
  * `k = 0.0` being sold as harmless when it structurally cannot produce a D.

The measurement primitives themselves are covered by tests/test_perception.py
and tests/test_multiview.py and are not re-tested here.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from probe_side_fallback import (  # noqa: E402
    CLASS_TO_B,
    CLASS_TO_C,
    SideDetection,
    find_side_items,
    fuse_flanks,
    item_prism_mask,
    side_dims_mm,
    side_world_points,
    verdict_class,
)

from src.constants import (  # noqa: E402
    CAMERA_SIDE_NEG_Y_POSE_M,
    CATEGORY_D,
    ROUND_K_THRESHOLD,
)
from src.multiview import camera_axes  # noqa: E402
from src.perception import BELT_TOP_Z_M, FX, IMG_W, _MIN_ITEM_PX  # noqa: E402

# Quarter-size frames with the intrinsics scaled to match: these tests assert on
# segmentation rules, not on the 640x480 geometry the probe runs at.
_H, _W = 120, 160
_FX = _FY = FX * _W / IMG_W


def _rect_scene_frame(pose, rects, h=_H, w=_W, fx=_FX, fy=_FY):
    """Depth frame a head returns for horizontal rectangles, nearest hit per ray.

    rects: [(z_m, (x0, x1), (y0, y1))]. Rays that miss everything get 0.0 — the
    byte-exact way this pipeline represents "no return" everywhere else.

    Horizontal faces only, on purpose: these tests exercise the prism's HEIGHT
    test and the component rules, not a renderer. A real side head also sees
    vertical flanks; the probe runs on live Gazebo frames that have them.
    """
    right, down, forward = camera_axes(pose)
    cam = np.asarray(pose[0], dtype=float)
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    dirs = (np.outer((us.ravel() - w / 2.0) / fx, right)
            + np.outer((vs.ravel() - h / 2.0) / fy, down)
            + forward)
    depth = np.zeros(h * w, dtype=float)
    for z0, (x0, x1), (y0, y1) in rects:
        with np.errstate(divide="ignore", invalid="ignore"):
            # depth is measured along `forward`, and a point at parameter t along
            # `dirs` sits at t * (dirs . forward) = t, since forward is a unit
            # vector and dirs has unit forward component by construction.
            t = (z0 - cam[2]) / dirs[:, 2]
        hit = cam + dirs * t[:, None]
        ok = (t > 0) & np.isfinite(t) & (hit[:, 0] >= x0) & (hit[:, 0] <= x1) \
            & (hit[:, 1] >= y0) & (hit[:, 1] <= y1)
        closer = ok & ((depth <= 0) | (t < depth))
        depth[closer] = t[closer]
    return depth.reshape(h, w)


def _box_cloud(size_mm, centre_xy_m=(1.5, 0.0), n=15):
    """Analytic surface cloud of a box resting on the belt (world metres)."""
    lx, ly, lz = (s / 1000.0 for s in size_mm)
    ax = np.linspace(-lx / 2, lx / 2, n) + centre_xy_m[0]
    ay = np.linspace(-ly / 2, ly / 2, n) + centre_xy_m[1]
    az = np.linspace(0.0, lz, n) + BELT_TOP_Z_M
    gx, gy, gz = np.meshgrid(ax, ay, az)
    return np.column_stack([gx.ravel(), gy.ravel(), gz.ravel()])


def test_the_world_cloud_pairs_with_its_raster_pixels():
    """The index that maps a raster component back to its points must be exact.

    `side_world_points` re-derives the valid-pixel set and TRUSTS that
    `world_cloud_from_depth` emitted its points in the same row-major order. If
    that ever drifts, every component would be measured from a neighbour's
    points and the report would still look entirely reasonable.
    """
    pose = CAMERA_SIDE_NEG_Y_POSE_M
    depth = _rect_scene_frame(pose, [(BELT_TOP_Z_M, (1.0, 2.0), (-0.25, 0.25))])
    pts, vs, us = side_world_points(depth, pose, _FX, _FY)

    assert len(pts) == len(vs) > 100
    _right, _down, forward = camera_axes(pose)
    cam = np.asarray(pose[0], dtype=float)
    # Depth is the component along `forward`, so this recovers the pixel's own
    # depth value from the world point and must match the frame exactly.
    along = (pts - cam) @ forward
    assert np.allclose(along, depth[vs, us], atol=1e-9)


def test_bare_belt_is_not_goods():
    """An empty belt must yield zero goods pixels, or the belt becomes the item.

    This is the property `SIDE_BELT_MARGIN_M` is derived for, checked from a side
    head's own viewpoint rather than assumed to transfer from the top view.
    """
    pose = CAMERA_SIDE_NEG_Y_POSE_M
    depth = _rect_scene_frame(pose, [(BELT_TOP_Z_M, (0.5, 2.5), (-0.25, 0.25))])
    mask, _pts, _index = item_prism_mask(depth, pose, _FX, _FY)
    assert mask.sum() == 0
    assert find_side_items(depth, pose, _FX, _FY) == []


def test_a_body_standing_on_the_belt_is_found_without_any_top_detection():
    """The whole point: a side head alone produces a detection.

    Nothing here consults a top measurement — no crop box, no position, no dims.
    That is exactly the contract `cameras.md` §8 T3 says the shipped node lacks.
    """
    pose = CAMERA_SIDE_NEG_Y_POSE_M
    depth = _rect_scene_frame(pose, [
        (BELT_TOP_Z_M, (0.5, 2.5), (-0.25, 0.25)),
        (BELT_TOP_Z_M + 0.08, (1.40, 1.60), (-0.10, 0.10)),
    ])
    found = find_side_items(depth, pose, _FX, _FY)
    assert len(found) == 1
    assert found[0].n_pixels >= _MIN_ITEM_PX
    # Height comes out of the cloud, so it is the dimension a side head owns.
    assert max(found[0].dims_mm) > 0.0
    assert min(found[0].dims_mm) == pytest.approx(80.0, abs=5.0)


def test_a_component_touching_the_frame_border_is_rejected():
    """An item riding into view must be discarded, exactly as the top head does.

    `src/perception.py:348` drops border-touching components because a partial
    view yields garbage dims. In a live run that is a likelier cause of "no top
    detection" than any dropout, so a fallback missing this rule would spend its
    whole budget publishing the rubbish the top head refused.
    """
    pose = CAMERA_SIDE_NEG_Y_POSE_M
    belt = (BELT_TOP_Z_M, (0.5, 2.5), (-0.25, 0.25))
    inside = _rect_scene_frame(pose, [belt, (BELT_TOP_Z_M + 0.08, (1.40, 1.60),
                                             (-0.10, 0.10))])
    assert len(find_side_items(inside, pose, _FX, _FY)) == 1

    # The same body, but long enough along the belt to run off both edges of the
    # frame — an item mid-passage rather than one standing fully in view.
    at_border = _rect_scene_frame(pose, [belt, (BELT_TOP_Z_M + 0.08, (0.0, 3.0),
                                                (-0.10, 0.10))])
    assert item_prism_mask(at_border, pose, _FX, _FY)[0].sum() > _MIN_ITEM_PX
    assert find_side_items(at_border, pose, _FX, _FY) == []


def test_specks_below_the_pixel_floor_are_not_items():
    """`_MIN_ITEM_PX` carries over unchanged — a few pixels are noise, not goods."""
    pose = CAMERA_SIDE_NEG_Y_POSE_M
    depth = _rect_scene_frame(pose, [
        (BELT_TOP_Z_M, (0.5, 2.5), (-0.25, 0.25)),
        (BELT_TOP_Z_M + 0.08, (1.48, 1.52), (-0.02, 0.02)),
    ])
    speck = item_prism_mask(depth, pose, _FX, _FY)[0].sum()
    assert 0 < speck < _MIN_ITEM_PX
    assert find_side_items(depth, pose, _FX, _FY) == []


def test_two_opposing_flanks_restore_the_across_belt_dimension():
    """The one thing a THIRD head buys that a second cannot, under own detection.

    A single side head sees ONE flank, so the footprint of a flat face collapses
    across the belt and the item measures far too narrow. Two opposing flanks
    span the true extent. Under the SHIPPED fusion this mechanism cannot appear
    at all — side points are only admitted inside the box the top head already
    measured — which is why the 2-vs-3 question has to be re-asked on the new
    architecture rather than inherited from the old numbers.
    """
    box = _box_cloud((200.0, 100.0, 80.0))
    # A flank is the outermost layer of the surface, not a mathematical plane:
    # one grid step thick, so the footprint has a hull instead of degenerating.
    step = 100.0 / 14 / 1000.0
    near = box[box[:, 1] <= box[:, 1].min() + step + 1e-9]   # what the -y head sees
    far = box[box[:, 1] >= box[:, 1].max() - step - 1e-9]    # what the +y head sees

    one_flank = sorted(side_dims_mm(near), reverse=True)
    both_flanks = fuse_flanks([SideDetection([], len(near), near),
                               SideDetection([], len(far), far)])
    # Across-belt extent of one flank is its own thickness, not the item's 100 mm.
    assert one_flank[2] == pytest.approx(1000.0 * step, abs=1.0)
    assert both_flanks == pytest.approx([200.0, 100.0, 80.0], abs=1.0)


def test_k_zero_cannot_produce_a_round_verdict_and_that_costs_the_bottle():
    """`k = 0.0` is safe against inventing D — and therefore cannot restore D.

    The plan argues the aggregator's MEDIAN K makes this free. It does, while
    fallback frames are a minority. Black film and specular wrap blind the top
    head for the WHOLE passage, and then every frame is a fallback frame: a
    genuinely round body routes to the main sorter with full confidence. This
    test states that cost rather than leaving it in prose.
    """
    assert ROUND_K_THRESHOLD > 0.0
    bottle_like = [303.0, 103.0, 103.0]
    assert verdict_class(bottle_like, 0.0, CATEGORY_D) == CLASS_TO_B
    # An item that is out of the sorter's size bounds still routes correctly:
    # the size decision owes nothing to K.
    assert verdict_class([600.0, 200.0, 200.0], 0.0, CATEGORY_D) == CLASS_TO_C


def test_the_prism_excludes_structure_beyond_the_belt_edge():
    """The opposite head's housing sits at |y| = 0.90 and must never be goods.

    It is photographed in every rig frame (docs/report/img/head_views_3cam.png)
    and is a named failure mode (`cameras.md` §4); with no top measurement to
    crop against, the belt edge is the only thing keeping it out.
    """
    pose = CAMERA_SIDE_NEG_Y_POSE_M
    # On the head's own side of the belt, at the height its central ray passes:
    # visible, 210 mm above the belt (inside the prism's ceiling), and excluded
    # only because it sits beyond |y| = BELT_HALF_WIDTH_M.
    structure = (BELT_TOP_Z_M + 0.21, (1.2, 1.8), (-0.50, -0.35))
    assert (_rect_scene_frame(pose, [structure]) > 0).sum() > _MIN_ITEM_PX

    depth = _rect_scene_frame(pose, [(BELT_TOP_Z_M, (0.5, 2.5), (-0.25, 0.25)),
                                     structure])
    assert item_prism_mask(depth, pose, _FX, _FY)[0].sum() == 0
