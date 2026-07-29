# -*- coding: utf-8 -*-
"""Multi-head fusion: side depth frames refine the dims of a top-view detection.

WHAT THIS DOES AND DOES NOT DO. The top head keeps driving everything — it finds
the items, it owns the world position and the item id, and it alone decides K
(a side head sees the end circle of anything lying down and would report every
prone body as round). The side heads contribute POINTS ONLY, and only inside the
volume the top head already claimed for an item. That keeps `ItemMeasurement`
byte-identical and keeps one detector in the system rather than three.

Segmentation is the reason for that shape. `_find_item` separates an item from
the belt by depth against a known belt plane; a side head has no such background,
so segmenting its frame on its own terms is a second, different detector. Anchor-
ing on the top head's world box avoids inventing one.

THE SYNC PENALTY IS REAL AND IS COMPENSATED HERE. The heads are untriggered at
15 Hz, so two frames can be a full period apart — 66.7 ms, and at a belt speed of
1 m/s that is 66.7 mm of travel against a 5 mm accuracy budget. Every side cloud
is therefore shifted along +x by (t_top - t_side) * BELT_SPEED_M_S before it is
allowed near the dims. Uncompensated multi-head numbers are not conservative,
they are meaningless.
"""
import numpy as np

from src.constants import (
    BELT_SPEED_M_S,
    MAX_DIMS_MM,
    MIN_DIMS_MM,
    SIDE_BELT_MARGIN_M,
    SIDE_HEIGHT_MIN_GAIN_MM,
)
from src.perception import BELT_TOP_Z_M

# How far outside the top head's own box a side head's points may still belong to
# the same item. The top view measures a SHADOW, and the body under it can only be
# narrower, never wider — but registration error, the belt-travel residual after
# compensation, and the item's own hidden flanks all push points outward. Half the
# sorter's largest admissible dimension is a generous bound that still excludes a
# neighbouring product on the belt (items are fed sequentially, metres apart).
_CROP_MARGIN_M = MAX_DIMS_MM[0] / 2000.0

# Points this close to the belt are belt, not item. NOT the top view's margin:
# a grazing head lifts the belt plane under a pointing error where a downward
# head only slides it sideways. Derivation and the census that measured it live
# with the constant.
_BELT_MARGIN_M = SIDE_BELT_MARGIN_M


def camera_axes(pose):
    """(right, down, forward) unit vectors of a head given ((x,y,z), (lx,ly,lz))."""
    pos, look_at = np.asarray(pose[0], float), np.asarray(pose[1], float)
    forward = look_at - pos
    forward /= np.linalg.norm(forward)
    # world +z is up; for a straight-down head that is degenerate, so fall back to
    # +x, which is the belt direction and never parallel to a downward view.
    up = np.array([0.0, 0.0, 1.0])
    if abs(float(forward @ up)) > 0.999:
        up = np.array([1.0, 0.0, 0.0])
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    return right, down, forward


def world_cloud_from_depth(depth_m, pose, fx, fy, cx=None, cy=None, max_range_m=5.0):
    """Backproject one head's depth frame to world metres (N x 3).

    Zeros and non-finite returns are dropped rather than projected to the camera
    origin, where they would form a phantom blob at the lens.
    """
    depth_m = np.asarray(depth_m, dtype=np.float64)
    h, w = depth_m.shape
    if cx is None:
        cx = w / 2.0
    if cy is None:
        cy = h / 2.0
    vs, us = np.nonzero(np.isfinite(depth_m) & (depth_m > 0.0) & (depth_m < max_range_m))
    if not len(us):
        return np.empty((0, 3))
    d = depth_m[vs, us]
    right, down, forward = camera_axes(pose)
    xc = (us - cx) * d / fx
    yc = (vs - cy) * d / fy
    # Per-axis, not three np.outer products. The outer form allocated four N x 3
    # temporaries (~19 MB per head on a 196k-point frame) and summed them; a rig at
    # 15 Hz pays that per side head per frame. Three heads came to 88.6 ms against a
    # 66.7 ms period — the node fell behind and the verdict for a moving item arrived
    # after the diverter (bottle oi=1 rode to the end of the belt in 2 of 3 draws).
    # Same arithmetic per component, in the same order, so the result is unchanged;
    # `tests/test_multiview.py` pins that against the previous formulation.
    origin = np.asarray(pose[0], float)
    out = np.empty((d.size, 3))
    for axis in range(3):
        out[:, axis] = origin[axis] + xc * right[axis] + yc * down[axis] + d * forward[axis]
    return out


def compensate_belt_travel(pts_m, dt_s, belt_speed_m_s=BELT_SPEED_M_S):
    """Shift a head's cloud along the belt by the travel since the reference frame.

    dt_s = t_reference - t_head. A head that fired EARLIER than the reference saw
    the item further upstream, so its points move forward (+x) to meet it.
    """
    if not len(pts_m) or dt_s == 0.0:
        return pts_m
    out = np.array(pts_m, dtype=float, copy=True)
    out[:, 0] += dt_s * belt_speed_m_s
    return out


def crop_to_item(pts_m, position_m, dims_mm, margin_m=_CROP_MARGIN_M):
    """Keep only the points that can plausibly belong to the top head's detection."""
    if not len(pts_m):
        return pts_m
    reach = max(dims_mm) / 2000.0 + margin_m
    px, py, _pz = position_m
    keep = ((np.abs(pts_m[:, 0] - px) <= reach)
            & (np.abs(pts_m[:, 1] - py) <= reach)
            & (pts_m[:, 2] >= BELT_TOP_Z_M + _BELT_MARGIN_M))
    return pts_m[keep]


# Fewest side points that make the height percentile trustworthy. Below this a
# grazing head has seen only a sliver of a flank (a thin item edge-on, or a nearly
# occluded one) and its z-spread is noise; the rig then degrades to the top view for
# that frame rather than reading a dimension off a handful of pixels.
SIDE_MIN_POINTS = 30
# The item's height is the robust RISE of the side cloud above the belt. 99.5 rather
# than the raw max so one stray high return does not set the dimension — the same
# principle as the top footprint's trim.
SIDE_HEIGHT_PCTL = 99.5


def fuse_dims_mm(top_dims_mm, side_clouds_m, position_m):
    """Dims (mm, desc) after ARBITRATION with the side heads, or the top dims.

    NOT a cloud-merge: merging the top and side clouds into one min-volume OBB tilted
    on soft and round bodies and regressed organizer tolerance 27 -> 23 (the naive
    union, docs/decisions.md 2026-07-21). The heads have ASYMMETRIC roles, so each
    measures what it sees best. The top head owns the FOOTPRINT — a grazing side head
    sees only the near flank through occlusion and under-reads the two lateral dims, so
    it must not touch them — and it owns K. The side heads contribute ONE thing: the
    item's HEIGHT, the hidden vertical extent a top-down view under-reads (the helmet
    dome, a lying body's lower half-shell). Height is taken as max(top, side), so a head
    that sees no new height, is lost, or returns too few points degrades BIT-EXACT to
    the top measurement — a rig sorts on the heads it still has. This is the structural
    estimator that measured 30/33 (docs/experiments.md 2026-07-21), rebuilt on the
    robust top footprint and generalised to any number of side clouds.

    THE OVERRIDE IS NOT FREE AND IS GATED. A plain max() over a noisy estimator can
    only inflate, so a side head that merely NOISES upward would keep winning. The
    gain must therefore clear SIDE_HEIGHT_MIN_GAIN_MM — the head's own vertical
    resolution — or the reading is not evidence of hidden height. Without that gate
    the rig lifts the 9 mm Pen to 11 mm, out from under MIN_DIMS_MM, and the size
    rule that made its verdict correct stops applying (derivation and the measured
    cell are with the constant).
    """
    clouds = [c for c in side_clouds_m if c is not None and len(c) >= SIDE_MIN_POINTS]
    if not clouds:
        return list(top_dims_mm)
    z_above_m = np.concatenate([c[:, 2] for c in clouds]) - BELT_TOP_Z_M
    side_height_mm = float(np.percentile(z_above_m, SIDE_HEIGHT_PCTL)) * 1000.0
    # The top head's own height, recovered from where measure_item centred the item:
    # world_z = BELT_TOP_Z_M + dz_mm / 2, so dz_mm = (world_z - belt) * 2.
    top_height_mm = (float(position_m[2]) - BELT_TOP_Z_M) * 2000.0
    if side_height_mm - top_height_mm < SIDE_HEIGHT_MIN_GAIN_MM:
        return list(top_dims_mm)              # no gain above the head's resolution: degrade
    # BOUNDARY POLICY, and the resolution gate above is NOT enough on its own. The
    # gate scales with the head's optics; the noise does not. At sigma 3 mm the
    # 99.5th percentile of a 9 mm flank can clear 5 mm of gain by itself, so a
    # larger sigma re-opens exactly the Pen defect the gate was added to close
    # (found by the sweep in tests/test_multiview.py, not reasoned about after).
    #
    # So the size boundary gets an explicit refusal: a dimension the TOP head
    # placed under the small-item floor is never carried over that floor by a side
    # head. This is the same conservative stance classification.classify_conservative
    # already takes at the sorter's size bounds, applied one stage earlier, and it
    # is directional on purpose — crossing the floor DOWNWARD is impossible here
    # (height only rises), and crossing it upward hands the verdict from the size
    # rule to the shape rule, which is how the Pen became D.
    #
    # THE REFUSAL IS BOUNDED BY THE SAME RESOLUTION ARGUMENT, not by what happens to
    # be in the catalogue. It applies only while the side reading is itself within one
    # resolution band ABOVE the floor — that is, exactly where "over the threshold" and
    # "under the threshold" are indistinguishable to this head. A side head that reports
    # a height far above the floor is not making a marginal call and is allowed through:
    # if the top view under-read a 300 mm body to 4 mm, a 296 mm gain is overwhelming
    # evidence, and vetoing it would turn a segmentation failure into a misroute.
    #
    # An unbounded veto (refuse whenever top < floor <= side) was written first and is
    # wrong for a reason this project already paid for: it would be justified only by
    # "our catalogue has no body between 10 and 15 mm", which is the catalogue-luck
    # reasoning that disqualified the perhead-median fusion rule on 21.07 (docs/
    # decisions.md: "это везение каталога, не свойство метода"). Same yardstick applies
    # to our own fix.
    floor_mm = min(MIN_DIMS_MM)
    if top_height_mm < floor_mm <= side_height_mm < floor_mm + SIDE_HEIGHT_MIN_GAIN_MM:
        return list(top_dims_mm)
    # Raise the height component (the dim the top set closest to its own height) to the
    # taller reading. Increasing one element and re-sorting only lifts each order
    # statistic, so the result never carves a dimension the top saw directly.
    dims = list(top_dims_mm)
    i = int(np.argmin([abs(d - top_height_mm) for d in dims]))
    dims[i] = side_height_mm
    return sorted(dims, reverse=True)
