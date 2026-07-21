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

from src.constants import BELT_SPEED_M_S, MAX_DIMS_MM
from src.perception import BELT_TOP_Z_M, _body_obb_dims_mm, _obb_dims_px

# How far outside the top head's own box a side head's points may still belong to
# the same item. The top view measures a SHADOW, and the body under it can only be
# narrower, never wider — but registration error, the belt-travel residual after
# compensation, and the item's own hidden flanks all push points outward. Half the
# sorter's largest admissible dimension is a generous bound that still excludes a
# neighbouring product on the belt (items are fed sequentially, metres apart).
_CROP_MARGIN_M = MAX_DIMS_MM[0] / 2000.0

# Points this close to the belt are belt, not item — the same margin the top-view
# segmentation uses to avoid swallowing a 9 mm pen into the plane.
_BELT_MARGIN_M = 0.005


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
    return np.asarray(pose[0], float) + np.outer(xc, right) + np.outer(yc, down) \
        + np.outer(d, forward)


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


def fuse_dims_mm(top_dims_mm, side_clouds_m, position_m):
    """Dims (mm, desc) after admitting the side heads' points, or the top dims.

    The fused cloud is resolved exactly the way production resolves a single view:
    the belt-aligned shadow box is the candidate to beat and the tilted body-OBB
    replaces it only when its volume is genuinely smaller. Returning the top dims
    unchanged is the correct DEGRADED answer, not an error — a rig that loses a
    head keeps sorting on the head it still has.
    """
    clouds = [c for c in side_clouds_m if c is not None and len(c) >= 4]
    if not clouds:
        return list(top_dims_mm)
    pts = np.vstack(clouds)
    if len(pts) < 4:
        return list(top_dims_mm)

    from scipy.spatial import ConvexHull, QhullError

    heights_m = pts[:, 2] - BELT_TOP_Z_M
    dz_mm = max(float(heights_m.max()) * 1000.0, float(min(top_dims_mm)))
    try:
        hull = ConvexHull(pts[:, :2] * 1000.0)
    except QhullError:
        return list(top_dims_mm)          # degenerate side view: trust the top head
    long_mm, short_mm, _dir = _obb_dims_px((pts[:, :2] * 1000.0)[hull.vertices])
    shadow = sorted([float(long_mm), float(short_mm), dz_mm], reverse=True)
    body = _body_obb_dims_mm(
        xs=pts[:, 0], ys=pts[:, 1], depth_col_m=np.ones(len(pts)),
        heights_m=heights_m, fx=1.0, fy=1.0, cx=0.0, cy=0.0,
        legacy_dims_mm=tuple(shadow), dz_mm=dz_mm, px_pad_mm=0.0)
    fused = shadow if body is None else body
    # The top head saw the item unoccluded from above; extra views may only ADD
    # extent that was hidden, never carve away what was directly observed. Without
    # this floor a partially-visible flank shrinks a correct measurement.
    return sorted([max(a, b) for a, b in zip(fused, sorted(top_dims_mm, reverse=True))],
                  reverse=True)
