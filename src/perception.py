# -*- coding: utf-8 -*-
"""Perception: dims (mm), roundness K, world position from the top-down depth frame.

The core is one pure numpy+scipy function measure_item(); depth-PNG loading and
the cv2 overlay live around it so the geometry is testable without ROS or OpenCV.

Scene calibration mirrors sim/worlds/cell.sdf (that world file is the single source):
camera looks straight down from (1.5, 0, 1.9) m, belt top at z=0.4 m, RGBD 640x480,
horizontal_fov 1.05 rad. At runtime the node should prefer /camera/camera_info;
these constants are the offline calibration for the saved frames. If the world
changes, update here.

Pixel -> world mapping (verified empirically on a frame with the box offset to
world (1.8, 0.1): recovered (1.791, 0.100), docs/experiments.md 2026-07-11):
    world_x = CAMERA_X_M - (v - cy) * depth / fy      # +x is UP in the image
    world_y = CAMERA_Y_M - (u - cx) * depth / fx      # +y is LEFT in the image
"""
from dataclasses import dataclass

import numpy as np

CAMERA_X_M = 1.5      # cell.sdf: camera model pose x
CAMERA_Y_M = 0.0      # cell.sdf: camera model pose y
CAMERA_Z_M = 1.9      # cell.sdf: camera model pose z
BELT_TOP_Z_M = 0.4    # cell.sdf: belt top surface (platform top)
IMG_W, IMG_H = 640, 480
HFOV_RAD = 1.05       # cell.sdf: camera horizontal_fov

# depth of the empty belt surface seen from the camera
BELT_DEPTH_M = CAMERA_Z_M - BELT_TOP_Z_M
# square pixels: fy == fx (verified against the real frame, ±3 mm on Короб 300)
FX = FY = (IMG_W / 2.0) / np.tan(HFOV_RAD / 2.0)

_MIN_ITEM_PX = 200    # ignore specks; a real item covers thousands of pixels
# Mask margin above the belt plane: must clear the thinnest item (Ручка, 9 mm)
# while staying above the empty-belt depth spread (<=1 mm on the saved real
# frames — sim depth is clean; 20 mm silently swallowed items under 20 mm).
MASK_MARGIN_M = 0.005


@dataclass
class Measurement:
    """One item measured on one frame. Contract mirror of ItemMeasurement.msg."""

    dims_mm: list        # three dims, sorted descending, millimeters
    k: float             # r_inscribed / R_circumscribed of the top-view hull, [0..1]
    position_m: tuple    # (x, y, z) of the item center, meters, world frame
    bbox_px: tuple       # (x0, y0, x1, y1) pixel bbox, for overlays/debug


def _item_mask(depth_m, belt_depth_m, margin_m):
    """Boolean mask of pixels standing above the belt by more than margin."""
    return (depth_m > 0) & (depth_m < belt_depth_m - margin_m)


def _find_item(depth_m, belt_depth_m, margin_m):
    """(mask, bbox) of the single item, or None.

    None when the belt is empty OR the item touches the frame border — a
    partially visible item (riding into/out of view) yields garbage dims, so
    the caller must wait for the next frame (camera runs at 15 Hz).
    """
    mask = _item_mask(depth_m, belt_depth_m, margin_m)
    ys, xs = np.where(mask)
    if xs.size < _MIN_ITEM_PX:
        return None
    h, w = depth_m.shape
    x0, y0, x1, y1 = int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())
    if x0 == 0 or y0 == 0 or x1 == w - 1 or y1 == h - 1:
        return None
    return mask, (x0, y0, x1, y1)


def _min_enclosing_radius(pts):
    """Radius of the minimal enclosing circle (incremental Welzl, deterministic).

    pts: (n, 2) float array — convex hull vertices, so n is small (tens).
    """
    def dist(a, b):
        return float(np.hypot(a[0] - b[0], a[1] - b[1]))

    def circle2(a, b):
        return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0), dist(a, b) / 2.0

    def circle3(a, b, c):
        # circumcircle via the perpendicular-bisector determinant formula
        d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]))
        if abs(d) < 1e-12:
            return None  # collinear
        ux = ((a[0] ** 2 + a[1] ** 2) * (b[1] - c[1]) + (b[0] ** 2 + b[1] ** 2) * (c[1] - a[1])
              + (c[0] ** 2 + c[1] ** 2) * (a[1] - b[1])) / d
        uy = ((a[0] ** 2 + a[1] ** 2) * (c[0] - b[0]) + (b[0] ** 2 + b[1] ** 2) * (a[0] - c[0])
              + (c[0] ** 2 + c[1] ** 2) * (b[0] - a[0])) / d
        center = (ux, uy)
        return center, dist(center, a)

    def contains(circle, p):
        (cx, cy), r = circle
        return np.hypot(p[0] - cx, p[1] - cy) <= r * (1 + 1e-9) + 1e-9

    pts = [tuple(map(float, p)) for p in pts]
    circle = (pts[0], 0.0)
    for i, p in enumerate(pts):
        if contains(circle, p):
            continue
        circle = (p, 0.0)
        for j, q in enumerate(pts[:i]):
            if contains(circle, q):
                continue
            circle = circle2(p, q)
            for s in pts[:j]:
                if not contains(circle, s):
                    c3 = circle3(p, q, s)
                    if c3 is not None:
                        circle = c3
    return circle[1]


def _roundness_k(pts, hull):
    """K = r_inscribed / R_circumscribed of the convex `hull` of mask pixels `pts`.

    Same definition as the task criterion and the docs/md/models.md analysis:
    the hull (not the raw outline), so a cylinder with tie-wraps reads as a
    rounded square, not a circle. Dimensionless: pixel scale cancels out.
    """
    from scipy.optimize import linprog

    # Chebyshev center: maximize r s.t. A@c + r <= -b (hull normals are unit)
    a_ub = np.hstack([hull.equations[:, :2], np.ones((len(hull.equations), 1))])
    b_ub = -hull.equations[:, 2]
    res = linprog(c=[0.0, 0.0, -1.0], A_ub=a_ub, b_ub=b_ub,
                  bounds=[(None, None), (None, None), (0.0, None)], method="highs")
    r_in = float(res.x[2])
    r_circ = _min_enclosing_radius(pts[hull.vertices])
    assert r_circ > 0.0, "degenerate hull"
    return float(min(r_in / r_circ, 1.0))


def _min_area_rect_dims(poly):
    """Long/short side lengths (px) of the minimum-area rectangle enclosing the
    convex polygon `poly` (rotating calipers: a side of the min-area rectangle
    is collinear with a hull edge). This is the item footprint independent of
    its yaw, unlike the axis-aligned pixel bbox, which inflates when the item is
    rotated on the belt (docs/experiments.md 2026-07-11).
    """
    n = len(poly)
    best_area = None
    best = (0.0, 0.0)
    for i in range(n):
        edge = poly[(i + 1) % n] - poly[i]
        length = float(np.hypot(edge[0], edge[1]))
        if length < 1e-9:
            continue
        ux, uy = edge / length
        proj = poly @ np.array([ux, uy])    # extent along the edge
        perp = poly @ np.array([-uy, ux])   # extent across the edge
        w = float(proj.max() - proj.min())
        h = float(perp.max() - perp.min())
        area = w * h
        if best_area is None or area < best_area:
            best_area = area
            best = (w, h)
    return max(best), min(best)


def _obb_dims_px(hull_vertices):
    """Long/short footprint (px) of the mask via its oriented bounding box,
    from the convex-hull vertices (CCW) of the mask pixels."""
    return _min_area_rect_dims(hull_vertices)


def measure_item(depth_m, belt_depth_m=BELT_DEPTH_M, fx=FX, fy=FY, margin_m=MASK_MARGIN_M,
                 camera_x_m=CAMERA_X_M, camera_y_m=CAMERA_Y_M):
    """Measurement of the single item on the belt, or None (empty / partial view).

    depth_m: HxW depth image in meters (0 = no return). Two lateral dims come
    from the oriented bounding box (yaw-invariant footprint) at the item's
    top-face depth; height from how far the top rises above the belt. K from the
    top-view hull; position from the mask centroid via the verified pixel->world
    mapping.
    """
    found = _find_item(depth_m, belt_depth_m, margin_m)
    if found is None:
        return None
    mask, (x0, y0, x1, y1) = found
    ys, xs = np.where(mask)

    top_depth_m = float(np.median(depth_m[mask]))
    # One convex hull of the mask pixels, shared by the OBB footprint and the
    # roundness K below — both need it, computing it twice per frame was waste.
    from scipy.spatial import ConvexHull

    pts = np.column_stack([xs, ys]).astype(float)
    hull = ConvexHull(pts)
    # oriented bbox, not the axis-aligned pixel bbox: a box rotated in yaw on the
    # belt inflates the axis-aligned extent (measured 341x322 for a 300x200 box,
    # docs/experiments.md) and can cross the sorter limit. +1 px matches the
    # inclusive-pixel convention (a lone axis-aligned box reads the same as before).
    long_px, short_px = _obb_dims_px(pts[hull.vertices])
    dx_mm = (long_px + 1.0) * top_depth_m / fx * 1000.0
    dy_mm = (short_px + 1.0) * top_depth_m / fy * 1000.0
    # height = the item's HIGHEST point (bounding box), not the mask median:
    # concave items (Тарелка — rim 27 mm, dish bottom 8 mm) would otherwise
    # read below the 10 mm min-dim threshold and flip category to C.
    # 1st percentile, not min: robust to stray depth returns.
    dz_mm = (belt_depth_m - float(np.percentile(depth_m[mask], 1.0))) * 1000.0
    dims = sorted([dx_mm, dy_mm, dz_mm], reverse=True)

    cx, cy = depth_m.shape[1] / 2.0, depth_m.shape[0] / 2.0
    world_x = camera_x_m - (float(ys.mean()) - cy) * top_depth_m / fy
    world_y = camera_y_m - (float(xs.mean()) - cx) * top_depth_m / fx
    world_z = BELT_TOP_Z_M + dz_mm / 1000.0 / 2.0

    return Measurement(
        dims_mm=dims,
        k=_roundness_k(pts, hull),
        position_m=(world_x, world_y, world_z),
        bbox_px=(x0, y0, x1, y1),
    )


def measure_dims_mm(depth_m, belt_depth_m=BELT_DEPTH_M, fx=FX, fy=FY, margin_m=MASK_MARGIN_M):
    """Three dimensions (mm, sorted descending) of the item, or None."""
    m = measure_item(depth_m, belt_depth_m, fx=fx, fy=fy, margin_m=margin_m)
    return None if m is None else m.dims_mm


def item_bbox_px(depth_m, belt_depth_m=BELT_DEPTH_M, margin_m=MASK_MARGIN_M):
    """(x0, y0, x1, y1) pixel bounding box of the item, or None."""
    m = measure_item(depth_m, belt_depth_m, margin_m=margin_m)
    return None if m is None else m.bbox_px


def load_depth_png(path):
    """Load a uint16-millimeter depth PNG (as dumped by scripts/dump_camera.py) to meters."""
    import cv2

    # cv2.imread() cannot open non-ASCII paths on some Windows builds.  Read
    # the encoded bytes with numpy (which uses Python's Unicode-aware file
    # APIs) and let OpenCV decode the in-memory buffer instead.
    encoded = np.fromfile(path, dtype=np.uint8)
    mm = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if mm is None:
        raise FileNotFoundError(path)
    return mm.astype(np.float64) / 1000.0


def save_overlay(depth_m, out_path, belt_depth_m=BELT_DEPTH_M):
    """Draw the measured bbox, dims and K over the depth frame (Karpathy #4)."""
    import cv2

    m = measure_item(depth_m, belt_depth_m)
    gray = np.clip((depth_m - 1.2) / (2.0 - 1.2), 0, 1)  # belt/floor band -> readable
    vis = cv2.cvtColor((gray * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    if m is not None:
        x0, y0, x1, y1 = m.bbox_px
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 2)
        label = " x ".join(f"{d:.0f}" for d in m.dims_mm) + f" mm  K={m.k:.2f}"
        cv2.putText(vis, label, (x0, max(y0 - 8, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.imwrite(str(out_path), vis)
    return None if m is None else m.dims_mm
