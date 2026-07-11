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

from src.constants import ROUND_K_THRESHOLD

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

# Vertical cross-section roundness (day 4, P2). A body of revolution lying on
# its side (Бутылка) shows a rectangular top-view silhouette, so silhouette K
# alone reads it as B — but its hidden end section is a circle (K=1 -> D). The
# top surface across the SHORT axis traces that section's upper arc; a circle
# fit to it recovers the section.
#
# A round section RESTING ON THE BELT is a circle tangent to it: centre height
# equals the radius, so tau = peak/R_fit ~ 2. That single fact separates it from
# a dome (Шлем), whose top-arc fits a circle that either floats high above the
# belt (tau >> 2) or is a shallow wide cap (tau << 2). A tie-rod Цилиндр end is
# tau ~ 2 too, but a rounded SQUARE, not a circle -> caught by the fit residual.
# So "round" = tau in a BAND around 2 AND low residual. Thresholds from top-down
# depth frames of the real settled STLs, 3 seeded poses each (docs/experiments.md
# 2026-07-11):
#   item       tau (poses)          rms/R        -> section verdict
#   bottle     2.13 2.19 2.26       <=0.009         circle on belt   -> D  (the fix)
#   cylinder   1.88 2.20 11.17      0.087..0.206    rounded square   -> B  (rms / tau)
#   helmet     1.11 1.38 2.98       0.01..0.02      dome             -> B  (tau band)
#   bag        2.13                 0.086           soft blob        -> B  (rms; also D via silhouette)
#   box/deterg/pouf/plate 0.44..0.98  --            flat/shallow     -> B  (tau band)
SECTION_TAU_LO = 1.85     # circle tangent to belt has tau ~ 2 (centre = radius);
SECTION_TAU_HI = 2.6      # below = shallow arc, above = dome cap floating high
SECTION_TAU_RAMP = 0.15   # trapezoid edge width -> full-strength plateau ~[2.0, 2.45]
SECTION_RMS_HI = 0.05     # rms/R above this is not a circle (rounded square / soft blob)
SECTION_RMS_SPAN = 0.03   # saturates to "circle" at rms/R <= HI - SPAN (0.02)

# Flatness gate on the silhouette->D route (day 4, P3). A round top-view
# silhouette means D (round in a section) only for a genuinely FLAT disc
# (Тарелка). A soft Мешок slumped on the belt also fills a round hull (silhouette
# K=0.88, solidity=1.0 — the solidity discriminator could NOT tell it from a
# plate, refuted on real Gazebo frames, docs/decisions.md 2026-07-12), but it is a
# THICK lump, not a disc: its height rivals its diameter. So when the silhouette
# CLAIMS roundness (K > ROUND_K_THRESHOLD) we additionally require the item to be
# flat; flatness = height/diameter read off the real settled STLs:
#   plate  dz/diam = 0.14  (flat disc)   -> keep silhouette K -> D
#   bag    dz/diam = 0.85  (thick lump)  -> drop silhouette K -> B
# The gate fires ONLY on a round-AND-thick silhouette (just the bag): a box's
# K=0.55 is below threshold and untouched, and the synthetic thick-puck test
# (dz/diam 0.53) stays below FLATNESS_MAX, so 0.7 leaves it round. The cross-
# section K route (Бутылка) is untouched, so a lying body of revolution reaches D.
FLATNESS_MAX = 0.7


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


def _silhouette_solidity(pts, hull):
    """Fraction of the top-view convex `hull` actually filled by mask pixels `pts`.

    _roundness_k measures the HULL, so a slumped soft item (Мешок) with an
    irregular outline is smoothed into a round hull -> high silhouette K though it
    is not round (measured 0.79..0.87, straddling the 0.8 -> D threshold against
    its reference 0.72 -> B). Solidity = mask_area / hull_area stays ~1 for a rigid
    item that fills its hull (a flat round Тарелка, legitimately D) but drops for a
    lumpy blob with a concave boundary -- the intended discriminator to keep the
    silhouette-K -> D route honest without touching K's threshold (which protects
    Шлем 0.78, see classify_conservative).

    NOT yet wired into measure_item: the cutoff must be read off a real Gazebo bag
    frame, never tuned on synthetic masks (docs/decisions.md, Karpathy #1). Reuses
    the `pts`/`hull` already built in measure_item.
    """
    hull_area = float(hull.volume)  # 2-D ConvexHull.volume is the enclosed area
    assert hull_area > 0.0, "degenerate hull"
    return float(min(len(pts) / hull_area, 1.0))


def _section_roundness_k(xs, ys, heights_m, long_dir, scale_m_per_px):
    """K of the item's cross-section perpendicular to its long axis, recovered
    from the top-view height map (heights_m: pixel heights above the belt, m).

    Reconstructs the section's upper arc by taking, per bin across the SHORT
    axis, the highest surface point (the ridge), then fits a circle (algebraic
    Kåsa fit) to that arc. Returns ~1 only when the arc is a genuine circle
    RESTING ON THE BELT — tau = peak/R in a band around 2 (tangent to the belt)
    AND low residual — else ~0, leaving the top-view silhouette K to decide. See
    the SECTION_* constants for why both gates are needed (Цилиндр lands in the
    tau band but its square section fails the residual; every Шлем dome pose
    falls outside the tau band — a dome is never tangent-circle-on-belt).
    """
    ux, uy = long_dir
    sx, sy = -uy, ux                          # short axis, perpendicular to long
    w_m = ((xs - xs.mean()) * sx + (ys - ys.mean()) * sy) * scale_m_per_px
    lo, hi = float(w_m.min()), float(w_m.max())
    if hi - lo <= 0.0:
        return 0.0
    nb = 24
    edges = np.linspace(lo, hi, nb + 1)
    idx = np.clip(np.digitize(w_m, edges) - 1, 0, nb - 1)
    wc, hc = [], []
    for b in range(nb):
        sel = idx == b
        if int(sel.sum()) < 3:                # a stable ridge needs a few pixels
            continue
        wc.append(0.5 * (edges[b] + edges[b + 1]))
        hc.append(float(heights_m[sel].max()))
    if len(wc) < 6:                           # too few bins to fit a circle
        return 0.0
    wc, hc = np.asarray(wc), np.asarray(hc)
    # Kåsa fit: solve w^2+h^2 + D*w + E*h + F = 0 in least squares
    a_mat = np.column_stack([wc, hc, np.ones_like(wc)])
    sol, *_ = np.linalg.lstsq(a_mat, -(wc ** 2 + hc ** 2), rcond=None)
    a_c, b_c = -sol[0] / 2.0, -sol[1] / 2.0
    disc = a_c ** 2 + b_c ** 2 - sol[2]
    if disc <= 0.0:
        return 0.0
    r_fit = float(np.sqrt(disc))
    if r_fit <= 0.0 or not np.isfinite(r_fit):
        return 0.0
    tau = float(hc.max()) / r_fit             # ~2 tangent circle, else dome/flat
    rms_rel = float(np.sqrt(np.mean((np.hypot(wc - a_c, hc - b_c) - r_fit) ** 2))) / r_fit
    # trapezoid on tau (0 outside [LO, HI], plateau in the middle) x circle-fit gate
    c_shape = np.clip(min((tau - SECTION_TAU_LO) / SECTION_TAU_RAMP,
                          (SECTION_TAU_HI - tau) / SECTION_TAU_RAMP), 0.0, 1.0)
    c_fit = np.clip((SECTION_RMS_HI - rms_rel) / SECTION_RMS_SPAN, 0.0, 1.0)
    return float(c_shape * c_fit)


def _min_area_rect_dims(poly):
    """Long/short side lengths (px) and the LONG side's unit direction of the
    minimum-area rectangle enclosing the convex polygon `poly` (rotating
    calipers: a side of the min-area rectangle is collinear with a hull edge).
    This is the item footprint independent of its yaw, unlike the axis-aligned
    pixel bbox, which inflates when the item is rotated on the belt
    (docs/experiments.md 2026-07-11). The long-side direction is the item's
    on-belt long axis, used by the vertical cross-section K below.
    """
    n = len(poly)
    best_area = None
    best = (0.0, 0.0)
    best_dir = (1.0, 0.0)
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
            best_dir = (ux, uy) if w >= h else (-uy, ux)  # unit vector of long side
    return max(best), min(best), best_dir


def _obb_dims_px(hull_vertices):
    """Long/short footprint (px) and long-axis direction of the mask via its
    oriented bounding box, from the convex-hull vertices (CCW) of the mask."""
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
    long_px, short_px, long_dir = _obb_dims_px(pts[hull.vertices])
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

    # K = max of the top-view silhouette and the vertical cross-section: a body
    # of revolution lying on its side is a rectangle from above (low silhouette
    # K) but a circle in its hidden end section (K=1). max, per the criterion
    # "max r_in/R_circ over sections along the principal axes" (docs/md/models.md).
    heights_m = belt_depth_m - depth_m[mask]
    k_silhouette = _roundness_k(pts, hull)
    # The top-view silhouette means D only for a FLAT disc. A thick round lump
    # (Мешок slumped on the belt) fills a round hull too (silhouette K=0.88,
    # solidity=1.0 — indistinguishable from Тарелка by silhouette shape), so once
    # the silhouette CLAIMS roundness, require the item to be flat; otherwise drop
    # the silhouette K and let the cross-section K decide (0 for a lump -> B). The
    # flat disc (dz/diam ~0.14) is untouched. See FLATNESS_MAX; provenance
    # docs/decisions.md 2026-07-12 (solidity refuted on the real bag frame).
    if k_silhouette > ROUND_K_THRESHOLD and dz_mm > FLATNESS_MAX * max(dx_mm, dy_mm):
        k_silhouette = 0.0
    k_section = _section_roundness_k(xs, ys, heights_m, long_dir, top_depth_m / fx)

    return Measurement(
        dims_mm=dims,
        k=max(k_silhouette, k_section),
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
