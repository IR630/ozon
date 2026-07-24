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

from src.constants import ROUND_K_THRESHOLD, SANE_DIM_MM_MAX, SANE_DIM_MM_MIN

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

# Reject specks, but the thinnest item in the task is only a few pixels wide: at
# 1.5 m the camera resolves 2.72 mm/px, so Ручка (148x13x9 mm) covers just
# 155..179 px lying on the belt — the old 200 px gate silently dropped it on EVERY
# pose (matrix pen 0/3 "no measurement" -> default B instead of C, day 4). The
# floor this gate must clear is the empty belt, and that is measured at exactly
# 0 px (the 5 mm margin below already suppresses the ~1 mm belt depth spread), so
# 24 px sits an order of magnitude above the noise and 6x below the pen.
# A pen standing on its end (15 px, tallest 149 mm) stays below detection —
# documented limit, not a fix target: on the moving belt it topples flat.
_MIN_ITEM_PX = 24
# Mask margin above the belt plane: must clear the thinnest item (Ручка, 9 mm)
# while staying above the empty-belt depth spread (<=1 mm on the saved real
# frames — sim depth is clean; 20 mm silently swallowed items under 20 mm).
MASK_MARGIN_M = 0.005

# EDT prominence (px) a second distance-transform peak must clear to count as a
# separate touching item rather than a bump on one item's medial-axis ridge. A
# rotated rectangle rasterizes into a ridge of many near-equal EDT maxima, so
# plain peak COUNTING over-splits a single item at every non-axis-aligned yaw;
# the prominence (h-maxima) test keeps only maxima that rise more than this above
# the highest saddle joining them to a taller peak. Rasterization bumps have
# prominence <1 px (merged); two touching items are separated by a saddle that
# dips to the neck half-width, a drop far larger than this. Stable for h in
# [2, 8] px on synthetic singles (every yaw -> 1) and touching pairs (-> 2);
# 3 px ~ 8 mm at the real camera, above jitter and below any item's peak.
_TOUCH_PEAK_PROMINENCE_PX = 3.0

# Solidity (mask area / convex-hull area) above which a component is taken to be
# a single convex item and the costly h-maxima split is skipped. Only the neck
# between two touching bodies pulls solidity down: every single rectangle (any
# yaw) fills its hull (solidity 1.0), while a necked pair drops well below (corner
# touch 0.71, partial-edge 0.78..0.92). The gate is a pure fast path — a low
# solidity only licenses a split attempt; peak count and compactness below can
# still keep an irregular single item whole. The fast path just keeps the common
# convex-item frame off the reconstruction path. Validated on the real single-item
# frames (they stay one component) — docs/experiments.md.
_TOUCH_MIN_SOLIDITY = 0.97

# Normalized inradius evidence for two full bodies rather than a thin articulated
# silhouette: max EDT radius / short side of the component's oriented box. Across
# 24 unequal/offset rectangle contacts the measured range is 0.367..0.495; the
# known Box400 false splits are 0.109..0.116 over eight yaws. The midpoint leaves
# a wide rasterization margin while preserving all measured two-product contacts.
_TOUCH_MIN_COMPACTNESS = 0.20

# A two-peak silhouette is not by itself proof of two products.  Long relief
# bodies such as Cylinder have two thick end lobes joined by a thinner waist;
# depending on the exact belt pose, h-maxima sees those lobes as two products
# and the tracker publishes phantom IDs.  The real failing stream frames measure
# 8.58..9.06 long/short, while every supported synthetic two-box contact is
# 1.30..2.01.  Only split reasonably compact components; an elongated contact
# stays one conservative measurement.  This intentionally gives up splitting
# unsupported end-to-end contacts between two long thin products rather than
# inventing commands for one real product.
_TOUCH_MAX_SPLIT_ASPECT = 4.0

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
# Elongation gate on the cross-section->D route (day 11, P3). The section route
# recovers the hidden end-circle of a body of revolution LYING on its side, which
# is by definition elongated: its footprint long side (the body) far exceeds its
# short side (the circle diameter) — Бутылка measures 296x90 px-equiv, long/short
# ~3.3. A soft Мешок slumped into a near-cuboidal footprint (long/short ~1.2) has
# no such hidden long axis, yet in the seeded oi=1 belt-ride pose its top ridge
# fits a belt-tangent circle almost as tightly as the bottle (rms/R 0.015 vs
# 0.009, tau ~2.05) and the section route false-positived it to D (bag oi=1 K=1.0,
# census #3 misroute). Require the footprint to be elongated for the section
# circle to count. The bottle clears this with 1.6x margin in every pose; a blob
# never does; the tie-rod Цилиндр (also elongated) is still rejected by the rms
# gate, so this only removes a false D, never adds one — no B item can regress.
# Threshold ~ geometric mean of bag 1.27 and bottle 3.29 (real frames, decisions.md).
SECTION_MIN_ELONGATION = 2.0

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

# Below this, the visible height map is a function of RADIUS ALONE — the item is a
# body of revolution standing on the belt (ball, upright can), genuinely round, not
# a lump whose convex hull merely smooths into a circle. Measured
# (docs/probe-models.md): upright can 0.0002, ball 0.0236 | real Мешок frame 0.0702.
# 0.04 sits between, ~1.7x above the ball and ~1.8x below the bag. Only silhouettes
# that ALREADY read round (K > 0.8) and thick ever reach this test, so Шлем (K=0.74)
# never does — the helmet dome is MORE axially symmetric (0.0551) than the bag and
# would otherwise be pulled into D against its reference B.
AXIAL_SYMMETRY_MAX = 0.04


@dataclass
class Measurement:
    """One item measured on one frame. Contract mirror of ItemMeasurement.msg."""

    dims_mm: list        # three dims, sorted descending, millimeters
    k: float             # r_inscribed / R_circumscribed of the top-view hull, [0..1]
    position_m: tuple    # (x, y, z) of the item center, meters, world frame
    bbox_px: tuple       # (x0, y0, x1, y1) pixel bbox, for overlays/debug


def _dims_are_sane(dims_mm):
    """Whether three measured dimensions are finite and physically plausible."""
    return all(
        np.isfinite(dim) and SANE_DIM_MM_MIN <= dim <= SANE_DIM_MM_MAX
        for dim in dims_mm
    )


def _item_mask(depth_m, belt_depth_m, margin_m):
    """Boolean mask of pixels standing above the belt by more than margin."""
    return (depth_m > 0) & (depth_m < belt_depth_m - margin_m)


def _find_item(depth_m, belt_depth_m, margin_m):
    """(mask, bbox) of the LARGEST item on the belt, or None.

    None when the belt is empty OR every item is only partially visible (an item
    touching the frame border is riding into/out of view and yields garbage
    dims, so the caller waits for the next frame — camera runs at 15 Hz).

    Largest COMPONENT, not the whole above-belt mask: the mask version fused two
    items in frame into one convex hull and reported the dims of their union, so
    every single-item caller (measure_dims_mm, item_bbox_px, save_overlay's
    debug overlay, scripts/dump_camera.py) quietly described one item that does
    not exist. Callers that must see all items use measure_items().
    """
    found = _find_items(depth_m, belt_depth_m, margin_m)
    if not found:
        return None
    return max(found, key=lambda mask_bbox: int(mask_bbox[0].sum()))


def _reconstruct_by_dilation(marker, image):
    """Grayscale morphological reconstruction of `image` from `marker` (<= image).

    Iterated geodesic dilation to stability. Operates on the small cropped
    sub-image, so the pixel-at-a-time propagation converges in a few ms. scipy
    has no reconstruction primitive; this is the standard fixed-point loop.
    """
    from scipy.ndimage import grey_dilation

    prev = marker
    while True:
        cur = np.minimum(grey_dilation(prev, size=3), image)
        if np.array_equal(cur, prev):
            return cur
        prev = cur


def _split_touching(mask):
    """Split a connected mask of two touching items into one mask per item.

    Touching products share one connected component, but their silhouette has a
    neck (concavity) between the bodies — flush face-to-face contact fuses into a
    single rectangle and is not recoverable from a top-down silhouette, so it is
    out of scope. The Euclidean distance transform of the mask peaks at each
    item's center and dips at the neck.

    The markers are the PROMINENT EDT maxima (h-maxima transform): a single item
    rasterizes — once rotated — into a ridge of many near-equal maxima, so plain
    peak counting over-splits it at every non-axis-aligned yaw. The prominence
    test keeps a maximum only if it rises more than _TOUCH_PEAK_PROMINENCE_PX
    above the saddle joining it to a taller one, which merges rasterization bumps
    (prominence <1 px) yet keeps two touching items apart (their saddle dips to
    the neck half-width). The markers then grassfire outward THROUGH the mask (a
    geodesic seeded split, not scipy's background-leaking watershed_ift — see
    docs/decisions.md): the neck is reached from both bodies at once, so the cut
    lands on it and each body keeps its true footprint. One prominent peak -> the
    mask is returned unchanged (no over-split); two compact body-like lobes -> one
    sub-mask each. Sub-masks partition the input exactly (no pixels lost or shared).
    The transforms run on the component's bbox crop for speed.
    """
    from scipy.ndimage import distance_transform_edt, label
    from scipy.spatial import ConvexHull, QhullError

    ys, xs = np.where(mask)
    # The foreground labeler also sees tiny depth speckles. _find_items filters
    # them before this function, but keep the primitive safe for direct callers
    # and degenerate masks: Qhull requires a genuinely 2-D point cloud.
    if xs.size < 4 or np.ptp(xs) == 0 or np.ptp(ys) == 0:
        return [mask]
    # Fast path: a convex silhouette (a single item) fills its convex hull; only a
    # neck between two touching bodies drops solidity below the gate. Skip the
    # h-maxima reconstruction and grassfire in that (common) case.
    pts = np.column_stack([xs, ys]).astype(float)
    try:
        component_hull = ConvexHull(pts)
        hull_area = float(component_hull.volume)
    except QhullError:
        # A split is an optional refinement. If the hull cannot be constructed,
        # fail closed to one component instead of crashing the perception node;
        # _find_items will still reject an invalid zero-width bbox below.
        return [mask]
    if hull_area <= 0.0:
        return [mask]
    if float(mask.sum()) / hull_area > _TOUCH_MIN_SOLIDITY:
        return [mask]
    y0, y1, x0, x1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
    sub = mask[y0:y1, x0:x1]
    dist = distance_transform_edt(sub).astype(float)
    if dist.max() <= 0.0:
        return [mask]
    long_px, short_px, _ = _obb_dims_px(pts[component_hull.vertices])
    aspect = long_px / short_px if short_px > 0.0 else float("inf")
    if aspect > _TOUCH_MAX_SPLIT_ASPECT:
        return [mask]
    compactness = float(dist.max()) / short_px if short_px > 0.0 else 0.0
    if compactness < _TOUCH_MIN_COMPACTNESS:
        return [mask]
    # h-maxima: reconstruct dist from (dist - h). This flattens every sub-h bump
    # into its surroundings, so the ridge/cap of one item becomes a single
    # regional maximum while two touching items keep two, split by their (deeper)
    # saddle. The prominent peaks are the regional maxima of hmax: pixels that a
    # second reconstruction from just below cannot fill up to hmax (a plain
    # local-max filter is fooled by the flat diagonal ridges of a square's EDT).
    from scipy.ndimage import grey_dilation

    hmax = _reconstruct_by_dilation(dist - _TOUCH_PEAK_PROMINENCE_PX, dist)
    peaks = (hmax > _reconstruct_by_dilation(hmax - 1e-6, hmax)) & (dist > 0.0)
    markers, n = label(peaks, structure=np.ones((3, 3), dtype=int))
    # This optional refinement has evidence for one specific ambiguity only:
    # TWO touching products.  With three or more prominent peaks the binary
    # silhouette cannot distinguish several products from one irregular body
    # (Box400 and Pouf produced 3 and 10 peaks in valid single-item poses).  Fail
    # closed to the original component: under-splitting an unsupported 3-way
    # contact is preferable to inventing phantom items and measuring only the
    # largest fragment of a real product.
    if n != 2:
        return [mask]
    # Grassfire from the markers, constrained to the mask: grow every label one
    # pixel per step into unassigned mask pixels until the whole component is
    # claimed. Growth is geodesic (it flows THROUGH the silhouette, not across the
    # background), so the neck is reached from both bodies at once and the boundary
    # lands there — a Euclidean nearest-marker split instead cuts a straight line
    # between the compact peaks and inflates each body's oriented bbox on offset
    # contacts (docs/experiments.md). grey_dilation propagates the larger label
    # into ties, an arbitrary but deterministic tie-break along the seam.
    assigned = markers.copy()
    while True:
        unassigned = (assigned == 0) & sub
        if not unassigned.any():
            break
        grown = grey_dilation(assigned, size=3)
        newly = unassigned & (grown > 0)
        if not newly.any():
            break
        assigned[newly] = grown[newly]
    out = []
    for label_id in range(1, n + 1):
        piece = assigned == label_id
        if not piece.any():
            continue
        full = np.zeros_like(mask)
        full[y0:y1, x0:x1] = piece
        out.append(full)
    return out


def _find_items(depth_m, belt_depth_m, margin_m):
    """Valid item masks and bboxes in one frame; touching items are split."""
    from scipy.ndimage import label

    foreground = _item_mask(depth_m, belt_depth_m, margin_m)
    labels, count = label(foreground, structure=np.ones((3, 3), dtype=int))
    h, w = depth_m.shape
    found = []
    for component_id in range(1, count + 1):
        component = labels == component_id
        # Preserve the pre-split contract: noise below _MIN_ITEM_PX never reaches
        # ConvexHull. The seed-1 stream exposed this when a tiny speck raised
        # QhullError before the old size filter had a chance to discard it.
        if int(component.sum()) < _MIN_ITEM_PX:
            continue
        for mask in _split_touching(component):
            ys, xs = np.where(mask)
            if xs.size < _MIN_ITEM_PX:
                continue
            # A diagonal one-pixel streak spans both x and y, so the bbox checks
            # below accept it even though its point cloud is still rank 1. Such
            # depth noise cannot have a 2-D hull and used to raise QhullError in
            # measure_item(), killing the whole perception node. Reject it here
            # so it also cannot win _find_item() over a valid small neighbour.
            dx, dy = xs - xs[0], ys - ys[0]
            anchor = np.flatnonzero((dx != 0) | (dy != 0))[0]
            if not np.any(dx * dy[anchor] != dy * dx[anchor]):
                continue
            x0, y0, x1, y1 = int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())
            if x0 == x1 or y0 == y1:
                continue
            if x0 == 0 or y0 == 0 or x1 == w - 1 or y1 == h - 1:
                continue
            found.append((mask, (x0, y0, x1, y1)))
    return found


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
            # collinear: the minimal enclosing circle is the diameter of the two
            # farthest points (it covers the middle one). Return that instead of
            # a 2-point circle that could leave the third point uncovered.
            p, q = max(((a, b), (a, c), (b, c)), key=lambda pq: dist(pq[0], pq[1]))
            return circle2(p, q)
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
                    circle = circle3(p, q, s)
    return circle[1]


def _roundness_k(pts, hull):
    """K = r_inscribed / R_circumscribed of the convex `hull` of mask pixels `pts`.

    Same definition as the task criterion and the docs/md/models.md analysis:
    the hull (not the raw outline), so a cylinder with tie-wraps reads as a
    rounded square, not a circle. Dimensionless: pixel scale cancels out.

    BOTH RADII COME FROM ONE COMMON CENTRE. The organizers' figure
    (docs/md/images/circle_criterion.png) draws the two circles concentric in all
    three examples: one centre, r out to the nearest edge, R out to the farthest
    vertex. Taking r from the Chebyshev centre but R from an INDEPENDENT minimal
    enclosing circle mixes two centres and inflates K -- it lifted Шлем
    0.783 -> 0.820 and Мешок 0.800 -> 0.827, carrying both across the 0.8
    threshold into D against their reference B (docs/decisions.md 2026-07-19).
    The centre is optimised, not assumed: fixing it at the Chebyshev point
    under-reports elongated outlines (Цилиндр 0.749 -> 0.695) and would drift
    from the published models.md table.
    """
    from scipy.optimize import linprog, minimize

    a_ub = np.hstack([hull.equations[:, :2], np.ones((len(hull.equations), 1))])
    b_ub = -hull.equations[:, 2]
    res = linprog(c=[0.0, 0.0, -1.0], A_ub=a_ub, b_ub=b_ub,
                  bounds=[(None, None), (None, None), (0.0, None)], method="highs")
    c0 = res.x[:2]  # Chebyshev centre: inside the hull, a good starting point

    normals, offsets = hull.equations[:, :2], hull.equations[:, 2]
    verts = pts[hull.vertices]

    def neg_ratio(c):
        r = float(-(normals @ c + offsets).max())  # distance to the nearest edge
        if r <= 0.0:                               # centre escaped the hull
            return 1.0
        big_r = float(np.linalg.norm(verts - c, axis=1).max())
        return -r / big_r

    best = minimize(neg_ratio, c0, method="Nelder-Mead",
                    options={"xatol": 1e-3, "fatol": 1e-6, "maxiter": 400})
    k = -float(best.fun)
    assert k > 0.0, "degenerate hull"
    return float(min(k, 1.0))


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


def _axial_symmetry_residual(xs, ys, heights_m, nbins=10):
    """RMS spread of height WITHIN a radius ring, normalised by the item's height.

    A body of revolution standing on the belt (ball, upright can) has a height map
    that depends on radius alone, so the spread inside a ring is ~0. A slumped bag
    does not: at the same radius its surface wanders. This is the discriminator the
    silhouette cannot supply — Мешок fills a round hull exactly like a rigid round
    body does (solidity was refuted on the real bag frame for that reason,
    docs/decisions.md 2026-07-12), but its RELIEF is irregular where a ball's is not.

    Rings with too few pixels are skipped; each surviving ring is weighted by its
    population so the wide outer rings dominate, as they should.
    """
    r = np.hypot(xs - xs.mean(), ys - ys.mean())
    h_max = float(heights_m.max())
    if r.max() <= 0.0 or h_max <= 0.0:
        return None
    edges = np.linspace(0.0, float(r.max()), nbins + 1)
    idx = np.clip(np.digitize(r, edges) - 1, 0, nbins - 1)
    spreads, weights = [], []
    for b in range(nbins):
        ring = heights_m[idx == b]
        if len(ring) < 8:  # too few pixels for a meaningful spread
            continue
        spreads.append(float(ring.std()))
        weights.append(float(len(ring)))
    if not spreads:
        return None
    spreads, weights = np.asarray(spreads), np.asarray(weights)
    return float(np.sqrt((weights * spreads**2).sum() / weights.sum()) / h_max)


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


# Body-OBB relief gate: share of the item's height that the visible surface itself
# spans (robust p95-p05). A flat cloud (a box lid: relief ~0) reveals nothing about
# the hidden sides, so only the shadow-box dims are honest for it; a cloud with real
# relief (dome, lying cylinder: relief ~1) exposes the body's form and licenses a
# tilted bounding box. 0.5 splits the measured populations (boxes/detergent ~0.05
# vs dome-family >=0.9) with a wide margin on both sides.
BODY_OBB_MIN_RELIEF = 0.5


def _body_obb_dims_mm(xs, ys, depth_col_m, heights_m, fx, fy, cx, cy,
                      legacy_dims_mm, dz_mm, px_pad_mm):
    """Dims (mm, desc) of the smallest box around the item's BODY, or None.

    The top-view OBB measures the item's SHADOW: a body resting on a tilted hull
    facet projects chords longer than any of its true edges — the tilted-rest
    helmet reads 373x336 mm against an intrinsic 352x298 and crosses the 320 limit,
    diverting a B item to C (docs/experiments.md 2026-07-14). The depth frame
    carries the third coordinate, so bound the body instead: backproject the mask
    to 3D and take the smallest-volume box over hull-facet-aligned orientations
    (O'Rourke's flush-face initialization — exact for polyhedral rests, a few mm
    for domes), keeping the belt-aligned shadow box (`legacy_dims_mm`) as the
    candidate to beat.

    Two corrections keep the tilt honest, both because the camera sees only the
    upper surface: (1) no candidate may be thinner than the measured height above
    the belt — a lying cylinder's visible half-shell is only half its diameter
    deep, but the body reaches the belt; (2) a near-flat cloud (a box lid) hides
    the sides entirely, so it licenses no tilted box at all (see
    BODY_OBB_MIN_RELIEF) — without this gate the minimum-volume box of a bare
    rectangle SLICES the hidden body diagonally and under-reports its long edge.
    The two in-plane extents carry the same +1 px inclusive-pixel pad as the
    top-view footprint.
    """
    relief = np.percentile(heights_m, 95.0) - np.percentile(heights_m, 5.0)
    if dz_mm <= 0.0 or relief * 1000.0 < BODY_OBB_MIN_RELIEF * dz_mm:
        return None

    from scipy.spatial import ConvexHull, QhullError

    pts_mm = np.column_stack([
        (xs - cx) * depth_col_m / fx,
        (ys - cy) * depth_col_m / fy,
        heights_m,
    ]) * 1000.0
    # Decimate before the hull: extents of 1500 surface points match the full
    # cloud's within ~one pixel, and both the hull-vertex count and the candidate
    # count below scale with it. Deterministic stride, not random (Karpathy #5).
    if len(pts_mm) > 1500:
        pts_mm = pts_mm[np.linspace(0, len(pts_mm) - 1, 1500).astype(int)]
    try:
        hull3 = ConvexHull(pts_mm)
    except QhullError:
        return None  # degenerate cloud (the 9 mm pen): the shadow box is all there is
    verts = pts_mm[hull3.vertices]

    # Candidate box normals: every hull facet orientation (O'Rourke's flush-face
    # initialization), quantized to ~3 degrees. The full un-quantized set on an
    # un-decimated dome is thousands of near-duplicates (2339 on the tilted-helmet
    # frame -> 7.4 s/frame in a 15 Hz node: the verdict-trimesh cost lesson again);
    # quantization costs <1 mm of extent while cutting the search to ~30 ms. PCA
    # axes were tried as a cheap substitute and are NOT one: the dome shell's
    # principal axes track the BELT, not the body — the winning facet on the
    # tilted-helmet frame is 70 degrees away from every PCA axis, and the PCA-only
    # search kept the item oversized.
    normals = hull3.equations[:, :3]
    normals = np.unique(np.round(normals / np.linalg.norm(normals, axis=1, keepdims=True)
                                 / 0.15), axis=0) * 0.15
    normals = normals / np.linalg.norm(normals, axis=1, keepdims=True)
    if len(normals) > 200:  # hard bound on the frame budget, whatever the geometry
        normals = normals[np.linspace(0, len(normals) - 1, 200).astype(int)]

    # In-plane search by brute angle, one matmul for EVERY (candidate, angle) pair:
    # the per-candidate rotating-calipers loop is pure Python and cost seconds on
    # a dome's hundreds of candidates. A min-area rectangle's angle is 90-degree
    # periodic; 2-degree steps cost <0.1% of extent.
    seeds = np.where(np.abs(normals[:, :1]) < 0.9, [[1.0, 0.0, 0.0]], [[0.0, 1.0, 0.0]])
    u = np.cross(normals, seeds)
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    v = np.cross(normals, u)
    angles = np.arange(0.0, np.pi / 2, np.radians(3.0))
    cos_a, sin_a = np.cos(angles)[None, :, None], np.sin(angles)[None, :, None]
    d1 = (u[:, None, :] * cos_a + v[:, None, :] * sin_a).reshape(-1, 3)
    d2 = (v[:, None, :] * cos_a - u[:, None, :] * sin_a).reshape(-1, 3)
    proj = verts @ np.concatenate([d1, d2, normals]).T
    extents = proj.max(axis=0) - proj.min(axis=0)
    n_pairs = len(d1)
    e1 = extents[:n_pairs] + px_pad_mm
    e2 = extents[n_pairs:2 * n_pairs] + px_pad_mm
    thick = np.maximum(np.repeat(extents[2 * n_pairs:], len(angles)), dz_mm)

    volumes = e1 * e2 * thick
    best = int(np.argmin(volumes))
    if volumes[best] >= float(np.prod(legacy_dims_mm)):
        return list(legacy_dims_mm)
    return sorted([float(e1[best]), float(e2[best]), float(thick[best])], reverse=True)


def measure_item(depth_m, belt_depth_m=BELT_DEPTH_M, fx=FX, fy=FY, margin_m=MASK_MARGIN_M,
                 camera_x_m=CAMERA_X_M, camera_y_m=CAMERA_Y_M, cx=None, cy=None,
                 _found=None):
    """Measurement of the single item on the belt, or None (empty / partial view).

    depth_m: HxW depth image in meters (0 = no return). Two lateral dims come
    from the oriented bounding box (yaw-invariant footprint) at the item's
    top-face depth; height from how far the top rises above the belt. K from the
    top-view hull; position from the mask centroid via the verified pixel->world
    mapping. cx/cy: principal point (px); default to the image center — the node
    passes CameraInfo's k[2]/k[5] so a shifted principal point is honored.
    """
    # measure_items() has already segmented the full frame. Reusing its exact
    # mask avoids copying a belt-sized frame and running _find_items once more
    # per product; ordinary single-item callers keep the public behavior.
    found = _find_item(depth_m, belt_depth_m, margin_m) if _found is None else _found
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
    if cx is None:
        cx = depth_m.shape[1] / 2.0
    if cy is None:
        cy = depth_m.shape[0] / 2.0
    # Footprint in MILLIMETERS, every mask pixel backprojected with ITS OWN depth.
    # A single median depth takes the scale from the item's TOP and applies it to
    # the silhouette of its widest section, which on a convex body sits deeper:
    # a systematic UNDER-read that grows with convexity (Пуфик 65.1 mm of error
    # -> 6.7 mm, 11/11 against 8/11 offline; docs/plan-fix-depth-scale.md). The
    # pixel OBB above stays as it is — the silhouette K and the elongation gate
    # below are scale-free ratios and must keep reading pixels, not millimeters.
    pts_mm = np.column_stack([(xs - cx) * depth_m[mask] / fx,
                              (ys - cy) * depth_m[mask] / fy]) * 1000.0
    hull_mm = ConvexHull(pts_mm)
    long_mm, short_mm, _ = _obb_dims_px(pts_mm[hull_mm.vertices])
    # +1 px of inclusive-pixel pad, as the pixel path had; at the median depth,
    # which is where that convention was calibrated.
    px_pad_mm = top_depth_m / fx * 1000.0
    dx_mm = long_mm + px_pad_mm
    dy_mm = short_mm + px_pad_mm
    # height = the item's HIGHEST point (bounding box), not the mask median:
    # concave items (Тарелка — rim 27 mm, dish bottom 8 mm) would otherwise
    # read below the 10 mm min-dim threshold and flip category to C.
    # 1st percentile, not min: robust to stray depth returns.
    dz_mm = (belt_depth_m - float(np.percentile(depth_m[mask], 1.0))) * 1000.0
    dims = sorted([dx_mm, dy_mm, dz_mm], reverse=True)
    if not _dims_are_sane(dims):
        return None

    world_x = camera_x_m - (float(ys.mean()) - cy) * top_depth_m / fy
    world_y = camera_y_m - (float(xs.mean()) - cx) * top_depth_m / fx
    world_z = BELT_TOP_Z_M + dz_mm / 1000.0 / 2.0

    heights_m = belt_depth_m - depth_m[mask]
    k_section = _section_roundness_k(xs, ys, heights_m, long_dir, top_depth_m / fx)
    # The footprint box above measures the item's SHADOW, which a tilted body
    # outgrows (the tilted-rest helmet: 373x336 against an intrinsic 352x298 —
    # a B item reads C); when the visible relief exposes the body's form, bound
    # the body itself. UNLESS the hidden end-section is a proven circle: a lying
    # body of revolution is already belt-aligned, its shadow box IS its body box,
    # and the only thing a smaller box can cut there is the invisible lower
    # half-section (the day-4 bottle read 69 mm against its true 91 that way).
    if k_section <= ROUND_K_THRESHOLD:
        body_dims = _body_obb_dims_mm(xs, ys, depth_m[mask], heights_m, fx, fy,
                                      cx, cy, dims, dz_mm, px_pad_mm)
        if body_dims is not None:
            dims = body_dims
    if not _dims_are_sane(dims):
        return None

    # K = max of the top-view silhouette and the vertical cross-section: a body
    # of revolution lying on its side is a rectangle from above (low silhouette
    # K) but a circle in its hidden end section (K=1). max, per the criterion
    # "max r_in/R_circ over sections along the principal axes" (docs/md/models.md).
    k_silhouette = _roundness_k(pts, hull)
    # The top-view silhouette means D only for a FLAT disc. A thick round lump
    # (Мешок slumped on the belt) fills a round hull too (silhouette K=0.88,
    # solidity=1.0 — indistinguishable from Тарелка by silhouette shape), so once
    # the silhouette CLAIMS roundness, require the item to be flat; otherwise drop
    # the silhouette K and let the cross-section K decide (0 for a lump -> B). The
    # flat disc (dz/diam ~0.14) is untouched. See FLATNESS_MAX; provenance
    # docs/decisions.md 2026-07-12 (solidity refuted on the real bag frame).
    # CLAMP, not zero. Zeroing made a 0.003 difference in silhouette K explode into
    # 0.8 of reported K: the real Мешок frame reads 0.8015 and its render 0.799, so
    # one collapsed to 0.0 while the other kept its value, though BOTH route to B
    # (test_render_depth::bag_2). Clamping to the threshold expresses the same policy
    # — a thick round lump must not reach D through its silhouette — while keeping the
    # signal continuous: k > threshold is strict, so exactly-threshold is still B, and
    # max(k_silhouette, k_section) is unchanged for every verdict either way.
    # ...UNLESS the relief says it really is a body of revolution. The flatness gate
    # alone vetoed EVERY thick round silhouette, which left the pipeline with no route
    # to D for a compact round body at all — ball and squat_can rode to B, the exact
    # item the sorter must never receive (KNOWN_GAPS, docs/probe-models.md). Axial
    # symmetry separates the two: a ball's height depends on radius alone (0.0236),
    # a slumped bag's does not (0.0702).
    axial = _axial_symmetry_residual(xs, ys, heights_m)
    is_body_of_revolution = axial is not None and axial < AXIAL_SYMMETRY_MAX
    if (k_silhouette > ROUND_K_THRESHOLD
            and dz_mm > FLATNESS_MAX * max(dx_mm, dy_mm)
            and not is_body_of_revolution):
        k_silhouette = ROUND_K_THRESHOLD
    # The section circle is only the hidden end of a LYING body of revolution,
    # which is elongated (long footprint >> short). A near-cuboidal blob (Мешок)
    # whose ridge merely happens to fit a belt-tangent circle is not one, so drop
    # the section K unless the footprint is elongated (bag oi=1 K=1.0->B, census #3;
    # bottle 3.3x clears it, Цилиндр stays B on the rms gate). Ratio is scale-free,
    # so the raw px extents suffice (no depth term).
    if long_px < SECTION_MIN_ELONGATION * short_px:
        k_section = 0.0

    return Measurement(
        dims_mm=dims,
        k=max(k_silhouette, k_section),
        position_m=(world_x, world_y, world_z),
        bbox_px=(x0, y0, x1, y1),
    )


def measure_items(depth_m, belt_depth_m=BELT_DEPTH_M, fx=FX, fy=FY,
                  margin_m=MASK_MARGIN_M, camera_x_m=CAMERA_X_M,
                  camera_y_m=CAMERA_Y_M, cx=None, cy=None):
    """Measure every disconnected, fully visible item in a depth frame.

    Geometry remains single-sourced in measure_item(), but the masks found in the
    one full-frame segmentation are passed directly into that path. Touching
    products may already be separate masks after the conservative EDT split.
    """
    measurements = []
    for found in _find_items(depth_m, belt_depth_m, margin_m):
        measurement = measure_item(
            depth_m,
            belt_depth_m=belt_depth_m,
            fx=fx,
            fy=fy,
            margin_m=margin_m,
            camera_x_m=camera_x_m,
            camera_y_m=camera_y_m,
            cx=cx,
            cy=cy,
            _found=found,
        )
        if measurement is not None:
            measurements.append(measurement)
    return sorted(measurements, key=lambda measurement: measurement.position_m[0])


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


def _write_png(image, out_path):
    """Encode a PNG in memory and write it through Python's Unicode path API."""
    import cv2

    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise RuntimeError(f"OpenCV could not encode PNG: {out_path}")
    with open(out_path, "wb") as stream:
        stream.write(encoded.tobytes())


def save_depth_png(depth_m, out_path):
    """Write a depth frame (meters) as the uint16-millimeter PNG load_depth_png reads.

    Inverse of load_depth_png and Unicode-safe by construction: encodes in memory
    and writes through Python's path API. cv2.imwrite silently misses a non-ASCII
    Windows path (a participant may check out under such a username), so the
    perception dump must not use it — same defect/fix as the overlay writers.
    """
    mm = (np.nan_to_num(depth_m) * 1000.0).clip(0, 65535).astype(np.uint16)
    _write_png(mm, out_path)


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
    _write_png(vis, out_path)
    return None if m is None else m.dims_mm


def render_items_overlay(depth_m, tagged_items):
    """BGR debug frame: every item's bbox with its id, dims/K and aggregation state.

    tagged_items: iterable of (item_id, Measurement, state_text_or_None). The
    perception node passes the tracker's ids and the classifier's last state per
    id ("B conf=0.95"), so the frame shows what the PIPELINE currently thinks
    about each item, not just the geometry (Karpathy #4). Pure drawing — the
    measuring stays in measure_items().
    """
    import cv2

    gray = np.clip((depth_m - 1.2) / (2.0 - 1.2), 0, 1)  # belt/floor band -> readable
    vis = cv2.cvtColor((gray * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    for item_id, m, state in tagged_items:
        x0, y0, x1, y1 = m.bbox_px
        cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 2)
        dims = "x".join(f"{d:.0f}" for d in m.dims_mm)
        cv2.putText(vis, f"id={item_id} {dims}mm K={m.k:.2f}",
                    (x0, max(y0 - 26, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(vis, state if state else "unclassified",
                    (x0, max(y0 - 8, 30)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return vis


def save_items_overlay(depth_m, tagged_items, out_path):
    """Write render_items_overlay() to out_path (PNG), including Unicode paths.

    OpenCV's Windows ``imwrite`` can return True while silently writing to a
    mojibake path when a directory or filename is non-ASCII. Encode in memory
    and let Python's Unicode-aware file API own the path instead.
    """
    _write_png(render_items_overlay(depth_m, tagged_items), out_path)
