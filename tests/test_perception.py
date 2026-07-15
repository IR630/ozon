# -*- coding: utf-8 -*-
"""Perception geometry: dims (mm), roundness K, world position (day 2, P3).

Synthetic tests lock the math with analytic numbers and run everywhere
(numpy+scipy only). Real-frame tests check the saved day-1/day-2 camera frames
against ground truth and need cv2 to read PNG — skipped where OpenCV is absent
(Windows dev host), run in WSL and the ROS CI job.
"""
import shutil
from pathlib import Path

import numpy as np
import pytest

from src.perception import BELT_DEPTH_M, measure_dims_mm, measure_item, measure_items

IMG_DIR = Path(__file__).resolve().parents[1] / "docs" / "report" / "img"
DEPTH_PNG = IMG_DIR / "day1_camera_depth.png"
OFFSET_PNG = IMG_DIR / "day2_offset_x1.8_y0.1_depth.png"
BOTTLE_PNG = IMG_DIR / "day4_bottle_depth.png"
BAG_PNG = IMG_DIR / "day4_bag_depth.png"
BAG_OI1_PNG = IMG_DIR / "day11_bag_oi1_depth.png"
BAG_OI2_PNG = IMG_DIR / "day11_bag_oi2_depth.png"
HELMET_OI2_PNG = IMG_DIR / "day11_helmet_oi2_depth.png"
PLATE_OI1_PNG = IMG_DIR / "day11_plate_oi1_depth.png"
PLATE_PNG = IMG_DIR / "day4_plate_depth.png"
PEN_PNG = IMG_DIR / "day4_pen_depth.png"
VALIDATION_PEN_PNG = IMG_DIR / "validation_pen" / "depth_000.png"
VALIDATION_PARTIAL_BOX_PNG = IMG_DIR / "validation_partial_box" / "depth_000.png"


def _synthetic_box(rows=slice(100, 200), cols=slice(150, 250), top=1.3):
    depth = np.full((480, 640), 1.5)
    depth[rows, cols] = top
    return depth


def test_measure_synthetic():
    # belt at 1.5 m, a 100x100 px item top at 1.3 m, fx=fy=500 px
    dims = measure_dims_mm(_synthetic_box(), belt_depth_m=1.5, fx=500.0, fy=500.0)
    # lateral = 100 * 1.3 / 500 * 1000 = 260 mm; height = (1.5-1.3)*1000 = 200 mm
    assert dims == pytest.approx([260.0, 260.0, 200.0])


def _synthetic_rotated_rect(l_px, w_px, angle_deg, top=1.3, belt=1.5):
    """Depth frame with an l_px x w_px rectangle rotated by angle_deg about center."""
    depth = np.full((480, 640), belt)
    yy, xx = np.mgrid[0:480, 0:640]
    a = np.deg2rad(angle_deg)
    dx, dy = xx - 320.0, yy - 240.0
    xr = np.cos(a) * dx + np.sin(a) * dy
    yr = -np.sin(a) * dx + np.cos(a) * dy
    depth[(np.abs(xr) <= l_px / 2) & (np.abs(yr) <= w_px / 2)] = top
    return depth


def _section_strip(profile, r_px=40, length_px=200, belt=1.5, fx=2000.0):
    """Depth frame of a prism lying along the columns whose cross-section top
    (across the rows) follows `profile(dy_px) -> height_above_belt_m`. Long axis
    = columns, short axis = rows: the vertical section K reads the row profile.
    fx=2000 keeps heights small vs the belt so a single depth scale is faithful."""
    depth = np.full((480, 640), belt)
    for dy in range(-r_px, r_px + 1):
        h = profile(dy)
        depth[240 + dy, 220:220 + length_px] = belt - h
    return depth


def test_measure_empty_belt_returns_none():
    assert measure_dims_mm(np.full((480, 640), 1.5)) is None


def test_obb_dims_invariant_to_yaw():
    # A 200x120 px box: the oriented bbox recovers its true footprint at ANY yaw.
    # The old axis-aligned bbox would read ~226x226 at 45deg (root cause of the
    # rotation-inflation finding, docs/experiments.md 2026-07-11).
    fx = 500.0
    long_mm = (200 + 1) * 1.3 / fx * 1000.0   # ~522.6
    short_mm = (120 + 1) * 1.3 / fx * 1000.0  # ~314.6
    for angle in (0, 20, 45, 70):
        dims = measure_dims_mm(_synthetic_rotated_rect(200, 120, angle),
                               belt_depth_m=1.5, fx=fx, fy=fx)
        assert dims[0] == pytest.approx(long_mm, rel=0.04), f"long @ {angle}deg: {dims}"
        assert dims[1] == pytest.approx(short_mm, rel=0.04), f"short @ {angle}deg: {dims}"
        assert dims[2] == pytest.approx(200.0, abs=1.0)  # height unaffected by yaw


def test_measure_concave_item_height_from_rim():
    # Тарелка is a dish: rim 27 mm tall, interior bottom ~8 mm. Height must be
    # the rim (bounding box), not the median of the mask — 8 mm would flip the
    # category to C via the min-dim rule (dims have priority over shape)
    depth = np.full((480, 640), 1.5)
    yy, xx = np.mgrid[0:480, 0:640]
    r2 = (xx - 320) ** 2 + (yy - 240) ** 2
    depth[r2 <= 80**2] = 1.5 - 0.008   # dish interior, 8 mm above belt
    depth[(r2 > 60**2) & (r2 <= 80**2)] = 1.5 - 0.027  # rim, 27 mm
    dims = measure_dims_mm(depth, belt_depth_m=1.5, fx=500.0, fy=500.0)
    assert dims[2] == pytest.approx(27.0, abs=1.0)


def test_measure_thin_item_9mm():
    # Ручка rests 9 mm tall (docs/md/models.md) — the mask margin must not
    # swallow it, or perception silently reports an empty belt
    dims = measure_dims_mm(_synthetic_box(top=1.491), belt_depth_m=1.5, fx=500.0, fy=500.0)
    assert dims is not None
    assert dims[2] == pytest.approx(9.0)


def test_corrupt_near_zero_depth_is_not_published_as_a_product():
    """A nonzero invalid return must fail closed before classifier sanity checks."""
    depth = np.full((480, 640), 1.5)
    depth[100:105, 100:105] = 0.001

    assert measure_item(depth, belt_depth_m=1.5, fx=500.0, fy=500.0) is None


def test_partial_item_returns_none():
    # item touching the frame border = riding into view -> refuse to measure
    depth = _synthetic_box(rows=slice(0, 150), cols=slice(200, 300))
    assert measure_item(depth, belt_depth_m=1.5) is None


def test_k_square():
    # square hull spans 99x99 px: r_in = 49.5, R = 49.5*sqrt(2) -> K = 1/sqrt(2)
    m = measure_item(_synthetic_box(), belt_depth_m=1.5, fx=500.0, fy=500.0)
    assert m.k == pytest.approx(1 / np.sqrt(2), abs=0.01)


def test_k_rectangle_300x200():
    # hull 299x199 px: r_in = 99.5, R = hypot(149.5, 99.5) -> K = 0.5540
    m = measure_item(_synthetic_box(rows=slice(100, 300), cols=slice(100, 400)),
                     belt_depth_m=1.5)
    assert m.k == pytest.approx(99.5 / np.hypot(149.5, 99.5), abs=0.01)


def test_k_circle():
    yy, xx = np.mgrid[0:480, 0:640]
    depth = np.full((480, 640), 1.5)
    depth[(xx - 320) ** 2 + (yy - 240) ** 2 <= 80**2] = 1.3
    m = measure_item(depth, belt_depth_m=1.5)
    assert m.k > 0.97  # pixelated circle


def test_section_k_lying_cylinder_is_round():
    # A body of revolution lying on its side (Бутылка, 305x91x91, ref K=1.00->D):
    # the top-view silhouette is a rectangle (silhouette K ~ 0.37, would be B),
    # but the hidden end section is a circle. The vertical section K recovers it.
    s = 1.5 / 2000.0
    depth = _section_strip(lambda dy: s * (40 + np.sqrt(max(40**2 - dy**2, 0.0))))
    m = measure_item(depth, belt_depth_m=1.5, fx=2000.0, fy=2000.0)
    assert m.k > 0.8, f"lying cylinder must read round (D): K={m.k}"


def test_section_k_dome_stays_low():
    # Flat-bottomed semicircle section (dome, Шлем analog, ref K=0.78->B): the
    # arc peaks at only ~1 radius (tau~1), not a full circle (tau~2). The tau
    # gate must keep the section K low so a dome is NOT mistaken for round.
    s = 1.5 / 2000.0
    depth = _section_strip(lambda dy: s * np.sqrt(max(40**2 - dy**2, 0.0)))
    m = measure_item(depth, belt_depth_m=1.5, fx=2000.0, fy=2000.0)
    assert m.k < 0.8, f"dome must stay B: K={m.k}"


def test_section_k_rounded_square_stays_low():
    # Superellipse section (tie-rod Цилиндр, 435x50x43, ref K=0.74->B): its arc
    # is nearly as tall as a circle (tau~1.9) but is a rounded SQUARE, so the
    # circle-fit residual is large. The residual gate must keep it B — this is
    # the organizers' deliberate "named a cylinder but not round" trap.
    s, a, n = 1.5 / 2000.0, 40, 4
    depth = _section_strip(
        lambda dy: s * (a + a * max(1.0 - (abs(dy) / a) ** n, 0.0) ** (1.0 / n)))
    m = measure_item(depth, belt_depth_m=1.5, fx=2000.0, fy=2000.0)
    assert m.k < 0.8, f"rounded-square section must stay B: K={m.k}"


def test_section_k_flat_box_not_inflated():
    # A flat-topped box must not gain roundness from the section path: the
    # section is a straight top -> section K ~ 0, K stays the silhouette value.
    depth = _synthetic_box(rows=slice(100, 300), cols=slice(100, 400))
    m = measure_item(depth, belt_depth_m=1.5)
    assert m.k == pytest.approx(99.5 / np.hypot(149.5, 99.5), abs=0.02)


def _mask_points(mask):
    ys, xs = np.where(mask)
    return np.column_stack([xs, ys]).astype(float)


def test_measure_items_returns_two_disconnected_products():
    from src.perception import measure_items

    depth = np.full((480, 640), BELT_DEPTH_M, dtype=float)
    depth[100:180, 100:180] = 1.30
    depth[260:350, 420:520] = 1.25

    measured = measure_items(depth)

    assert len(measured) == 2
    assert measured[0].position_m[0] < measured[1].position_m[0]
    assert all(measurement.dims_mm[0] > 150 for measurement in measured)


def _two_boxes(belt=1.5, top=1.3):
    """100x100 px item at rows 100..199, 60x60 px item at rows 320..379."""
    depth = np.full((480, 640), belt)
    depth[100:200, 150:250] = top
    depth[320:380, 300:360] = top
    return depth


_BIG_DIMS = [260.0, 260.0, 200.0]     # (99+1) px * 1.3 / 500 * 1000; height 200 mm
_SMALL_DIMS = [200.0, 156.0, 156.0]   # (59+1) px * 1.3 / 500 * 1000, sorted desc


def test_measure_items_keeps_each_items_own_dims():
    # stronger than "two items with dims > 150": each item must carry its OWN
    # footprint. A hull fused over both would span rows 100..379 and report a
    # single ~700 mm blob — the exact numbers are what lock the split.
    items = measure_items(_two_boxes(), belt_depth_m=1.5, fx=500.0, fy=500.0)
    assert [i.dims_mm for i in items] == [pytest.approx(_SMALL_DIMS),
                                          pytest.approx(_BIG_DIMS)]


def test_measure_item_returns_largest_not_fused_union():
    # the single-item entry point still feeds the debug overlay and the offline
    # scripts. On a two-item frame it must degrade to "the largest item", never
    # to the garbage dims of both items' union.
    m = measure_item(_two_boxes(), belt_depth_m=1.5, fx=500.0, fy=500.0)
    assert m.dims_mm == pytest.approx(_BIG_DIMS)


def test_partial_item_does_not_suppress_its_visible_neighbour():
    # an item riding into view touches the border and is refused (garbage dims),
    # but it must not blind perception to the item already fully in frame
    depth = np.full((480, 640), 1.5)
    depth[0:60, 150:250] = 1.3       # entering the frame: touches row 0
    depth[200:300, 300:400] = 1.3    # fully visible
    items = measure_items(depth, belt_depth_m=1.5, fx=500.0, fy=500.0)
    assert len(items) == 1
    assert items[0].dims_mm == pytest.approx(_BIG_DIMS)
    assert measure_item(depth, belt_depth_m=1.5, fx=500.0, fy=500.0) is not None


def test_solidity_filled_square_is_one():
    # a convex filled shape fills its own hull -> solidity ~ 1
    from scipy.spatial import ConvexHull

    from src.perception import _silhouette_solidity
    mask = np.zeros((60, 60), dtype=bool)
    mask[10:50, 10:50] = True
    pts = _mask_points(mask)
    assert _silhouette_solidity(pts, ConvexHull(pts)) == pytest.approx(1.0, abs=0.03)


def test_solidity_filled_disk_is_near_one():
    from scipy.spatial import ConvexHull

    from src.perception import _silhouette_solidity
    yy, xx = np.mgrid[0:80, 0:80]
    mask = (xx - 40) ** 2 + (yy - 40) ** 2 <= 30**2
    pts = _mask_points(mask)
    assert _silhouette_solidity(pts, ConvexHull(pts)) > 0.9


def test_solidity_concave_blob_drops():
    # a plus/cross: the hull is the full bounding square but the mask fills only
    # the arms -> solidity well below 1, the soft-blob signature the bag gate keys
    # on (a round hull that is NOT a filled round shape)
    from scipy.spatial import ConvexHull

    from src.perception import _silhouette_solidity
    mask = np.zeros((90, 90), dtype=bool)
    mask[35:55, 10:80] = True   # horizontal bar
    mask[10:80, 35:55] = True   # vertical bar
    pts = _mask_points(mask)
    assert _silhouette_solidity(pts, ConvexHull(pts)) < 0.7


def test_position_synthetic():
    # mask centroid (199.5, 149.5) px, center (320, 240), top depth 1.3, f=500:
    # world_x = 1.5 - (149.5-240)*1.3/500 ; world_y = 0 - (199.5-320)*1.3/500
    m = measure_item(_synthetic_box(), belt_depth_m=1.5, fx=500.0, fy=500.0)
    x, y, z = m.position_m
    assert x == pytest.approx(1.5 + 90.5 * 1.3 / 500, abs=1e-6)
    assert y == pytest.approx(120.5 * 1.3 / 500, abs=1e-6)
    assert z == pytest.approx(0.4 + 0.1, abs=1e-6)  # belt top + height/2


def test_min_enclosing_radius_collinear():
    # Degenerate hull: three collinear points. The enclosing circle must be the
    # diameter of the two farthest (r = half the span), not a 2-point circle that
    # leaves the third uncovered (circle3 collinear-branch hardening).
    from src.perception import _min_enclosing_radius

    pts = np.array([[0.0, 0.0], [5.0, 0.0], [10.0, 0.0]])
    assert _min_enclosing_radius(pts) == pytest.approx(5.0)


def test_position_honors_principal_point():
    # cx/cy default to image center; a shifted principal point must move the
    # recovered world position. Locks that measure_item CONSUMES cx/cy (regression:
    # perception_node adopted fx/fy from CameraInfo but dropped k[2]/k[5]).
    depth = _synthetic_box()          # mask centroid (xs=199.5, ys=149.5) px
    cx, cy = 300.0, 220.0             # shifted 20 px from the 320/240 center
    m = measure_item(depth, belt_depth_m=1.5, fx=500.0, fy=500.0, cx=cx, cy=cy)
    x, y, z = m.position_m
    # world_x = camera_x - (ys - cy)*depth/fy ; world_y = camera_y - (xs - cx)*depth/fx
    assert x == pytest.approx(1.5 - (149.5 - cy) * 1.3 / 500.0, abs=1e-6)
    assert y == pytest.approx(0.0 - (199.5 - cx) * 1.3 / 500.0, abs=1e-6)


def test_measure_real_frame():
    pytest.importorskip("cv2")
    from src.perception import load_depth_png

    m = measure_item(load_depth_png(DEPTH_PNG))
    assert m is not None
    # ground truth Короб 300: 300 x 200 x 200 mm, sorted descending
    for measured, truth in zip(m.dims_mm, [300.0, 200.0, 200.0]):
        assert abs(measured - truth) <= 10.0, f"{m.dims_mm} vs [300, 200, 200]"
    # top view of the box is a 300x200 rectangle -> K = 100/hypot(150,100) = 0.55
    assert m.k == pytest.approx(0.5547, abs=0.05)
    # box was spawned under the camera at world (1.5, 0)
    assert m.position_m[0] == pytest.approx(1.5, abs=0.015)
    assert m.position_m[1] == pytest.approx(0.0, abs=0.015)


def test_measure_real_offset_frame():
    # frame with the box spawned at world (1.8, 0.1): locks the pixel->world
    # axis mapping (sign errors would flip these coordinates)
    pytest.importorskip("cv2")
    from src.perception import load_depth_png

    m = measure_item(load_depth_png(OFFSET_PNG))
    assert m is not None
    assert m.position_m[0] == pytest.approx(1.8, abs=0.02)
    assert m.position_m[1] == pytest.approx(0.1, abs=0.02)
    # off-axis view: the camera sees a bit of the box side faces, so lateral
    # dims read up to ~20 mm large (v0 limitation, docs/day2-plan.md)
    for measured, truth in zip(m.dims_mm, [300.0, 200.0, 200.0]):
        assert abs(measured - truth) <= 20.0, f"{m.dims_mm} vs [300, 200, 200]"


def test_measure_real_bottle_frame_is_round():
    # Real top-down depth of Бутылка lying on its side (Gazebo, day 4). The
    # top-view silhouette is a rectangle (silhouette K ~ 0.3), so this locks the
    # vertical cross-section K that reads the hidden round end: K must clear the
    # 0.8 roundness threshold so the item routes to D (ground truth 305x91x91,
    # ref K=1.00). The synthetic section tests fix the math; this fixes reality.
    pytest.importorskip("cv2")
    from src.perception import load_depth_png

    m = measure_item(load_depth_png(BOTTLE_PNG))
    assert m is not None
    for measured, truth in zip(m.dims_mm, [305.0, 91.0, 91.0]):
        assert abs(measured - truth) <= 20.0, f"{m.dims_mm} vs [305, 91, 91]"
    assert m.k > 0.8, f"lying bottle must read round (-> D): K={m.k}"


def test_measure_real_bag_frame_is_not_round():
    # Real top-down depth of Мешок settled under the camera (Gazebo, day 4). The
    # slumped bag fills a ROUND convex hull (silhouette K=0.88, solidity=1.0 —
    # indistinguishable from a plate by shape), so silhouette K alone misroutes it
    # to D against reference B (196x173x166 mm, ref K=0.72). The flatness gate keys
    # on the one real difference vs a Тарелка: the bag is a THICK round lump
    # (dz/diameter ~0.9), not a flat disc (~0.14). K must drop below the 0.8
    # threshold so the item routes to B. Threshold read off this real frame, never
    # synthetic (docs/decisions.md, Karpathy #1: solidity was refuted here).
    pytest.importorskip("cv2")
    from src.perception import load_depth_png

    m = measure_item(load_depth_png(BAG_PNG))
    assert m is not None
    for measured, truth in zip(m.dims_mm, [196.0, 173.0, 166.0]):
        assert abs(measured - truth) <= 20.0, f"{m.dims_mm} vs [196, 173, 166]"
    assert m.k <= 0.8, f"thick round lump must NOT read round (-> B): K={m.k}"


def test_measure_real_bag_oi1_frame_is_not_round():
    # Real belt-ride frame of Мешок in the seeded oi=1 pose (seed 0, item 4,
    # orient 1) — the one confirmed misroute of census #3: perception measured
    # 214x174x169 mm K=1.00 -> D against reference B. Root cause (Karpathy #1, real
    # frames, docs/decisions.md 2026-07-13): NOT the silhouette (K=0.74, below 0.8)
    # but the vertical cross-section — the blob's top ridge fits a belt-tangent
    # circle (tau=2.05, rms/R=0.015) almost as tightly as a real lying Бутылка
    # (rms/R=0.009). The elongation gate keys on the one physical difference: a
    # lying body of revolution is elongated (bottle 296/90 ~3.3), the bag is a
    # near-cuboidal lump (214/168 ~1.27) with no hidden long axis. K must drop
    # below 0.8 so the item routes to B.
    pytest.importorskip("cv2")
    from src.classification import classify_conservative
    from src.perception import load_depth_png

    m = measure_item(load_depth_png(BAG_OI1_PNG))
    assert m is not None
    for measured, truth in zip(m.dims_mm, [214.0, 174.0, 169.0]):
        assert abs(measured - truth) <= 20.0, f"{m.dims_mm} vs [214, 174, 169]"
    assert m.k <= 0.8, f"near-cuboidal bag lump must NOT read round (-> B): K={m.k}"
    assert classify_conservative(m.dims_mm, m.k) == "B", (
        f"bag oi=1 must route to B, not D: {m.dims_mm}, K={m.k}"
    )


def test_measure_real_bag_oi2_frame_is_not_round():
    # Day-11 validation set: a SECOND seeded bag pose (seed 0, item 4, orient 2),
    # belt-ride frame. Guards that the elongation-gate fix did not disturb another
    # bag pose — here the blob is held B by the flatness gate (silhouette) and the
    # section rms gate, not elongation, so it exercises a different route to B.
    pytest.importorskip("cv2")
    from src.classification import classify_conservative
    from src.perception import load_depth_png

    m = measure_item(load_depth_png(BAG_OI2_PNG))
    assert m is not None
    assert m.k <= 0.8, f"bag oi=2 must NOT read round (-> B): K={m.k}"
    assert classify_conservative(m.dims_mm, m.k) == "B", f"{m.dims_mm}, K={m.k}"


def test_measure_real_helmet_oi2_frame_is_not_round():
    # Day-11 validation set: Шлем in the seeded oi=2 belt-ride pose — the
    # OBB/K-straddle cell flagged in PLAN-WEEK2 (held B by ~3 mm and K near 0.8).
    # Locks the current baseline (K~0.77 < 0.8 -> B) before the helmet
    # lateral-inflation slice tunes anything.
    pytest.importorskip("cv2")
    from src.classification import classify_conservative
    from src.perception import load_depth_png

    m = measure_item(load_depth_png(HELMET_OI2_PNG))
    assert m is not None
    assert m.k <= 0.8, f"helmet dome must NOT read round (-> B): K={m.k}"
    assert classify_conservative(m.dims_mm, m.k) == "B", f"{m.dims_mm}, K={m.k}"


def test_measure_real_plate_oi1_frame_stays_round():
    # Day-11 validation set: Тарелка in the seeded oi=1 belt-ride pose. It settles
    # flat (213x211x29), so the silhouette route legitimately routes it to D. Locks
    # the plate baseline for the plate-edge slice.
    pytest.importorskip("cv2")
    from src.classification import classify_conservative
    from src.perception import load_depth_png

    m = measure_item(load_depth_png(PLATE_OI1_PNG))
    assert m is not None
    assert m.k > 0.8, f"flat round plate must read round (-> D): K={m.k}"
    assert classify_conservative(m.dims_mm, m.k) == "D", f"{m.dims_mm}, K={m.k}"


def test_measure_real_plate_frame_stays_round():
    # Regression guard for the flatness gate: a real flat round Тарелка
    # (213x211x29 mm, dz/diameter ~0.14) must still read round via the silhouette
    # (K=0.98 > 0.8 -> D). The gate suppresses silhouette K only for THICK round
    # items (Мешок); it must leave the genuine flat disc untouched.
    pytest.importorskip("cv2")
    from src.perception import load_depth_png

    m = measure_item(load_depth_png(PLATE_PNG))
    assert m is not None
    assert m.k > 0.8, f"flat round plate must read round (-> D): K={m.k}"


def test_measure_real_pen_frame_is_seen_and_routed_to_c():
    # Real top-down depth of Ручка (148x13x9 mm) settled on the belt (Gazebo, day 4).
    # The thinnest item in the task: its mask is only ~179 px, so the old 200 px
    # min-px gate returned None on every pose — the item was never measured and the
    # contour defaulted it to B (matrix pen 0/3). It must now be SEEN.
    #
    # The 9 mm dimension is 3.3 px at this camera (2.72 mm/px), so it measures
    # 10.7 mm — straddling the very 10 mm limit that decides its category. That tie
    # is NOT resolvable by the camera and must not be resolved by a tuned threshold:
    # the conservative policy owns it (a "fits" dim within 5 mm of a bound -> C).
    # This test therefore asserts the CONTOUR's verdict, not a lucky measurement.
    pytest.importorskip("cv2")
    from src.classification import classify_conservative
    from src.perception import load_depth_png

    m = measure_item(load_depth_png(PEN_PNG))
    assert m is not None, "the pen must be detected at all (regression: 200 px gate)"
    assert max(m.dims_mm) == pytest.approx(148.0, abs=10.0), f"pen length: {m.dims_mm}"
    assert classify_conservative(m.dims_mm, m.k) == "C", (
        f"pen must route to C (min dim near the 10 mm bound): {m.dims_mm}, K={m.k}"
    )


def test_frozen_validation_pen_rgbd_capture_is_detected_and_routes_c():
    """Fresh final-world RGBD capture closes the Pen validation slice."""
    pytest.importorskip("cv2")
    from src.classification import classify_conservative
    from src.perception import load_depth_png

    m = measure_item(load_depth_png(VALIDATION_PEN_PNG))
    assert m is not None, "the 9 mm pen must survive the depth speck filter"
    assert max(m.dims_mm) == pytest.approx(148.0, abs=10.0), m.dims_mm
    assert classify_conservative(m.dims_mm, m.k) == "C", (m.dims_mm, m.k)


def test_frozen_real_partial_box_is_rejected_instead_of_measured_as_a_small_item():
    """A real object clipped by the camera border must not become a fake product."""
    pytest.importorskip("cv2")
    from src.perception import load_depth_png

    depth = load_depth_png(VALIDATION_PARTIAL_BOX_PNG)
    assert measure_item(depth) is None
    assert measure_items(depth) == []


def test_empty_belt_is_not_an_item():
    # Guard for lowering the min-px gate to 24: the belt itself must never look like
    # an item. Measured on real empty-belt frames the mask is 0 px (Gazebo, day 4);
    # this locks the invariant against a future margin/threshold change.
    assert measure_item(np.full((480, 640), 1.5)) is None


def test_items_overlay_draws_every_item_with_id_and_state(tmp_path):
    # Day 9 debt: the debug overlay must show EVERY item with its id and the
    # pipeline's aggregation state, not just the largest blob's geometry.
    cv2 = pytest.importorskip("cv2")
    from src.perception import save_items_overlay

    depth = np.full((480, 640), 1.5)
    depth[80:160, 100:180] = 1.30
    depth[280:370, 430:530] = 1.25
    items = measure_items(depth, belt_depth_m=1.5, fx=500.0, fy=500.0)
    assert len(items) == 2
    by_row = sorted(items, key=lambda m: m.bbox_px[1])
    tagged = [(7, by_row[0], "B conf=0.95"), (8, by_row[1], None)]

    out = tmp_path / "overlay.png"
    save_items_overlay(depth, tagged, out)

    # cv2.imread/imwrite do not reliably support non-ASCII Windows paths (our
    # checkout and pytest temp both live below C:\Users\Максим). Decode bytes so
    # this test checks the PNG itself rather than OpenCV's narrow path API.
    vis = cv2.imdecode(np.frombuffer(out.read_bytes(), dtype=np.uint8), cv2.IMREAD_COLOR)
    assert vis.shape == (480, 640, 3)
    green = (vis[:, :, 0] == 0) & (vis[:, :, 1] == 255) & (vis[:, :, 2] == 0)
    # both bboxes are painted, so the overlay is multi-item (top edges hit)
    assert green[80, 100:180].any()
    assert green[280, 430:530].any()
    # each item carries a text block above its bbox (id/dims line + state line)
    assert green[54:80, 100:320].any()
    assert green[254:280, 430:640].any()


def test_load_depth_png_from_unicode_path(tmp_path):
    pytest.importorskip("cv2")
    from src.perception import load_depth_png

    unicode_dir = tmp_path / "данные"
    unicode_dir.mkdir()
    unicode_png = unicode_dir / "глубина.png"
    shutil.copyfile(DEPTH_PNG, unicode_png)

    depth = load_depth_png(unicode_png)
    assert depth.shape == (480, 640)
    assert depth.dtype == np.float64
    assert depth[240, 320] == pytest.approx(1.3)


def test_tilted_helmet_dims_measure_the_body_not_its_shadow():
    """A B item resting on a tilted hull facet must not read as oversized.

    The helmet's intrinsic box is 352x298x282 mm (mid margin 22 mm), but on its
    tilted rest (~2% of its resting weight) the TOP-VIEW footprint projects
    373x336 — both over the 320 limit — and the guarded belt now carries that
    pose stably to the camera, so the shadow-box dims divert a B item to C
    end-to-end (measured; docs/experiments.md 2026-07-14). The depth frame
    carries the third coordinate: dims must bound the BODY, not its shadow.
    """
    pytest.importorskip("cv2")
    from src.classification import classify_conservative
    from src.perception import load_depth_png

    frame = (Path(__file__).resolve().parent / "fixtures" / "frames"
             / "helmet_tilt_depth.png")
    m = measure_item(load_depth_png(frame))

    assert m is not None
    assert classify_conservative(m.dims_mm, m.k) == "B", (
        f"tilted-rest helmet read {sorted(m.dims_mm, reverse=True)} -> "
        "oversized: the dims describe the item's shadow, not the item")
