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

from src.perception import measure_dims_mm, measure_item

IMG_DIR = Path(__file__).resolve().parents[1] / "docs" / "report" / "img"
DEPTH_PNG = IMG_DIR / "day1_camera_depth.png"
OFFSET_PNG = IMG_DIR / "day2_offset_x1.8_y0.1_depth.png"


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


def test_position_synthetic():
    # mask centroid (199.5, 149.5) px, center (320, 240), top depth 1.3, f=500:
    # world_x = 1.5 - (149.5-240)*1.3/500 ; world_y = 0 - (199.5-320)*1.3/500
    m = measure_item(_synthetic_box(), belt_depth_m=1.5, fx=500.0, fy=500.0)
    x, y, z = m.position_m
    assert x == pytest.approx(1.5 + 90.5 * 1.3 / 500, abs=1e-6)
    assert y == pytest.approx(120.5 * 1.3 / 500, abs=1e-6)
    assert z == pytest.approx(0.4 + 0.1, abs=1e-6)  # belt top + height/2


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
