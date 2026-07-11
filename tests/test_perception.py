# -*- coding: utf-8 -*-
"""Perception geometry: dims (mm), roundness K, world position (day 2, P3).

Synthetic tests lock the math with analytic numbers and run everywhere
(numpy+scipy only). Real-frame tests check the saved day-1/day-2 camera frames
against ground truth and need cv2 to read PNG — skipped where OpenCV is absent
(Windows dev host), run in WSL and the ROS CI job.
"""
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


def test_measure_empty_belt_returns_none():
    assert measure_dims_mm(np.full((480, 640), 1.5)) is None


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
