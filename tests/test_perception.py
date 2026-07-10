# -*- coding: utf-8 -*-
"""Perception geometry: dimensions in mm from a top-down depth frame (day 2, P3).

test_measure_synthetic locks the projection math with explicit numbers and runs
everywhere. test_measure_real_frame checks the actual day-1 camera frame against
the ground-truth Короб 300 (±10 mm) and needs cv2 to read the PNG — skipped where
OpenCV is absent (Windows dev host, plain pytest CI), run in WSL.
"""
from pathlib import Path

import numpy as np
import pytest

from src.perception import measure_dims_mm

DEPTH_PNG = Path(__file__).resolve().parents[1] / "docs" / "report" / "img" / "day1_camera_depth.png"


def test_measure_synthetic():
    # belt at 1.5 m, a 100x100 px item top at 1.3 m, fx=fy=500 px
    depth = np.full((480, 640), 1.5)
    depth[100:200, 150:250] = 1.3  # h_px=100, w_px=100
    dims = measure_dims_mm(depth, belt_depth_m=1.5, fx=500.0, fy=500.0)
    # lateral = 100 * 1.3 / 500 * 1000 = 260 mm; height = (1.5-1.3)*1000 = 200 mm
    assert dims == pytest.approx([260.0, 260.0, 200.0])


def test_measure_empty_belt_returns_none():
    assert measure_dims_mm(np.full((480, 640), 1.5)) is None


def test_measure_real_frame():
    pytest.importorskip("cv2")
    from src.perception import load_depth_png

    depth = load_depth_png(DEPTH_PNG)
    dims = measure_dims_mm(depth)
    assert dims is not None
    # ground truth Короб 300: 300 x 200 x 200 mm, sorted descending
    for measured, truth in zip(dims, [300.0, 200.0, 200.0]):
        assert abs(measured - truth) <= 10.0, f"{dims} vs [300, 200, 200]"
