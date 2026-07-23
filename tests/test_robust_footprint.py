# -*- coding: utf-8 -*-
"""The robust mm footprint must reject depth-noise outliers a convex hull admits.

This is the source-level fix for the one census miss that survived to the contour:
the helmet's width inflated 297 -> 354 mm under range noise because the footprint was
the convex hull of every mask pixel and a single outlier set the dimension
(docs/decisions.md 2026-07-23). These pin the two properties that fix must have —
outlier rejection and yaw invariance — and the deliberate baseline shift a trim costs.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.perception import _robust_footprint_mm  # noqa: E402


def _filled_rect(w_mm, h_mm, step=2.0):
    xs = np.arange(-w_mm / 2, w_mm / 2 + step, step)
    ys = np.arange(-h_mm / 2, h_mm / 2 + step, step)
    gx, gy = np.meshgrid(xs, ys)
    return np.column_stack([gx.ravel(), gy.ravel()])


def _rotate(pts, deg):
    t = np.radians(deg)
    r = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
    return pts @ r.T


def test_clean_rectangle_reads_true_within_trim():
    long_mm, short_mm, _ = _robust_footprint_mm(_filled_rect(300, 200))
    # A 1% trim per end shaves ~2% off a dense filled extent; nothing more.
    assert 0.95 * 300 <= long_mm <= 300
    assert 0.95 * 200 <= short_mm <= 200


def test_noise_outliers_do_not_inflate():
    pts = _filled_rect(300, 200)  # ~15k dense points
    # A handful of depth-noise outliers far outside the body — the exact thing a
    # convex hull would promote to a vertex and read as the dimension.
    outliers = np.array([[600, 0], [-600, 0], [0, 500], [0, -500], [550, 400]])
    long_mm, short_mm, _ = _robust_footprint_mm(np.vstack([pts, outliers]))
    # 5 outliers out of ~15k are < 1% and must be trimmed: dims stay near true,
    # nowhere near the 1200 mm a hull would report from the +/-600 points.
    assert long_mm < 320
    assert short_mm < 220


def test_yaw_invariant():
    base_long, base_short, _ = _robust_footprint_mm(_filled_rect(300, 200))
    for deg in (15, 30, 45, 70):
        long_mm, short_mm, _ = _robust_footprint_mm(_rotate(_filled_rect(300, 200), deg))
        assert abs(long_mm - base_long) < 8.0
        assert abs(short_mm - base_short) < 8.0


def test_long_axis_tracks_orientation():
    _l, _s, direction = _robust_footprint_mm(_rotate(_filled_rect(300, 100), 30))
    ang = np.degrees(np.arctan2(direction[1], direction[0])) % 180
    assert min(abs(ang - 30), abs(ang - 210 % 180)) < 5.0
