# -*- coding: utf-8 -*-
"""The noise-floor metric must read ~0 on a clean field and ~sigma on a noisy one.

If this inverts or collapses, the safety gate that decides whether a "noisy" world
is actually noisy is broken, and every 2-vs-3 camera number taken on it is a lie.
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from verify_noise_reaches_depth import depth_noise_floor_mm  # noqa: E402


def _flat_field(mm=1500, h=200, w=200):
    return np.full((h, w), mm, dtype=np.uint16)


def test_clean_flat_field_reads_near_zero():
    floor = depth_noise_floor_mm(_flat_field())
    assert floor is not None
    assert floor < 1.0  # a perfectly flat depth has no high-frequency content


def test_gaussian_noise_field_reads_near_sigma():
    rng = np.random.default_rng(0)
    field = _flat_field().astype(np.float64)
    sigma_mm = 20.0
    noisy = (field + rng.normal(0, sigma_mm, field.shape)).clip(0, 65535).astype(np.uint16)
    floor = depth_noise_floor_mm(noisy)
    assert floor is not None
    # A 3x3 median residual recovers a fraction of the true sigma but must be an
    # order of magnitude above the clean field and clearly non-trivial.
    assert floor > 8.0


def test_all_invalid_pixels_returns_none():
    assert depth_noise_floor_mm(np.zeros((50, 50), dtype=np.uint16)) is None
