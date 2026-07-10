# -*- coding: utf-8 -*-
"""Classification rules (docs/md/task.md, section 2).

Rule priority is fixed by the task: gabarits first (-> C), then
circle-in-section (-> D), otherwise B. This module is the single
implementation of the rules; everything else (scripts, ROS nodes) calls it.
"""
import numpy as np

from src.constants import (
    CATEGORY_B,
    CATEGORY_C,
    CATEGORY_D,
    MAX_DIMS_MM,
    MIN_DIMS_MM,
    ROUND_K_THRESHOLD,
    SANE_DIM_MM_MAX,
    SANE_DIM_MM_MIN,
)


def classify(dims_mm, k):
    """Category B/C/D for an item with given dimensions (mm) and roundness K.

    dims_mm: three linear dimensions in any order, millimeters.
    k: r_inscribed / R_circumscribed of the best cross-section, in [0, 1].
    """
    d = np.sort(np.asarray(dims_mm, dtype=float))[::-1]
    if d.shape != (3,):
        raise ValueError(f"expected 3 dimensions, got {dims_mm!r}")
    if np.any(d < SANE_DIM_MM_MIN) or np.any(d > SANE_DIM_MM_MAX):
        raise ValueError(f"dims out of physical range [{SANE_DIM_MM_MIN}, {SANE_DIM_MM_MAX}] mm: {d}")
    if not 0.0 <= k <= 1.0:
        raise ValueError(f"K out of [0, 1]: {k}")

    fits = bool(np.all(d < MAX_DIMS_MM)) and bool(np.all(d > MIN_DIMS_MM))
    if not fits:
        return CATEGORY_C
    if k > ROUND_K_THRESHOLD:
        return CATEGORY_D
    return CATEGORY_B
