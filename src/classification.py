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
    MEASUREMENT_TOL_MM,
    MEASUREMENT_TOL_VOLUME_FRAC,
    MIN_DIMS_MM,
    ROUND_K_THRESHOLD,
    SANE_DIM_MM_MAX,
    SANE_DIM_MM_MIN,
)

# Conservative-policy margin (day 4, P4). A "fits" verdict within this many mm of
# a sorter size bound is fragile under measurement noise, so we bias it to C.
# Small on purpose: the boundary cases are held on the correct side by frame
# aggregation, not by this margin (Цилиндр's 435 sits 15 mm from the 450 limit
# and must stay B — docs/decisions.md). This is not a domain threshold, so it
# lives here, not in constants.py.
CONSERVATIVE_DIM_MARGIN_MM = 5.0


def measurement_error(measured_dims_mm, truth_dims_mm):
    """(worst per-side error in mm, relative volume error) against ground truth.

    Both dimension triples are compared sorted descending, because neither the
    task nor the organizers assign meaning to the ORDER of an item's extents —
    only to the multiset of them.
    """
    got = np.sort(np.asarray(measured_dims_mm, dtype=float))[::-1]
    truth = np.sort(np.asarray(truth_dims_mm, dtype=float))[::-1]
    if got.shape != (3,) or truth.shape != (3,):
        raise ValueError(f"expected 3 dimensions each, got {measured_dims_mm!r} / {truth_dims_mm!r}")
    if np.any(truth <= 0.0):
        raise ValueError(f"truth dimensions must be positive: {truth}")
    side_err = float(np.max(np.abs(got - truth)))
    vol_err = float(abs(np.prod(got) - np.prod(truth)) / np.prod(truth))
    return side_err, vol_err


def within_measurement_tolerance(measured_dims_mm, truth_dims_mm,
                                 tol_mm=MEASUREMENT_TOL_MM,
                                 tol_volume=MEASUREMENT_TOL_VOLUME_FRAC):
    """Is a measurement inside the accuracy the organizers allow?

    Either criterion passing is enough — the experts said "5 mm on a side and
    10 % by volume, the MAXIMUM of these two parameters" ([08:45]), i.e. the more
    permissive one governs. Which one that is depends on size: 10 % by volume is
    ~3.2 % per side, so it is the looser rule on a 300 mm box (~9.7 mm) and the
    tighter one on a 20 mm part (~0.6 mm), where the flat 5 mm takes over.
    """
    side_err, vol_err = measurement_error(measured_dims_mm, truth_dims_mm)
    return side_err <= tol_mm or vol_err <= tol_volume


def classify(dims_mm, k):
    """Category B/C/D for an item with given dimensions (mm) and roundness K.

    dims_mm: three linear dimensions in any order, millimeters.
    k: r_inscribed / R_circumscribed of the best cross-section, in [0, 1].
    """
    d = np.sort(np.asarray(dims_mm, dtype=float))[::-1]
    if d.shape != (3,):
        raise ValueError(f"expected 3 dimensions, got {dims_mm!r}")
    if not np.all(np.isfinite(d)):
        raise ValueError(f"dimensions must be finite: {d}")
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


def classify_conservative(dims_mm, k, dim_margin_mm=CONSERVATIVE_DIM_MARGIN_MM):
    """classify() plus the conservative policy at the sorter SIZE bounds.

    Strategy (PLAN.md): an error toward B is the expensive one — a wrongly-B item
    jams the main sorter. So a "fits" item measured within dim_margin_mm of a size
    bound (near-oversized or near-undersized) is routed to C instead, where C is
    already the safe verdict for out-of-range items.

    The policy touches the SIZE decision only, NOT the K-near-0.8 roundness
    decision: a symmetric K band would push Шлем (K=0.78) to D against its
    reference B. The roundness edge cases are held by frame aggregation upstream
    (src/aggregation.py); this margin is a small extra guard on genuine size ties.
    """
    strict = classify(dims_mm, k)
    if strict != CATEGORY_B:
        return strict
    d = np.sort(np.asarray(dims_mm, dtype=float))[::-1]
    near_max = np.any(d > np.subtract(MAX_DIMS_MM, dim_margin_mm))
    near_min = np.any(d < np.add(MIN_DIMS_MM, dim_margin_mm))
    if near_max or near_min:
        return CATEGORY_C
    return CATEGORY_B
