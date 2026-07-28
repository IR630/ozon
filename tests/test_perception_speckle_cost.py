# -*- coding: utf-8 -*-
"""Segmentation must cost what the ITEMS cost, not what the NOISE costs.

Sensor noise shatters a depth frame into thousands of one-pixel components above
the belt margin. `_find_items` used to build a full 640x480 boolean mask for every
one of them before the size gate could throw it away, so a frame that contains a
single plate cost 2.8 s to segment at sigma 3 mm against a 66.7 ms frame period.

That is not a cosmetic inefficiency. It is the mechanism behind the 15 census cells
that a two-head rig lost under noise (runs/c2v3_20260725_020520_seed0): the cells
measured and classified CORRECTLY and still failed, because the verdict arrived
after the item had passed the diverter — `conf=0.20, n=1` where a healthy cell
aggregates ~13 frames. Both facts below are locked here: the answer must not change,
and the cost must not scale with the discarded speckles.
"""
import time

import numpy as np

from src.perception import BELT_DEPTH_M, MASK_MARGIN_M, measure_items

# RELATIVE, not absolute: an absolute millisecond bound is a stopwatch on whatever
# machine happens to run it, and a shared CI runner is exactly where that misfires.
# The contract is that speckles the size gate throws away cost almost nothing, so the
# speckled frame is compared against the SAME frame without them. Measured: the old
# per-component full-frame mask cost ~35x the clean frame, the bincount+bbox pass
# costs ~1.2x. A 5x bound fails the old implementation by 7x and passes the new one
# with 4x of headroom.
_MAX_SPECKLE_COST_RATIO = 5.0
_SPECKLES = 4000


def _belt_with_plate():
    """Empty belt carrying one plate-sized body, both parallel to the sensor."""
    depth = np.full((480, 640), BELT_DEPTH_M, dtype=float)
    depth[200:260, 280:340] = BELT_DEPTH_M - 0.030
    return depth


def _add_speckles(depth, count, rng):
    """Isolated single pixels lifted just clear of the mask margin — what noise makes.

    Placed on a stride so they stay disconnected from each other, and kept out of a
    ring around the item: a speckle touching the body merges into its component and
    legitimately moves the hull, which would test the noise model rather than the
    segmentation cost this file is about.
    """
    out = depth.copy()
    ys = rng.choice(np.arange(4, 476, 3), size=count)
    xs = rng.choice(np.arange(4, 636, 3), size=count)
    far = ~((ys >= 190) & (ys < 270) & (xs >= 270) & (xs < 350))
    out[ys[far], xs[far]] = BELT_DEPTH_M - MASK_MARGIN_M - 0.002
    return out


def test_speckle_components_do_not_change_what_is_found():
    rng = np.random.default_rng(0)
    clean = _belt_with_plate()
    speckled = _add_speckles(clean, _SPECKLES, rng)

    found_clean = measure_items(clean)
    found_speckled = measure_items(speckled)

    assert len(found_clean) == 1, "the fixture itself must show exactly one body"
    assert len(found_speckled) == 1, "sub-gate speckles must not become items"
    assert found_speckled[0].dims_mm == found_clean[0].dims_mm
    assert found_speckled[0].k == found_clean[0].k


def _median_seconds(frame, repeats=3):
    measure_items(frame)                 # warm scipy/numpy import paths
    runs = []
    for _ in range(repeats):
        start = time.perf_counter()
        measure_items(frame)
        runs.append(time.perf_counter() - start)
    return sorted(runs)[len(runs) // 2]


def test_segmentation_cost_does_not_scale_with_discarded_speckles():
    rng = np.random.default_rng(0)
    clean = _belt_with_plate()
    speckled = _add_speckles(clean, _SPECKLES, rng)

    ratio = _median_seconds(speckled) / _median_seconds(clean)

    assert ratio < _MAX_SPECKLE_COST_RATIO, (
        f"{_SPECKLES} sub-gate speckles made segmentation {ratio:.1f}x more "
        "expensive; the per-component full-frame mask is back")
