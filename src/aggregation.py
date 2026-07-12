# -*- coding: utf-8 -*-
"""Per-item frame aggregation (day 4, P4).

The classifier used to react to every camera frame, so a single noisy or
off-axis measurement flipped the category (C<->B<->D). Those flips were patched
downstream in the controller twice (docs/decisions.md) — the wrong place. This
module fixes the dither at the root: it collects the measurements of one item
(keyed by item_id) and returns their component-wise MEDIAN dims and MEDIAN K,
plus a confidence that GROWS with frame count and shrinks when the frames
disagree. The classifier then decides on that stable aggregate, not on one frame.

Pure Python (no rclpy): unit-tested on synthetic frame sequences.
"""
from collections import OrderedDict
from dataclasses import dataclass

import numpy as np

# Frames to reach full count-confidence. ~15 Hz perception over a ~1 s crossing
# gives well over this, so a clean item saturates to 1.0 before the pusher.
N_CONFIDENT = 5
# Dispersion of K that maps agreement to 0 (std of 0.2 in K ~ fully unreliable).
K_SPREAD_REF = 0.2
# Bound memory without assuming items are observed in numeric-ID order. Multi-item
# frames interleave updates (1, 2, 3, 1, ...), so "newest_id - 1" silently erased
# a still-visible item. Sixteen active/recent tracks is ample for a sequential
# belt and keeps a broken tracker from growing history forever.
MAX_TRACKED_ITEMS = 16

# NOTE: algorithm parameters live here, NOT in src/constants.py — that file holds
# only the jury-checked domain thresholds (0.8, 10, 450x320x320). Duplicating
# those numbers would be a bug (CLAUDE.md); these are not those numbers.


@dataclass
class Aggregate:
    """Stable estimate for one item over the frames seen so far.

    Contract mirror for the classifier: dims_mm sorted descending, K in [0, 1].
    """
    dims_mm: list  # 3 floats, sorted descending (median over frames)
    k: float       # median over frames
    confidence: float  # [0, 1], grows with n_frames, damped by frame disagreement
    n_frames: int


class ItemAggregator:
    """Accumulates measurements per item_id and yields their running median.

    update() is called once per ItemMeasurement and returns the aggregate over
    all frames of that item_id so far. A new item_id starts a fresh history.
    """

    def __init__(self):
        self._dims = OrderedDict()  # item_id -> frames; order = least-recently used
        self._k = {}                # item_id -> list of floats

    def update(self, item_id, dims_mm, k):
        if item_id not in self._dims:
            self._dims[item_id] = []
            self._k[item_id] = []
        else:
            self._dims.move_to_end(item_id)
        self._prune()
        d = np.sort(np.asarray(dims_mm, dtype=float))[::-1]
        self._dims[item_id].append(d)
        self._k[item_id].append(float(k))

        dims = np.array(self._dims[item_id])  # (n, 3)
        ks = np.array(self._k[item_id])       # (n,)
        med_dims = np.median(dims, axis=0)
        med_k = float(np.median(ks))
        conf = _confidence(dims, ks, med_dims)
        return Aggregate(
            dims_mm=[float(x) for x in med_dims],
            k=med_k,
            confidence=conf,
            n_frames=len(ks),
        )

    def _prune(self):
        while len(self._dims) > MAX_TRACKED_ITEMS:
            stale, _ = self._dims.popitem(last=False)
            del self._k[stale]


def _confidence(dims, ks, med_dims):
    """Grows with frame count, damped by how much the frames disagree.

    count term: min(1, n / N_CONFIDENT). agreement term: 1 minus the worst of
    (relative dims spread, K spread / reference). One frame -> agreement 1 but a
    low count term, so confidence still starts low and climbs as evidence agrees.
    """
    n = len(ks)
    count_conf = min(1.0, n / N_CONFIDENT)
    dim_rel_spread = float(np.max(np.std(dims, axis=0) / np.maximum(med_dims, 1.0)))
    k_spread = float(np.std(ks))
    dispersion = max(dim_rel_spread, k_spread / K_SPREAD_REF)
    agreement = float(np.clip(1.0 - dispersion, 0.0, 1.0))
    return count_conf * agreement
