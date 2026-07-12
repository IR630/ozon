# -*- coding: utf-8 -*-
"""Frame aggregation (day 4, P4) — pure Python, runs on the plain pytest job.

Pins the root fix for category dither: the running median must stay on the
correct side even when single frames would flip, and confidence must grow with
agreeing evidence and be damped by disagreement.
"""
from src.aggregation import MAX_TRACKED_ITEMS, N_CONFIDENT, ItemAggregator
from src.classification import classify, classify_conservative


def _last(agg, item_id, frames):
    """Feed a sequence of (dims, k) frames; return the final aggregate."""
    out = None
    for dims, k in frames:
        out = agg.update(item_id, dims, k)
    return out


def test_stable_input_keeps_dims_and_category():
    agg = ItemAggregator()
    frames = [([300.0, 200.0, 200.0], 0.71)] * 5  # Короб 300
    est = _last(agg, 1, frames)
    assert est.dims_mm == [300.0, 200.0, 200.0]
    assert est.k == 0.71
    assert classify_conservative(est.dims_mm, est.k) == "B"
    assert est.n_frames == 5


def test_median_absorbs_a_k_outlier_that_would_flip_per_frame():
    # Цилиндр truth K=0.74 -> B, but one frame reads 0.85 (> 0.8 -> per-frame D).
    frames = [([435.0, 50.0, 43.0], k) for k in (0.70, 0.74, 0.85, 0.74, 0.72)]
    # Per-frame reaction WOULD flip on the outlier:
    assert classify([435.0, 50.0, 43.0], 0.85) == "D"
    est = _last(ItemAggregator(), 1, frames)
    assert est.k == 0.74  # median rejects the 0.85 spike
    assert classify_conservative(est.dims_mm, est.k) == "B"


def test_median_absorbs_a_dims_outlier():
    # One off-axis frame reads a dim +30 mm large; median must reject it.
    frames = [([300.0, 200.0, 200.0], 0.6)] * 4 + [([330.0, 200.0, 200.0], 0.6)]
    est = _last(ItemAggregator(), 1, frames)
    assert est.dims_mm[0] == 300.0


def test_confidence_grows_with_frame_count():
    agg = ItemAggregator()
    c1 = agg.update(1, [300.0, 200.0, 200.0], 0.6).confidence
    est = _last(agg, 1, [([300.0, 200.0, 200.0], 0.6)] * (N_CONFIDENT - 1))
    assert c1 < est.confidence
    assert est.confidence == 1.0  # N_CONFIDENT agreeing frames saturate


def test_disagreement_damps_confidence():
    n = N_CONFIDENT
    tight = _last(ItemAggregator(), 1, [([300.0, 200.0, 200.0], 0.6)] * n)
    noisy = _last(ItemAggregator(), 2,
                  [([300.0, 200.0, 200.0], k) for k in (0.4, 0.9, 0.5, 0.85, 0.6)])
    assert noisy.confidence < tight.confidence


def test_new_item_id_resets_history():
    agg = ItemAggregator()
    _last(agg, 1, [([300.0, 200.0, 200.0], 0.6)] * 3)
    est2 = agg.update(2, [489.0, 489.0, 264.0], 1.0)  # Пуфик, fresh item
    assert est2.n_frames == 1
    assert est2.dims_mm == [489.0, 489.0, 264.0]


def test_three_interleaved_items_keep_independent_histories():
    agg = ItemAggregator()
    items = {
        1: ([300.0, 200.0, 200.0], 0.60),
        2: ([489.0, 489.0, 264.0], 1.00),
        3: ([435.0, 50.0, 43.0], 0.74),
    }

    estimates = {}
    for _ in range(3):
        for item_id, (dims, k) in items.items():
            estimates[item_id] = agg.update(item_id, dims, k)

    assert {item_id: est.n_frames for item_id, est in estimates.items()} == {
        1: 3, 2: 3, 3: 3}
    assert estimates[1].dims_mm == items[1][0]
    assert estimates[2].dims_mm == items[2][0]
    assert estimates[3].dims_mm == items[3][0]


def test_track_history_is_lru_bounded_not_numeric_cutoff():
    agg = ItemAggregator()
    dims = [300.0, 200.0, 200.0]
    for item_id in range(MAX_TRACKED_ITEMS):
        agg.update(item_id, dims, 0.6)

    # Refresh id=0, then add one track: id=1 is the least recently used and is
    # evicted; active id=0 survives even though its numeric value is the oldest.
    agg.update(0, dims, 0.6)
    agg.update(MAX_TRACKED_ITEMS, dims, 0.6)
    assert 0 in agg._dims
    assert 1 not in agg._dims
    assert len(agg._dims) == MAX_TRACKED_ITEMS
