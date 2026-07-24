# -*- coding: utf-8 -*-
"""The verdict-flip probe: the ways a fragility number would flatter us.

This probe produces a number that goes into the report, so its two silent
failure modes are tested by value: a lost detection quietly not counting, and
a "reproducible" run that moves between invocations.
"""
import numpy as np

from scripts.probe_verdict_flip import EDGE_ITEMS, flips_under_noise
from src.perception import BELT_DEPTH_M


def _frame_with_a_box(top=1.30):
    depth = np.full((120, 160), BELT_DEPTH_M, dtype=float)
    depth[50:70, 70:90] = top
    return depth


def _empty_belt():
    return np.full((120, 160), BELT_DEPTH_M, dtype=float)


def test_a_lost_detection_counts_as_a_wrong_outcome():
    """On a line an item nobody measured is not a neutral event. Skipping such a
    draw would make high noise look SAFER than low noise, which is exactly
    backwards."""
    flips = flips_under_noise(_empty_belt(), "B", 0.0, 5, np.random.default_rng(0))
    assert flips == 5


def test_the_count_is_reproducible_from_the_generator():
    """The report cites this number with a seed; two identical invocations must
    not disagree."""
    depth = _frame_with_a_box()
    first = flips_under_noise(depth, "B", 0.002, 6, np.random.default_rng([1, 2, 3]))
    second = flips_under_noise(depth, "B", 0.002, 6, np.random.default_rng([1, 2, 3]))
    assert first == second


def test_zero_noise_on_a_correct_pose_flips_nothing():
    """The control row: without noise the probe must agree with the plain
    measurement, or every flip count above it is measuring the probe itself."""
    from src.classification import classify_conservative
    from src.perception import measure_item

    depth = _frame_with_a_box()
    expect = classify_conservative(measure_item(depth).dims_mm, measure_item(depth).k)
    assert flips_under_noise(depth, expect, 0.0, 4, np.random.default_rng(0)) == 0


def test_the_probe_covers_the_items_whose_category_K_decides():
    """Pouf also clamps to the threshold but is settled as C by its 489 mm before
    K is consulted — including it would report flips that cannot happen."""
    slugs = [slug for slug, _idx, _expect in EDGE_ITEMS]
    assert slugs == ["helmet", "bag"]
    assert all(expect == "B" for _slug, _idx, expect in EDGE_ITEMS)
