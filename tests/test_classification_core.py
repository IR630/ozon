# -*- coding: utf-8 -*-
"""Rules core in src/classification.py: categories, priority, sanity checks.

Mirrors the pinned boundary cases from docs/md/models.md at the level of the
production core (scripts/analyze_models.py is a thin label wrapper over it).
"""
import pytest

from src.classification import classify, classify_conservative
from src.constants import CATEGORY_B, CATEGORY_C, CATEGORY_D


class TestRules:
    def test_fits_and_not_round_goes_to_b(self):
        assert classify([200, 100, 100], k=0.5) == CATEGORY_B

    def test_round_within_gabarits_goes_to_d(self):
        assert classify([300, 90, 90], k=0.95) == CATEGORY_D

    def test_oversized_goes_to_c(self):
        assert classify([500, 100, 100], k=0.5) == CATEGORY_C

    def test_pouf_round_but_oversized_goes_to_c(self):
        assert classify([489, 489, 264], k=1.0) == CATEGORY_C

    def test_pen_round_but_undersized_goes_to_c(self):
        assert classify([148, 13, 9], k=0.99) == CATEGORY_C

    def test_k_exactly_at_threshold_is_not_round(self):
        assert classify([200, 100, 100], k=0.8) == CATEGORY_B

    def test_dims_order_does_not_matter(self):
        assert classify([100, 500, 100], k=0.5) == CATEGORY_C


class TestConservativePolicy:
    """Day 4, P4: conservative size policy biases fragile 'fits' verdicts to C,
    without disturbing the pinned boundary four (docs/decisions.md)."""

    def test_boundary_four_route_correctly(self):
        # Reference dims/K from docs/md/models.md -> reference zones.
        assert classify_conservative([435, 50, 43], k=0.74) == CATEGORY_B  # Цилиндр
        assert classify_conservative([352, 298, 282], k=0.78) == CATEGORY_B  # Шлем
        assert classify_conservative([148, 13, 9], k=0.99) == CATEGORY_C     # Ручка
        assert classify_conservative([489, 489, 264], k=1.0) == CATEGORY_C   # Пуфик

    def test_cylinder_435_stays_b_not_swallowed_by_margin(self):
        # 435 is 15 mm from the 450 limit — the small margin must NOT reach it.
        assert classify_conservative([435, 50, 43], k=0.74) == CATEGORY_B

    def test_near_oversized_fit_biases_to_c(self):
        # Fits strictly (448 < 450) but within 5 mm of the bound -> conservative C.
        assert classify([448, 300, 300], k=0.5) == CATEGORY_B
        assert classify_conservative([448, 300, 300], k=0.5) == CATEGORY_C

    def test_near_undersized_fit_biases_to_c(self):
        # Smallest dim 12 mm fits (> 10) but within 5 mm of the floor -> C.
        assert classify([200, 100, 12], k=0.5) == CATEGORY_B
        assert classify_conservative([200, 100, 12], k=0.5) == CATEGORY_C

    def test_policy_does_not_touch_roundness_decision(self):
        # Шлем-like K just under 0.8 stays B; a symmetric K band would wrongly D it.
        assert classify_conservative([352, 298, 282], k=0.79) == CATEGORY_B

    def test_comfortable_fit_stays_b(self):
        assert classify_conservative([200, 100, 100], k=0.5) == CATEGORY_B


class TestSanity:
    """Karpathy principle 6: fail loudly on physically impossible inputs."""

    def test_k_above_one_raises(self):
        with pytest.raises(ValueError):
            classify([200, 100, 100], k=1.5)

    def test_negative_k_raises(self):
        with pytest.raises(ValueError):
            classify([200, 100, 100], k=-0.1)

    def test_zero_dim_raises(self):
        with pytest.raises(ValueError):
            classify([200, 100, 0], k=0.5)

    def test_absurd_dim_raises(self):
        with pytest.raises(ValueError):
            classify([5000, 100, 100], k=0.5)

    def test_wrong_dim_count_raises(self):
        with pytest.raises(ValueError):
            classify([200, 100], k=0.5)
