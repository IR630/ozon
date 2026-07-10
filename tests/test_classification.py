# -*- coding: utf-8 -*-
"""Classification rules (docs/md/task.md, section 2) and geometry smoke tests.

Boundary cases from docs/md/models.md are pinned here on purpose:
they are direct jury points for "устойчивость в пограничных случаях".
"""
from pathlib import Path

import numpy as np
import pytest

from analyze_models import CAT_B, CAT_C, CAT_D, analyze_file, classify

STL = Path(__file__).resolve().parents[1] / "docs" / "Stl"


class TestClassifyRules:
    def test_fits_and_not_round_goes_to_b(self):
        assert classify([200, 100, 100], k=0.5) == CAT_B

    def test_round_within_gabarits_goes_to_d(self):
        assert classify([300, 90, 90], k=0.95) == CAT_D

    def test_oversized_goes_to_c(self):
        assert classify([500, 100, 100], k=0.5) == CAT_C

    def test_priority_gabarits_over_shape_oversized(self):
        # Пуфик case: round AND oversized -> C, not D
        assert classify([489, 489, 264], k=1.0) == CAT_C

    def test_priority_gabarits_over_shape_undersized(self):
        # Ручка case: round AND one dim below 10 mm -> C, not D
        assert classify([148, 13, 9], k=0.99) == CAT_C

    def test_k_exactly_at_threshold_is_not_round(self):
        # criterion is strictly K > 0.8 (docs/md/images/circle_criterion.png)
        assert classify([200, 100, 100], k=0.8) == CAT_B

    def test_dims_order_does_not_matter(self):
        assert classify([100, 500, 100], k=0.5) == CAT_C


class TestGeometryOnTestSet:
    """Smoke tests on the organizers' STL set (fast models only)."""

    def test_box_dims_match_its_name(self):
        r = analyze_file(STL / "Короб 300х200х200.stl")
        assert np.allclose(np.asarray(r["dims"]), [301, 200.5, 200], atol=2)
        assert r["cat"] == CAT_B

    def test_plate_is_round(self):
        r = analyze_file(STL / "Тарелка.stl")
        assert r["k"] > 0.8
        assert r["cat"] == CAT_D

    def test_cylinder_with_tie_rods_is_not_round(self):
        # boundary case: named "Цилиндр" but hull of its section is a rounded
        # square (4 tie rods), so by the formal criterion it is NOT round
        r = analyze_file(STL / "Цилиндр.stl")
        assert r["k"] < 0.8
        assert r["cat"] == CAT_B

    def test_big_box_is_oversized(self):
        r = analyze_file(STL / "Короб 400х400х300.stl")
        assert r["cat"] == CAT_C


if __name__ == "__main__":
    pytest.main([__file__, "-q"])
