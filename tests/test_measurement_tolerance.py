# -*- coding: utf-8 -*-
"""Organizer-allowed measurement accuracy (docs/md/expert_session_qa.md [08:45]).

A measurement counts as correct if it is within 5 mm on a side OR within 10 % by
volume — whichever of the two is the MORE permissive. These cases pin that the
size-dependent switch between the two rules behaves as the experts described, and
that the explicit worked example they gave is not flagged as an error.
"""
import pytest

from src.classification import measurement_error, within_measurement_tolerance
from src.constants import MEASUREMENT_TOL_MM, MEASUREMENT_TOL_VOLUME_FRAC


class TestMeasurementError:
    def test_organizer_example_numbers(self):
        # 451x321x321 vs true 450x320x320: 1 mm per side, ~0.85 % by volume.
        side_err, vol_err = measurement_error((451, 321, 321), (450, 320, 320))
        assert side_err == pytest.approx(1.0)
        assert vol_err == pytest.approx(0.0085, abs=1e-3)

    def test_order_of_extents_is_irrelevant(self):
        # Same multiset of extents, permuted -> perfect measurement.
        side_err, vol_err = measurement_error((320, 450, 320), (450, 320, 320))
        assert side_err == pytest.approx(0.0)
        assert vol_err == pytest.approx(0.0)

    def test_truth_must_be_positive(self):
        with pytest.raises(ValueError):
            measurement_error((10, 10, 10), (10, 0, 10))

    def test_needs_three_dimensions(self):
        with pytest.raises(ValueError):
            measurement_error((10, 10), (10, 10, 10))


class TestWithinTolerance:
    def test_organizer_example_is_not_an_error(self):
        # The experts stated this exact pair is explicitly NOT an error ([26:56]).
        assert within_measurement_tolerance((451, 321, 321), (450, 320, 320))

    def test_flat_5mm_rule_governs_small_parts(self):
        # 20 mm cube measured 25 mm on one side: 5 mm exactly (passes the side
        # rule) but +25 % by volume (fails the volume rule). The looser 5 mm wins.
        assert within_measurement_tolerance((25, 20, 20), (20, 20, 20))

    def test_small_part_beyond_both_rules_fails(self):
        # 6 mm side error and +30 % volume: neither rule saves it.
        assert not within_measurement_tolerance((26, 20, 20), (20, 20, 20))

    def test_volume_rule_governs_large_boxes(self):
        # 450x320x320 measured 349 on the short side: 29 mm per side (far past the
        # flat 5 mm) but only +9.1 % by volume. The looser volume rule wins.
        assert within_measurement_tolerance((450, 320, 349), (450, 320, 320))

    def test_large_box_beyond_volume_rule_fails(self):
        # 355 on the short side: +10.9 % by volume, over the 10 % ceiling, and
        # 35 mm per side is nowhere near 5 mm either.
        assert not within_measurement_tolerance((450, 320, 355), (450, 320, 320))

    def test_defaults_come_from_constants(self):
        assert MEASUREMENT_TOL_MM == 5.0
        assert MEASUREMENT_TOL_VOLUME_FRAC == 0.10
