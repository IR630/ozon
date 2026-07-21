# -*- coding: utf-8 -*-
"""The cloud explainer must not be the thing that is wrong.

It exists because two hand-derived models of the calibration error disagreed
with the simulator, so its own arithmetic gets pinned: a diagnosis tool that
mislabels the belt would send the next reader after the wrong mechanism.
"""
import numpy as np

from scripts.explain_side_cloud import describe
from src.perception import BELT_TOP_Z_M


def test_an_empty_cloud_says_so_rather_than_dividing_by_zero():
    assert "no points survive" in describe(np.empty((0, 3)), "head")


def test_belt_points_are_reported_as_belt():
    """A sheet at belt height, belt width: 100% over the belt and hugging it."""
    xs, ys = np.meshgrid(np.linspace(1.5, 2.2, 40), np.linspace(-0.25, 0.25, 40))
    belt = np.column_stack([xs.ravel(), ys.ravel(),
                            np.full(xs.size, BELT_TOP_Z_M + 0.006)])
    line = describe(belt, "head")
    assert "100.0% over the belt" in line
    assert "100.0% within 20 mm of it" in line
    assert "y-span    500 mm" in line, line


def test_a_real_item_is_not_reported_as_belt():
    """A 100 mm tall box on the belt centre must read as neither belt-hugging
    nor belt-wide, so the two cases stay distinguishable in one glance."""
    xs, ys, zs = np.meshgrid(np.linspace(1.85, 1.95, 8), np.linspace(-0.05, 0.05, 8),
                             np.linspace(BELT_TOP_Z_M + 0.01, BELT_TOP_Z_M + 0.1, 8))
    box = np.column_stack([xs.ravel(), ys.ravel(), zs.ravel()])
    line = describe(box, "head")
    assert "100.0% over the belt" in line, "a centred item IS over the belt"
    assert "100.0% within 20 mm of it" not in line
    assert "y-span    100 mm" in line, line
