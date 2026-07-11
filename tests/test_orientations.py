# -*- coding: utf-8 -*-
"""Seed-driven spawn orientations (day 4, P1+P2) — pure Python.

Pins reproducibility: the same (seed, item, orient) always yields the same
rotation, which is exactly the milestone's `seed` guarantee (docs/decisions.md).
"""
import numpy as np

from spawn_orientations import orientation_quat


def test_orient_index_zero_is_identity():
    assert orientation_quat(0, 3, 0) == (0.0, 0.0, 0.0, 1.0)


def test_same_cell_is_reproducible():
    a = orientation_quat(42, 5, 2)
    b = orientation_quat(42, 5, 2)
    assert a == b


def test_quaternion_is_unit_norm():
    for oi in range(1, 6):
        q = orientation_quat(7, 1, oi)
        assert np.isclose(np.linalg.norm(q), 1.0)


def test_distinct_cells_differ():
    assert orientation_quat(1, 0, 1) != orientation_quat(2, 0, 1)  # seed
    assert orientation_quat(1, 0, 1) != orientation_quat(1, 1, 1)  # item
    assert orientation_quat(1, 0, 1) != orientation_quat(1, 0, 2)  # orientation


def test_canonical_hemisphere():
    for oi in range(1, 10):
        assert orientation_quat(3, 2, oi)[3] >= 0.0  # w >= 0
