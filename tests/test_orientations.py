# -*- coding: utf-8 -*-
"""Seed-driven spawn orientations (day 4, P1+P2) — pure Python.

Pins reproducibility: the same (seed, item, orient) always yields the same
rotation, which is exactly the milestone's `seed` guarantee (docs/decisions.md).
"""
import numpy as np
import pytest

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


def test_spawn_height_rests_the_item_on_the_belt_not_inside_it():
    # Regression (seed-0 census feed_jams): Gazebo creates the item at its CENTRE,
    # so the old fixed z=0.5 buried anything over 200 mm tall inside the belt
    # (top surface 0.4) and the solver ejected it at the spawn. Replaying box_400
    # oi=2 with only this number changed: z=0.5 FAIL at the spawn, z=0.71 PASS.
    pytest.importorskip("trimesh")
    from spawn_orientations import SPAWN_CLEARANCE_M, spawn_height_m

    from src.perception import BELT_TOP_Z_M

    # box_400 on edge in its oi=2 pose is the tallest cell of the matrix (579 mm)
    quat = orientation_quat(0, 2, 2)
    z = spawn_height_m("box_400x400x300", quat)
    assert z > BELT_TOP_Z_M + 0.25  # its half-height alone clears the belt by far
    assert z == pytest.approx(0.71, abs=0.02)

    # An upright flat item barely needs any lift — the height must track the pose,
    # not be a new constant.
    z_flat = spawn_height_m("plate", orientation_quat(0, 8, 0))
    assert z_flat < z
    assert z_flat >= BELT_TOP_Z_M + SPAWN_CLEARANCE_M
