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
    # Regression (origin mismatch): the model's origin is its default-pose BOTTOM
    # (build_item_models.set_belt_origin, Z=0 at the lowest point) and Gazebo
    # rotates the model about that origin. The old formula used height/2 (a
    # CENTRE-origin assumption), so at seed 0 items spawned up to 250 mm ABOVE the
    # belt (box_400 oi=0, pouf oi=0 -> dropped) or ~90 mm INSIDE it (helmet oi=2,
    # pouf oi=2 -> wedged at the spawn). The contract: the item's lowest point in
    # EVERY pose rests exactly one clearance above the belt top.
    pytest.importorskip("trimesh")
    import trimesh

    from build_item_models import ITEMS, STL_DIR, set_belt_origin
    from spawn_orientations import SPAWN_CLEARANCE_M, spawn_height_m

    from src.perception import BELT_TOP_Z_M

    def belt_gap_m(slug, item_index, orient_index):
        quat = orientation_quat(0, item_index, orient_index)
        mesh = trimesh.load(str(STL_DIR / f"{ITEMS[slug][0]}.stl"), force="mesh")
        set_belt_origin(mesh)
        x, y, z, w = quat
        mesh.apply_transform(trimesh.transformations.quaternion_matrix([w, x, y, z]))
        lowest_world = spawn_height_m(slug, quat) + mesh.bounds[0][2] / 1000.0
        return lowest_world - BELT_TOP_Z_M

    # a cell the old formula floated (box_400 upright, +205 mm) and cells it buried
    # (box_400 on edge -55 mm, helmet turned -89 mm, pouf turned -74 mm) all now
    # rest exactly one clearance above the belt.
    for slug, idx, oi in [("box_400x400x300", 2, 0), ("box_400x400x300", 2, 2),
                          ("helmet", 10, 2), ("pouf", 6, 2), ("plate", 8, 0)]:
        assert belt_gap_m(slug, idx, oi) == pytest.approx(SPAWN_CLEARANCE_M, abs=1e-4)
