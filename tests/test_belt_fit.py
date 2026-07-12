# -*- coding: utf-8 -*-
"""Which census cells the CELL itself cannot convey, whatever the sorter does."""
import pytest

trimesh = pytest.importorskip("trimesh")

from check_belt_fit import (  # noqa: E402
    BELT_WIDTH_MM,
    RAIL_GAP_MM,
    fits_the_belt,
    lateral_extent_for_mesh_mm,
)


def _box(x_mm, y_mm, z_mm):
    return trimesh.creation.box(extents=(x_mm, y_mm, z_mm))


def test_an_item_narrower_than_the_belt_rides_it():
    assert fits_the_belt(400.0) is None
    assert fits_the_belt(BELT_WIDTH_MM) is None


def test_an_item_wider_than_the_belt_cannot_be_conveyed():
    """The pouf is 489 mm across and the belt is 500 mm: rotate it and it stops fitting.

    Such a census cell is not a routing bug — no sorter with this belt can carry an
    item wider than the belt it rides on. It is a limit of the cell, and it is
    reported as one instead of being chased in perception (docs/decisions.md).
    """
    assert "WIDER THAN THE BELT" in fits_the_belt(BELT_WIDTH_MM + 1)
    assert "RAIL GAP" in fits_the_belt(RAIL_GAP_MM + 1)


def test_rotating_a_big_item_can_push_it_past_the_belt_width():
    identity = (0.0, 0.0, 0.0, 1.0)
    # 45 deg about Z: the diagonal, not the side, now faces across the belt
    yaw45 = (0.0, 0.0, 0.38268343, 0.92387953)

    upright_mm = lateral_extent_for_mesh_mm(_box(489, 489, 264), identity)
    turned_mm = lateral_extent_for_mesh_mm(_box(489, 489, 264), yaw45)

    assert upright_mm == pytest.approx(489, abs=1)     # fits the 500 mm belt
    assert turned_mm == pytest.approx(489 * 2**0.5, abs=2)   # 692 mm — it does not
    assert fits_the_belt(upright_mm) is None
    assert fits_the_belt(turned_mm) is not None
