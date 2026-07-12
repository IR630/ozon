# -*- coding: utf-8 -*-
"""Geometry locks for the diverter world's chutes (day 6, sim/worlds/cell_diverter.sdf).

The chute is what makes the gentle diverter actually REACH the zone: it guides the
item off the belt edge down to the patch instead of dropping it 0.4 m onto bare
floor (where it stopped 2-6 cm short — bottle/pen census failures).

Its geometry has two hard constraints, both found the hard way in Gazebo, and
both invisible in a diff — hence these tests:
  1) the chute must pass UNDER the blade, or it wedges it (the first cut ran the
     plate up to belt height and the diverter could not swing at all);
  2) it must stay OUT of the camera window, or perception segments it as the item.
Pure XML math — no simulator needed.
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

WORLD = Path(__file__).resolve().parents[1] / "sim" / "worlds" / "cell_diverter.sdf"

# Camera window: the frame's far edge at belt height (docs/decisions.md).
CAMERA_WINDOW_FAR_X = 2.4
BELT_EDGE_Y = 0.25
BELT_TOP_Z = 0.4


def model_pose_and_size(name):
    """(pose, box size) of a static model's collision box, from the world file."""
    root = ET.parse(WORLD).getroot()
    model = root.find(f".//model[@name='{name}']")
    assert model is not None, f"model {name} missing from {WORLD.name}"
    pose = [float(v) for v in model.find("pose").text.split()]
    size = [float(v) for v in model.find(".//collision/geometry/box/size").text.split()]
    return pose, size


def chute_edges(name):
    """Top and bottom edge (y, z) of a rolled chute plate."""
    (_, y0, z0, roll, _, _), (_, width, _) = model_pose_and_size(name)
    # the plate's two long edges sit at local (0, +-width/2, 0); rolling about +X
    # sends them to (y0 +- (w/2)cos(roll), z0 +- (w/2)sin(roll))
    dy, dz = (width / 2) * math.cos(roll), (width / 2) * math.sin(roll)
    a, b = (y0 - dy, z0 - dz), (y0 + dy, z0 + dz)
    return (a, b) if a[1] > b[1] else (b, a)  # (top, bottom)


def blade_bottom_z(name):
    """Lowest z the blade sweeps through (its box hangs off the pivot z)."""
    (_, _, pivot_z, _, _, _), (_, _, height) = model_pose_and_size(name)
    return pivot_z - height / 2


@pytest.mark.parametrize(("chute", "blade", "side"), [("chute_c", "diverter_c", +1),
                                                      ("chute_d", "diverter_d", -1)])
def test_chute_passes_under_the_blade(chute, blade, side):
    # The blade parks at y=+-0.28 — INSIDE the chute's y-span — so a chute that
    # reaches belt height (z=0.4) cuts straight through it and the diverter cannot
    # swing at all (bottle then rides the belt to the end: verified in Gazebo).
    (_, top_z), _ = chute_edges(chute)
    assert top_z < blade_bottom_z(blade), (
        f"{chute} top edge z={top_z:.3f} would wedge {blade} "
        f"(bottom z={blade_bottom_z(blade):.3f})")


@pytest.mark.parametrize(("chute", "side"), [("chute_c", +1), ("chute_d", -1)])
def test_chute_spans_the_belt_edge_to_the_patch(chute, side):
    (top_y, top_z), (bottom_y, bottom_z) = chute_edges(chute)
    # Top edge at the belt edge: no gap for the item to fall into...
    assert top_y == pytest.approx(side * BELT_EDGE_Y, abs=0.02)
    # ...and only a small step down from the belt, not the full 0.4 m drop.
    assert BELT_TOP_Z - top_z < 0.1
    # Bottom edge on the floor, inside the zone patch (patch spans |y| 0.5..1.3).
    assert bottom_z == pytest.approx(0.0, abs=0.02)
    assert 0.5 <= abs(bottom_y) <= 1.3


@pytest.mark.parametrize("blade", ["diverter_c", "diverter_d"])
def test_the_blade_rides_above_the_belt_and_does_not_cut_through_it(blade):
    """The blade's bottom edge was 50 mm INSIDE the belt slab (day 10).

    It swings in past the belt edge, so a blade whose bottom is below the belt
    surface collides with the conveyor — and stalls at 0.04 rad the moment the
    joint is force-controlled. It never showed up under velocity control, because
    a velocity command in Gazebo is a kinematic servo that overrides contact and
    simply bulldozed the blade through the belt. Everything measured about this
    mechanism before day 10 was measured with a blade cutting the conveyor.

    The clearance stays SMALL on purpose: the thinnest item is the 9 mm pen, which
    a blade riding high would pass straight under.
    """
    bottom_z = blade_bottom_z(blade)
    clearance_mm = (bottom_z - BELT_TOP_Z) * 1000

    assert clearance_mm > 0, (
        f"{blade} bottom z={bottom_z:.3f} is inside the belt (top z={BELT_TOP_Z}) "
        "— it will grind the conveyor as it swings in")
    assert clearance_mm <= 5, (
        f"{blade} rides {clearance_mm:.0f} mm above the belt — the 9 mm pen slips under")


@pytest.mark.parametrize("chute", ["chute_c", "chute_d"])
def test_chute_stays_out_of_the_camera_window(chute):
    # A chute inside the frame is segmented as the item and blinds perception —
    # the day-5 blocker that forced the blades downstream (docs/decisions.md).
    (x, _, _, _, _, _), (length, _, _) = model_pose_and_size(chute)
    assert x - length / 2 >= CAMERA_WINDOW_FAR_X
