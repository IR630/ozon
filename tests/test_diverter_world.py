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
BELT_EDGE_Y = 0.35  # belt widened to 700 mm (day 6, see test_infeed_gap_admits...)
BELT_TOP_Z = 0.4
# Widest any test item gets in its seeded spawn orientation: box_400 at oi=1
# (568 mm — a 400x400 box turned in plan). Measured off the STL meshes, not
# guessed; the infeed must admit it or the item jams before the camera.
WIDEST_ITEM_M = 0.568


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


def infeed_rail():
    """(|y| of a rail's centre, its thickness) — belt_guides holds both rails as
    collisions inside one link, so the geometry lives on the collision, not the model."""
    root = ET.parse(WORLD).getroot()
    rails = root.find(".//model[@name='belt_guides']").findall(".//collision")
    poses = [[float(v) for v in c.find("pose").text.split()] for c in rails]
    sizes = [[float(v) for v in c.find("geometry/box/size").text.split()] for c in rails]
    return abs(poses[0][1]), sizes[0][1]


def blade_sweep_rad(name):
    """The blade's engaged angle — read from the joint limit, never duplicated."""
    root = ET.parse(WORLD).getroot()
    model = root.find(f".//model[@name='{name}']")
    return float(model.find(".//joint/axis/limit/upper").text)


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


@pytest.mark.parametrize("chute", ["chute_c", "chute_d"])
def test_chute_stays_out_of_the_camera_window(chute):
    # A chute inside the frame is segmented as the item and blinds perception —
    # the day-5 blocker that forced the blades downstream (docs/decisions.md).
    (x, _, _, _, _, _), (length, _, _) = model_pose_and_size(chute)
    assert x - length / 2 >= CAMERA_WINDOW_FAR_X


def test_infeed_gap_admits_the_widest_item():
    # The 500 mm belt with rails at +-0.272 left a 504 mm gap — narrower than the
    # biggest items in a turned pose, and EVERY box_400/pouf failure of the seed-0
    # census was one of those cells (568/544/523/528 mm wide): the item either
    # jammed before the camera or squeezed through askew and blinded perception.
    # The belt is now 700 mm and the gap 604 mm, with margin over the widest item.
    rail_y, thickness = infeed_rail()
    gap = 2 * (abs(rail_y) - thickness / 2)
    assert gap > WIDEST_ITEM_M, f"infeed gap {gap*1000:.0f} mm jams the widest item"
    # ...and the rails must still sit ON the belt, or round items roll off under them
    assert abs(rail_y) + thickness / 2 <= BELT_EDGE_Y


@pytest.mark.parametrize("blade", ["diverter_c", "diverter_d"])
def test_blade_still_spans_the_wider_belt(blade):
    # A blade that no longer reaches across the belt leaves a corridor the item
    # rides straight through (the wall must be a wall).
    (_, pivot_y, _, _, _, _), (length, _, _) = model_pose_and_size(blade)
    reach_across = length * math.sin(blade_sweep_rad(blade))  # lateral span engaged
    far_edge = abs(pivot_y) + BELT_EDGE_Y  # from the pivot to the opposite belt edge
    assert reach_across >= far_edge, (
        f"{blade} spans {reach_across:.2f} m, needs {far_edge:.2f} m to close the belt")
