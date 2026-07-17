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


@pytest.mark.parametrize(("chute", "zone"), [("chute_c", "zone_c"), ("chute_d", "zone_d")])
def test_the_zone_covers_the_chute_that_feeds_it(chute, zone):
    """The C patch used to end at x=3.6 while its chute delivered out to x=3.9.

    The chute dropped goods 30 cm past the end of the zone meant to catch them, so a
    big item (pouf) landed just outside the scored window — a defect invisible in a
    diff and invisible to every single-item episode that happened to land short.
    """
    (chute_x, _, _, _, _, _), (chute_len, _, _) = model_pose_and_size(chute)
    (zone_x, _, _, _, _, _), (zone_len, _, _) = model_pose_and_size(zone)

    chute_from, chute_to = chute_x - chute_len / 2, chute_x + chute_len / 2
    zone_from, zone_to = zone_x - zone_len / 2, zone_x + zone_len / 2

    assert zone_from <= chute_from, (
        f"{zone} starts at x={zone_from:.2f}, behind {chute} (x={chute_from:.2f})")
    assert zone_to >= chute_to, (
        f"{zone} ends at x={zone_to:.2f} but {chute} delivers out to x={chute_to:.2f} "
        "— it drops goods past the zone that should catch them")


@pytest.mark.parametrize("zone", ["zone_c", "zone_d"])
def test_the_cage_has_an_end_wall_to_stop_a_rolling_item(zone):
    """A roll-cage with one wall is not a cage: a bottle rolled straight out (x=4.35)."""
    root = ET.parse(WORLD).getroot()
    model = root.find(f".//model[@name='{zone}']")
    walls = [c.get("name") for c in model.iter("collision")]

    assert "end_wall" in walls, f"{zone} has no wall downstream — round goods roll out"


def named_collisions(model_name):
    """collision name -> (pose, box size) for a static multi-collision model."""
    root = ET.parse(WORLD).getroot()
    model = root.find(f".//model[@name='{model_name}']")
    assert model is not None, f"model {model_name} missing from {WORLD.name}"
    return model, {
        col.get("name"): ([float(v) for v in col.find("pose").text.split()],
                          [float(v) for v in col.find("geometry/box/size").text.split()])
        for col in model.iter("collision")
    }


def parked_blade_wall_start_x(blade):
    """Where the parked blade begins walling the belt edge (its upstream end)."""
    root = ET.parse(WORLD).getroot()
    model = root.find(f".//model[@name='{blade}']")
    pivot_x = float(model.find("pose").text.split()[0])
    col = model.find(".//collision")
    off_x = float(col.find("pose").text.split()[0])
    size_x = float(col.find("geometry/box/size").text.split()[0])
    return pivot_x + off_x - size_x / 2


def test_camera_guards_have_no_visual_because_the_camera_would_segment_them():
    """The camera-section guards are collision-only ON PURPOSE — do not "fix" this.

    They exist to catch the tilted-rest helmet, which rolls off the moving belt at
    x~2.1 (deterministic: two e2e runs, same final pose to 6 digits) — INSIDE the
    camera window, where any visible geometry is segmented as the item (the day-5
    lesson that pushed the parked blades out to x>=2.4). Collision without visual is
    the depth-model analogue of a real cell's transparent polycarbonate guards.
    """
    model, guards = named_collisions("belt_guides_camera")

    assert model.find(".//visual") is None, (
        "belt_guides_camera grew a <visual> — inside the camera window it will be "
        "segmented as the item and blind perception")
    # and the guards genuinely overlap the camera window — the reason they must be clear
    (pose, size) = guards["left"]
    assert pose[0] - size[0] / 2 < CAMERA_WINDOW_FAR_X


@pytest.mark.parametrize(("side", "blade", "sign"),
                         [("left", "diverter_c", +1), ("right", "diverter_d", -1)])
def test_camera_guards_splice_infeed_rails_to_the_parked_blade(side, blade, sign):
    """No un-walled belt edge from the infeed to the diverters.

    The measured failure lived exactly in a gap like this: the infeed rails end at
    x=0.7, the parked blades wall the edges only from x=2.45/2.95, and the rocking
    helmet exited through the opening at x~2.1. The guard must start where the infeed
    rails stop, hand over to its blade with at most a few cm of splice, and sit on
    the belt edge tightly enough that nothing slips between guard and belt.
    """
    _, infeed = named_collisions("belt_guides")
    _, guards = named_collisions("belt_guides_camera")
    (ipose, isize) = infeed[side]
    (gpose, gsize) = guards[side]

    infeed_end = ipose[0] + isize[0] / 2
    guard_start, guard_end = gpose[0] - gsize[0] / 2, gpose[0] + gsize[0] / 2
    assert guard_start <= infeed_end + 1e-6, (
        f"{side} guard starts at x={guard_start:.2f}, leaving a gap after the infeed "
        f"rails (end x={infeed_end:.2f})")

    wall_x = parked_blade_wall_start_x(blade)
    assert guard_end < wall_x, f"{side} guard reaches into {blade}'s parked wall"
    assert wall_x - guard_end <= 0.05, (
        f"{side} guard ends {1000 * (wall_x - guard_end):.0f} mm short of {blade} — "
        "an item-sized opening in the wall")

    # same cross-section as the infeed rails: flush with the belt edge, same height
    assert sign * gpose[1] - gsize[1] / 2 == pytest.approx(BELT_EDGE_Y, abs=0.01)
    assert (gpose[2], gsize[2]) == (ipose[2], isize[2])


def test_collection_floors_do_not_stop_goods_at_the_chute_mouth():
    root = ET.parse(WORLD).getroot()

    for zone in ("zone_c", "zone_d"):
        floor = root.find(
            f".//model[@name='{zone}']/link/collision[@name='collision']"
        )
        assert floor is not None
        mu = floor.findtext("surface/friction/ode/mu")
        mu2 = floor.findtext("surface/friction/ode/mu2")
        assert mu is not None and float(mu) <= 0.1
        assert mu2 is not None and float(mu2) <= 0.1
