# -*- coding: utf-8 -*-
"""Episode verdict bands — the jury-visible definition of a correct run.

The bands are scored against the GOODS. Gazebo reports the model ORIGIN, which is the
default pose's BOTTOM and which the simulator rotates the body about, so for a turned
bulky item the two are up to 349 mm apart (scripts/zone_verdict.py). `body` carries the
item's real centre and lowest point; it defaults to the reported pose, which is exactly
right in the default pose — there the origin IS the contact point.
"""
import pytest

from zone_verdict import in_zone


def test_b_is_the_item_still_on_the_belt_past_the_mechanisms():
    assert in_zone("B", 4.5, 0.0, 0.45)
    assert not in_zone("B", 4.0, 0.0, 0.45)   # not past the D blade's reach (4.15) yet
    assert not in_zone("B", 4.5, 0.0, 0.10)   # on the floor: it was diverted


def test_b_is_not_awarded_to_an_item_merely_passing_through_on_its_way_to_a_cage():
    """The false PASS the old x>=3.5 band handed out, from a real census log.

    The D blade pivots at x=3.75, so EVERY D-bound item crosses x=3.5 at belt height
    on its way into it. The poll loop stops at the first YES — so bag oi=1, misrouted
    into D by the K-straddle, was scored PASS for B in census #1 while it ended up on
    the floor of the wrong cage (final pose y=-1.35 z=0.08). The band now starts past
    the last mechanism's reach, where no diverter can still take the item away.
    """
    assert not in_zone("B", 3.6, 0.0, 0.45)                # mid-belt, blade D still ahead
    assert not in_zone("B", 3.94, -1.349, 0.082)           # bag oi=1's real final pose
    # ...and an item sliding off the belt at belt height is not "on the belt" either.
    assert not in_zone("B", 4.3, 0.45, 0.40)


def test_c_and_d_are_the_roll_cages_on_the_floor():
    assert in_zone("C", 3.0, 0.9, 0.02)
    assert in_zone("D", 3.5, -0.9, 0.02)
    assert not in_zone("C", 3.0, -0.9, 0.02)  # right cage, wrong side
    assert not in_zone("D", 3.5, 0.9, 0.02)


def test_the_c_and_d_bands_reach_their_cage_wall_not_just_the_patch():
    """A large item (pouf) diverted into C settles AGAINST the cage wall (y=1.5),
    its CENTRE at y~1.43 — past the flat patch edge (1.3) but contained by the cage.

    The band must follow the CONTAINER (the walls), not the decorative patch, or a
    correctly-sorted big item FAILs by centimetres (docs/experiments.md 2026-07-13).
    Past the wall (|y|>1.5) is a genuine escape and must still fail.
    """
    assert in_zone("C", 3.07, 1.433, 0.14, body=(3.07, 1.433, 0.0))
    assert in_zone("D", 3.5, -1.433, 0.14, body=(3.5, -1.433, 0.0))
    assert not in_zone("C", 3.0, 1.55, 0.05, body=(3.0, 1.55, 0.0))   # past the wall
    assert not in_zone("D", 3.5, -1.55, 0.05, body=(3.5, -1.55, 0.0))


def test_an_item_still_on_the_belt_is_in_neither_cage():
    """Belt height (lowest point z~0.4) fails C and D — a diverted item must reach the floor."""
    assert not in_zone("C", 3.0, 0.9, 0.45)
    assert not in_zone("D", 3.5, -0.9, 0.45)


def test_an_item_hung_up_on_the_chute_has_not_been_delivered():
    """The failure the floor gate exists to catch: the item never left the ramp.

    The chute drops from the belt edge (y=0.25, z=0.34) to the cage floor (y=0.75,
    z=0). An item that stalls half way rests ~100-160 mm up it — inside the cage's
    x/y footprint, but NOT in the container: a human still has to unstick it. Scored
    on the BODY's lowest point, so it is one rule for a pouf and for a pen.
    """
    assert not in_zone("C", 3.6, 0.53, 0.32, body=(3.37, 0.53, 0.14))
    assert not in_zone("D", 3.6, -0.53, 0.32, body=(3.37, -0.53, 0.14))
    # ...and the same item, once it slides the rest of the way down, passes.
    assert in_zone("C", 3.6, 1.0, 0.27, body=(3.37, 1.0, 0.0))


def test_the_default_pose_needs_no_mesh_the_origin_is_the_contact_point():
    """The fallback contract (CI, the pusher world, run_stream): at the identity pose
    the model origin IS the body's lowest point, so scoring the reported pose is exact.
    A clean checkout has no STLs to measure, and must stay correct anyway."""
    assert in_zone("C", 3.0, 0.9, 0.0)        # box resting on the cage floor
    assert not in_zone("C", 3.0, 0.9, 0.14)   # same box hung up on the chute


def test_a_turned_pouf_lying_in_the_cage_is_scored_delivered():
    """The bug, end to end: pouf oi=1 lying FLAT on the cage floor.

    Its origin then reports z=0.268 — above the old z<0.25 gate — so census #1 and #3
    scored a correct delivery as a 'chute-stick' failure. Measured off the real mesh
    and the resting orientation, its lowest point is on the floor and it passes.
    """
    pytest.importorskip("trimesh")
    import trimesh

    from body_pose import body_pose_m
    from build_item_models import ITEMS, STL_DIR
    from spawn_orientations import orientation_quat

    stem, _ = ITEMS["pouf"]
    stl = STL_DIR / f"{stem}.stl"
    if not stl.exists():
        pytest.skip("organizer STL artifacts are not present")

    quat = orientation_quat(0, 6, 1)  # pouf is item index 6; oi=1 is the failing cell
    origin = (3.3, 1.0, 0.268)        # lying flat on the cage floor in that pose
    (cx, cy, _), lowest_z = body_pose_m(trimesh.load(str(stl), force="mesh"), quat, origin)

    assert not in_zone("C", *origin)                       # the old ruler: FAIL
    assert in_zone("C", *origin, body=(cx, cy, lowest_z))  # the goods: delivered
