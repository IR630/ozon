# -*- coding: utf-8 -*-
"""The scored point must be on the GOODS, not on the model origin (scripts/body_pose.py).

Gazebo reports the model ORIGIN, and ours is the default pose's BOTTOM
(build_item_models.set_belt_origin). Gazebo rotates about it, so for a turned bulky
item that point leaves the body entirely — the same origin/body confusion that already
caused the spawn-height and spawn-centring bugs (docs/decisions.md 2026-07-12), here
hitting the third and last consumer: the episode verdict.
"""
import math

import pytest

from body_pose import body_offsets_m, body_pose_m, quat_from_rpy

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def _box(extents=(300, 200, 400)):
    trimesh = pytest.importorskip("trimesh")
    return trimesh.creation.box(extents=extents)


def test_identity_pose_puts_the_origin_on_the_bodys_bottom():
    # The model convention: origin = XY bbox centre, Z=0 at the lowest point. So in
    # the default pose the origin IS the contact point and the two agree.
    centre, lowest = body_offsets_m(_box(), IDENTITY)
    assert lowest == pytest.approx(0.0, abs=1e-9)
    assert centre == pytest.approx((0.0, 0.0, 0.2), abs=1e-9)  # half of 400 mm


def test_turning_the_item_moves_the_origin_off_the_body():
    # Rolled 90 deg about X, the 400 mm dimension lies down and the 200 mm one stands
    # up: the body now reaches 100 mm BELOW the origin, which is what the reported
    # pose silently omits.
    centre, lowest = body_offsets_m(_box(), quat_from_rpy(math.pi / 2, 0.0, 0.0))
    assert lowest == pytest.approx(-0.1, abs=1e-6)
    assert centre[2] == pytest.approx(0.0, abs=1e-6)


def test_body_pose_lifts_the_reported_origin_onto_the_goods():
    # An item resting ON the floor (lowest point z=0) in that rolled pose must report
    # an ORIGIN at z=+0.1 — and the body's centre at half its standing height.
    quat = quat_from_rpy(math.pi / 2, 0.0, 0.0)
    (_, _, cz), lowest_z = body_pose_m(_box(), quat, (3.0, 1.0, 0.1))
    assert lowest_z == pytest.approx(0.0, abs=1e-6)
    assert cz == pytest.approx(0.1, abs=1e-6)


def test_the_hull_path_agrees_with_the_mesh_path_on_a_real_item(tmp_path):
    """The poll loop scores the body off a precomputed convex HULL, not the mesh.

    That is a performance fix with a correctness obligation: importing trimesh costs
    1.33 s per call and the loop runs 60 times, which by itself pushed cells past their
    180 s timeout and faked `physics_wedge` failures. The hull must therefore answer
    EXACTLY what the mesh would — the extreme point in any direction lies on the convex
    hull, so it carries the whole bounding box — and this pins that on a real item in a
    turned pose, where the two could silently drift apart.
    """
    pytest.importorskip("trimesh")
    import trimesh

    from body_pose import dump_hull
    from build_item_models import ITEMS, STL_DIR
    from zone_verdict import body_from_hull, read_hull_m

    stem, _ = ITEMS["pouf"]
    if not (STL_DIR / f"{stem}.stl").exists():
        pytest.skip("organizer STL artifacts are not present")

    roll, pitch, yaw = 1.82, -0.44, -2.16   # the pouf's real resting pose in Gazebo
    origin = (3.67, 0.58, 0.288)

    quat = quat_from_rpy(roll, pitch, yaw)
    mesh = trimesh.load(str(STL_DIR / f"{stem}.stl"), force="mesh")
    (mcx, mcy, _), m_low = body_pose_m(mesh, quat, origin)

    hull_file = tmp_path / "hull.txt"
    dump_hull("pouf", str(hull_file))
    hcx, hcy, h_low = body_from_hull(read_hull_m(str(hull_file)), roll, pitch, yaw, origin)

    assert (hcx, hcy, h_low) == pytest.approx((mcx, mcy, m_low), abs=1e-6)
    assert h_low == pytest.approx(0.011, abs=0.005)  # resting on the cage floor


@pytest.mark.parametrize(("slug", "item_index", "orient_index", "expected_m"),
                         [("pouf", 6, 1, 0.268),     # the 'chute-stick' cell
                          ("pouf", 6, 2, 0.342),
                          ("helmet", 10, 2, 0.255)])
def test_a_turned_item_reports_an_origin_high_above_its_own_contact_point(
        slug, item_index, orient_index, expected_m):
    """How far the reported pose is from the goods, per failing cell — measured.

    These are the items the verdict's z<0.25 gate cannot score: lying PERFECTLY FLAT
    on the cage floor, each reports an origin ABOVE the gate. The gate was derived for
    a box whose origin happens to sit near its centre; it does not survive a turned
    489 mm pouf. Locked as numbers so a change of mesh or convention has to face them.
    """
    pytest.importorskip("trimesh")
    import trimesh

    from build_item_models import ITEMS, STL_DIR
    from spawn_orientations import orientation_quat

    stem, _ = ITEMS[slug]
    stl = STL_DIR / f"{stem}.stl"
    if not stl.exists():
        pytest.skip("organizer STL artifacts are not present")

    mesh = trimesh.load(str(stl), force="mesh")
    _, lowest = body_offsets_m(mesh, orientation_quat(0, item_index, orient_index))
    origin_z_resting_on_the_floor = -lowest
    assert origin_z_resting_on_the_floor == pytest.approx(expected_m, abs=0.005)
    assert origin_z_resting_on_the_floor > 0.25  # i.e. above the old gate: unpassable
