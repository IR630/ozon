# -*- coding: utf-8 -*-
"""Only poses an item can actually HOLD may be used to grade the rule.

The sweep's whole worth is the population it draws from. Graded on uniformly random
rotations it reported Бутылка at 6/40 and Короб 300 at 16/40 — failures on orientations
that exist only in the sampler: a box balanced on a corner projects a 380 mm silhouette,
and the simulator tips it onto a face long before the camera sees it. Those numbers would
have sent the team hunting phantom CV bugs. Graded on STABLE rests, nine of eleven items
are 100% correct.

The remaining bias runs the other way and is also measured: a hull facet can be statically
stable yet never SETTLED into — the bottle's exactly-horizontal rest (the sweep's headline
"break", K=0.00 -> B) is one Gazebo always tips out of, onto the hull's neck cone (~8.5
deg), where the section K reads 1.00 -> D (tests/fixtures/frames/bottle_side, census oi=1/2).
So the sweep over-reports, never under-reports: its misses are leads to verify in Gazebo.
"""
import numpy as np
import pytest

from pose_sweep import stable_rests


def test_a_box_rests_on_its_six_faces_and_nothing_else():
    trimesh = pytest.importorskip("trimesh")

    rests = stable_rests(trimesh.creation.box(extents=(300, 200, 100)))

    assert len(rests) == 6
    assert sum(w for _, w in rests) == pytest.approx(1.0)


def test_a_flat_plate_rests_overwhelmingly_on_its_two_faces():
    trimesh = pytest.importorskip("trimesh")

    rests = stable_rests(trimesh.creation.box(extents=(200, 200, 10)))
    big = sorted((w for _, w in rests), reverse=True)

    # the two large faces carry almost all of the resting weight; the four edges are slivers
    assert big[0] + big[1] > 0.9


def test_a_face_the_item_topples_off_is_not_a_rest():
    """The criterion is the centre of mass, not the face's existence.

    A tall thin pillar has four narrow side faces. It CAN be laid on them (the CoM projects
    inside), but a cone cannot rest on its apex — its CoM falls outside that face, so the
    face must not appear as a rest. This is what keeps the sweep physical.
    """
    trimesh = pytest.importorskip("trimesh")

    cone = trimesh.creation.cone(radius=100, height=300)
    rests = stable_rests(cone)

    # every rest must put the cone's centre of mass over its support: the apex never does
    up = np.array([0.0, 0.0, 1.0])
    for transform, _ in rests:
        # the face turned downward must be the base, i.e. the cone's axis points up-ish
        axis = transform[:3, :3] @ up
        assert axis[2] > 0.5, "the cone came to rest on its apex"
