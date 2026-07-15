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
The stable-support population over-reports physically settled poses, while the bounded
weighted sample may miss an individually rare support. Its misses are leads to verify in
Gazebo, and a clean sweep is evidence over the printed sample, not a proof over every face.
"""
import numpy as np
import pytest

from pose_sweep import MAX_REST_SAMPLES, presentable_poses, stable_rests


def test_a_box_rests_on_its_six_faces_and_nothing_else():
    trimesh = pytest.importorskip("trimesh")

    rests = stable_rests(trimesh.creation.box(extents=(300, 200, 100)))

    assert len(rests) == 6
    assert sum(w for _, w in rests) == pytest.approx(1.0)


def test_single_triangle_hull_faces_are_stable_rest_candidates():
    """A regular icosahedron can rest on each of its 20 triangular faces.

    Trimesh exposes coplanar multi-face groups through ``hull.facets`` but may
    leave every singleton triangle out of that collection.  The support census
    must cover the convex hull itself, not only the subset grouped as facets.
    """
    trimesh = pytest.importorskip("trimesh")
    mesh = trimesh.creation.icosphere(subdivisions=0, radius=100.0)

    rests = stable_rests(mesh)

    assert len(mesh.convex_hull.faces) == 20
    assert len(rests) == 20
    assert [weight for _, weight in rests] == pytest.approx([1.0 / 20.0] * 20)


def test_many_small_rests_are_weighted_and_bounded_deterministically():
    """Fine collision meshes keep their full mass without exploding render count."""
    trimesh = pytest.importorskip("trimesh")
    mesh = trimesh.creation.icosphere(subdivisions=2, radius=100.0)

    first = presentable_poses(mesh, yaws=2, seed=7)
    second = presentable_poses(mesh, yaws=2, seed=7)

    assert len(first) == MAX_REST_SAMPLES * 2
    assert sum(weight for _, weight in first) == pytest.approx(1.0)
    assert first == second


def test_a_flat_plate_rests_overwhelmingly_on_its_two_faces():
    trimesh = pytest.importorskip("trimesh")

    rests = stable_rests(trimesh.creation.box(extents=(200, 200, 10)))
    big = sorted((w for _, w in rests), reverse=True)

    # the two large faces carry almost all of the resting weight; the four edges are slivers
    assert big[0] + big[1] > 0.9


def test_a_face_the_item_topples_off_is_not_a_rest():
    """The criterion is the centre of mass, not the face's existence.

    A cube with a long +x spike keeps its original +y face on the convex hull, but the
    spike moves the CoM projection beyond that face's +x edge. The body topples toward
    the spike instead of resting there, while its opposite -x face remains stable.
    """
    trimesh = pytest.importorskip("trimesh")
    vertices = np.vstack([trimesh.creation.box().vertices, [10.0, 0.0, 0.0]])
    mesh = trimesh.convex.convex_hull(vertices)

    rests = stable_rests(mesh)
    down = np.array([0.0, 0.0, -1.0])
    support_normals = [transform[:3, :3].T @ down for transform, _ in rests]

    assert mesh.center_mass[0] > 0.5  # beyond the retained +y face's x extent
    assert not any(np.allclose(normal, [0.0, 1.0, 0.0]) for normal in support_normals)
    assert any(np.allclose(normal, [-1.0, 0.0, 0.0]) for normal in support_normals)
