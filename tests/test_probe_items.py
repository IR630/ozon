# -*- coding: utf-8 -*-
"""Procedural edge-case probes: sane models, and no NEW pipeline regressions.

The probes exist to attack the empirical gates in src/perception.py, which were
all calibrated on the organizers' 11 models (docs/probe-models.md). Three of them
currently miss; those are pinned as KNOWN GAPS rather than deleted, so the suite
fails the moment a fourth appears or a known one silently changes shape.
"""
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import trimesh

from build_probe_items import OUT_DIR, PROBES, main as build_all
from measure_probe_shapes import KNOWN_GAPS, evaluate, is_stable, quat_of


@pytest.fixture(scope="session", autouse=True)
def built_probes():
    if not OUT_DIR.exists() or len(list(OUT_DIR.iterdir())) < len(PROBES):
        build_all()


@pytest.fixture(scope="session")
def results(built_probes):
    return evaluate()


@pytest.mark.parametrize("slug", sorted(PROBES))
def test_model_is_sane(slug):
    """Same contract as the released items (tests/test_item_models.py)."""
    out = OUT_DIR / slug
    root = ET.parse(out / "model.sdf").getroot()
    assert root.find(".//model").get("name") == f"item_{slug}"
    assert float(root.find(".//inertial/mass").text) == PROBES[slug].mass_kg
    for axis in ("ixx", "iyy", "izz"):
        value = float(root.find(f".//inertia/{axis}").text)
        assert 0 < value < 1.0, f"{axis}={value}"

    hull = trimesh.load(str(out / "meshes" / "collision.stl"), force="mesh")
    assert hull.is_watertight
    assert len(hull.faces) < 1500, "collision hull too heavy for physics"
    assert np.all(hull.extents > 5) and np.all(hull.extents < 600)
    assert abs(hull.bounds[0][2]) < 1.0, "item must rest at z=0"


@pytest.mark.parametrize("slug", sorted(PROBES))
def test_built_mesh_matches_the_drawing(slug):
    """The generator produces the dimensions the truth table was derived from.

    If a shape drifts, its hand-derived expected category stops being the truth
    and every verdict below is meaningless — so this is checked before them.
    """
    probe = PROBES[slug]
    built = np.sort(probe.build().extents)[::-1]
    assert np.allclose(built, np.sort(probe.dims_mm)[::-1], atol=1.0)


@pytest.mark.parametrize("slug", sorted(PROBES))
def test_declared_poses_are_physically_stable(slug):
    """Every declared pose rests on the belt — see the Probe docstring.

    Without this the gate can 'fail' against a pose that tips over on contact,
    which is how an 8 mm sheet measured 21 mm thick and looked like a rule break.
    """
    probe = PROBES[slug]
    mesh = probe.build()
    for pose_name, axis, degrees in probe.poses:
        assert is_stable(mesh, quat_of(axis, degrees)), f"{slug}/{pose_name} tips over"


def test_every_probe_is_detected_as_exactly_one_item(results):
    """No probe splits into two items or vanishes.

    The concave u_bracket is the one at real risk (the touching-items splitter),
    and thin_sheet stands 8 mm above a 5 mm mask margin — an undetected item
    defaults to B and jams the main sorter, the expensive failure.
    """
    bad = [(r.slug, r.pose, r.actual) for r in results if r.n_detected != 1]
    assert not bad, f"detection is not 1 item per frame: {bad}"


def test_no_unexplained_misclassification(results):
    """The gate: every stable pose is either correct or a documented known gap."""
    failures = [
        (r.slug, r.pose, f"expected {r.expected}, got {r.actual}")
        for r in results
        if r.stable and r.actual != r.expected
        and KNOWN_GAPS.get((r.slug, r.pose), (None,))[0] != r.actual
    ]
    assert not failures, f"new misclassifications: {failures}"


def test_known_gaps_still_have_the_shape_we_documented(results):
    """A known gap must stay exactly as diagnosed — it is not a licence to drift.

    If one of these starts passing, the fix is to delete it from KNOWN_GAPS (and
    from docs/probe-models.md), not to leave a stale excuse in the code.
    """
    for result in results:
        key = (result.slug, result.pose)
        if result.stable and key in KNOWN_GAPS:
            assert result.actual == KNOWN_GAPS[key][0], (
                f"{result.slug}/{result.pose} changed: documented "
                f"{KNOWN_GAPS[key][0]}, now {result.actual}"
            )


def test_size_rule_holds_at_a_12mm_margin(results):
    """slab_over_limit is 332 mm on its second dimension: 12 mm over the bound.

    This is the tightest size call in either model set (Шлем sits 22 mm under it),
    so it is the one that says whether the 320 mm limit survives measurement error.
    """
    slab = [r for r in results if r.slug == "slab_over_limit"]
    assert slab, "slab_over_limit produced no results"
    for result in slab:
        assert result.actual == "C", f"{result.pose}: {result.dims_mm}"
