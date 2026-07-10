# -*- coding: utf-8 -*-
"""Generated SDF item models are physically sane (Karpathy principle 6).

Models are generated (gitignored); the fixture builds them if missing,
so the test verifies the generator, not a stale artifact.
"""
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import trimesh

from build_item_models import ITEMS, OUT_DIR, main as build_all


@pytest.fixture(scope="session", autouse=True)
def built_models():
    if not OUT_DIR.exists() or len(list(OUT_DIR.iterdir())) < len(ITEMS):
        build_all()


@pytest.mark.parametrize("slug", sorted(ITEMS))
def test_model_is_sane(slug):
    out = OUT_DIR / slug
    root = ET.parse(out / "model.sdf").getroot()  # valid XML
    assert root.find(".//model").get("name") == f"item_{slug}"

    mass = float(root.find(".//inertial/mass").text)
    assert mass == ITEMS[slug][1]

    # inertia: positive diagonal, physically bounded (item <= 0.5 m, <= 3 kg)
    for axis in ("ixx", "iyy", "izz"):
        val = float(root.find(f".//inertia/{axis}").text)
        assert 0 < val < 1.0, f"{axis}={val}"

    hull = trimesh.load(str(out / "meshes" / "collision.stl"), force="mesh")
    assert hull.is_watertight
    assert len(hull.faces) < 1500, "collision hull too heavy for physics"
    # extents in physical range (mm), item rests at z=0
    assert np.all(hull.extents > 5) and np.all(hull.extents < 600)
    assert abs(hull.bounds[0][2]) < 1.0


def test_all_eleven_items_present():
    assert len(ITEMS) == 11
    assert sorted(p.name for p in OUT_DIR.iterdir()) == sorted(ITEMS)
