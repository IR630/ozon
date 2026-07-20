# -*- coding: utf-8 -*-
"""STL-path vs camera-path equivalence on the pose-ROBUST rigid models.

The full sweep (scripts/compare_mesh_vs_perception.py) is a diagnostic on identity
poses, where round/thin items read differently than in their settled census poses.
The honest, pose-invariant claim worth locking as a regression: on the rigid bodies
whose OBB does not depend on how they rest, the production depth pipeline recovers
the mesh dims within the organizers' tolerance AND routes to the same category as
the reference geometry. Round/thin/soft items are covered by the census (164/165)
and measure_validation instead.
"""
from pathlib import Path

import pytest

pytest.importorskip("cv2")
pytest.importorskip("trimesh")

from scripts.compare_mesh_vs_perception import compare_one  # noqa: E402

STL = Path(__file__).resolve().parents[1] / "docs" / "Stl"
POSE_ROBUST = [
    "Короб 300х200х200",
    "Короб 400х400х300",
    "ЛанчБокс",
    "Моющее средство",
    "Цилиндр",
]


@pytest.mark.parametrize("stem", POSE_ROBUST)
def test_rigid_model_agrees_within_tolerance(stem):
    r = compare_one(STL / f"{stem}.stl")
    assert r["detected"], stem
    assert r["in_tol"], (stem, r["side_err"], r["vol_err"])
    assert r["cat_match"], (stem, r["ref_cat"], r["perc_cat"])
