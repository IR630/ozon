# -*- coding: utf-8 -*-
"""The DOMAIN GAP: does a rendered depth frame measure like a real Gazebo one?

PLAN.md's rule is that synthetic data may not set a threshold until this gap is measured.
So it is measured here, on real Gazebo frames saved together with the item's RESTING pose
(scripts/dump_item_frame.sh) — without that pose a saved frame is a picture of an unknown
orientation, because the item settles before the camera sees it: the plate spawned on edge
comes to rest flat, and comparing a rendered spawn pose against it "finds" a 31 mm gap
that is nothing but the physics doing its job.

Measured at the SAME pose the gap is 3 mm and 0.01 of K — inside perception's own +-10 mm
accuracy — and every category agrees. That is what licenses the offline pose sweep.
"""
from pathlib import Path

import pytest

FIXTURES = Path(__file__).resolve().parent / "fixtures" / "frames"
CELLS = ["plate_1", "helmet_2", "bag_2", "cylinder_0"]

# The gap the renderer is allowed. Perception itself claims +-10 mm against ground truth
# (docs/experiments.md), so a renderer inside that is not the weakest link in the chain.
MAX_DIM_GAP_MM = 10.0
MAX_K_GAP = 0.05


@pytest.mark.parametrize("cell", CELLS)
def test_a_rendered_frame_measures_like_the_real_one(cell):
    pytest.importorskip("trimesh")
    cv2 = pytest.importorskip("cv2")
    import numpy as np

    from build_item_models import ITEMS, STL_DIR
    from render_depth import load_mesh, read_resting_quat, render_depth

    from src.classification import classify_conservative
    from src.perception import measure_item

    slug = cell.rsplit("_", 1)[0]
    stem, _ = ITEMS[slug]
    if not (STL_DIR / f"{stem}.stl").exists():
        pytest.skip("organizer STL artifacts are not present")

    real = cv2.imread(str(FIXTURES / f"{cell}_depth.png"), cv2.IMREAD_UNCHANGED)
    real_m = np.asarray(real, dtype=float) / 1000.0
    quat = read_resting_quat(FIXTURES / f"{cell}_pose.txt")

    got_real = measure_item(real_m)
    got_syn = measure_item(render_depth(load_mesh(slug), quat))
    assert got_real is not None and got_syn is not None

    dims_real = sorted(got_real.dims_mm, reverse=True)
    dims_syn = sorted(got_syn.dims_mm, reverse=True)
    for a, b in zip(dims_real, dims_syn):
        assert abs(a - b) <= MAX_DIM_GAP_MM, f"{cell}: {dims_real} vs {dims_syn}"
    assert abs(got_real.k - got_syn.k) <= MAX_K_GAP
    assert (classify_conservative(got_real.dims_mm, got_real.k)
            == classify_conservative(got_syn.dims_mm, got_syn.k))
