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
# bottle_side is the only cell where K comes from the VERTICAL-SECTION path (a lying body
# of revolution): the first four all decide K by silhouette, so without it the sweep's
# licence covered one of perception's two K paths and silently claimed both. On it the
# pose sweep reported B where two censuses measured D — suspicion fell on the renderer,
# and the frame acquits it: at the SAME resting pose (tilted ~8.5 deg — the hull's neck
# cone props the bottle, it never rests exactly horizontal) render and Gazebo agree to
# 5 mm / dK=0.00. The sweep's miss was its POSE population, not the pixels.
CELLS = ["plate_1", "helmet_2", "bag_2", "cylinder_0", "bottle_side"]
# helmet_tilt pins the sweep's one CONFIRMED break: the helmet settles on a tilted hull
# facet (~2% of its resting weight) and the top-view OBB reads 373x336 mm against the
# item's intrinsic 352x298x282 — a B item measured as C, on the REAL frame and the render
# alike (that agreement is what this test asserts; the misclassification itself is an open
# risk, docs/experiments.md 2026-07-14).
CELLS.append("helmet_tilt")

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


@pytest.mark.parametrize(
    ("slug", "quat", "expected_dims"),
    [
        (
            "box_400x400x300",
            (0.699874976, 0.108176654, -0.697740574, 0.107846749),
            [402.32, 401.14, 303.08],
        ),
        (
            "box_400x400x300",
            (-0.701157299, -0.099526944, -0.699018986, 0.099223418),
            [402.49, 401.14, 303.31],
        ),
        (
            "pouf",
            (-0.315538130, -0.632799880, 0.632799880, 0.315538130),
            [482.23, 475.58, 256.12],
        ),
    ],
)
def test_one_irregular_item_is_not_split_into_phantom_products(slug, quat, expected_dims):
    """Regression from the full stable-support sweep (seed 0).

    Two Box400 supports and one Pouf have respectively 3, 2, and 10 prominent
    EDT peaks. Treating every peak as a touching item measured only the largest
    fragment and changed both true C products to B.
    """
    pytest.importorskip("trimesh")

    from build_item_models import ITEMS, STL_DIR
    from render_depth import load_mesh, render_depth

    from src.classification import classify_conservative
    from src.perception import measure_items

    stem, _ = ITEMS[slug]
    if not (STL_DIR / f"{stem}.stl").exists():
        pytest.skip("organizer STL artifacts are not present")

    items = measure_items(render_depth(load_mesh(slug), quat))

    assert len(items) == 1
    assert items[0].dims_mm == pytest.approx(expected_dims, abs=10.0)
    assert classify_conservative(items[0].dims_mm, items[0].k) == "C"
