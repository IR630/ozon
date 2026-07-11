# -*- coding: utf-8 -*-
"""Offline STL->depth render path (tools/precision_sweep.py).

Locks the synthetic renderer + production perception on two LIGHT meshes so the
whole offline cross-check path stays honest without the simulator. Not a
threshold-tuning surface (docs/decisions.md): the asserts are loose, only
checking that the render feeds perception a faithful frame on known items.
Heavy meshes (helmet 55k, detergent 73k faces) are left to the CLI sweep.
"""
import sys
from pathlib import Path

import pytest

pytest.importorskip("trimesh")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from precision_sweep import render_item_depth  # noqa: E402

from src.classification import classify  # noqa: E402
from src.constants import CATEGORY_B, CATEGORY_D  # noqa: E402
from src.perception import measure_item  # noqa: E402


def test_render_box_300_recovers_dims_and_routes_b():
    # box_300x200x200 rendered flat under the camera must measure ~300x200x200
    # and route B — the same frame the real Gazebo depth of this box produces
    # (depth 1.3 m, docs/experiments.md).
    m = measure_item(render_item_depth("box_300x200x200"))
    assert m is not None
    for measured, truth in zip(m.dims_mm, [301.0, 200.0, 200.0]):
        assert abs(measured - truth) <= 10.0, f"{m.dims_mm} vs [301, 200, 200]"
    assert classify(m.dims_mm, m.k) == CATEGORY_B


def test_render_plate_is_round_and_routes_d():
    # Тарелка: a flat disc from above -> silhouette K > 0.8 -> D. Exercises the
    # round-item path of the offline render on a real STL, not a synthetic circle.
    m = measure_item(render_item_depth("plate"))
    assert m is not None
    assert m.k > 0.8, f"plate must read round: K={m.k}"
    assert classify(m.dims_mm, m.k) == CATEGORY_D
