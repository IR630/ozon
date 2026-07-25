# -*- coding: utf-8 -*-
"""The depth-dropout probe: the ways it would lie quietly.

A probe that punches holes in a frame fails silently in four specific ways — a hole
it never actually punched (a reassuring flat table), a hole of the wrong size, a
"reproducible" report that moves between runs, and the three outcomes collapsing
into one so that a LOST item is reported as merely mismeasured. Each gets a test by
value here; the measurement and the fusion themselves are covered by
tests/test_perception.py and tests/test_multiview.py.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from probe_depth_dropout import (  # noqa: E402
    drop_item_returns,
    load_rig_frames,
    measure_under_dropout,
    probe_dropout,
    slug_of_dump,
)

from src.constants import CAMERA_SIDE_NEG_Y_POSE_M  # noqa: E402
from src.perception import BELT_DEPTH_M, MASK_MARGIN_M, _item_mask  # noqa: E402

# Quarter-size frames: this suite runs measure_items over many draws, and the full
# 640x480 costs minutes of CI for geometry none of these tests assert on. The probe
# itself always runs on the dumped 640x480 frames.
_H, _W = 120, 160


def _top_frame_with_a_box(top=1.30):
    """Top depth frame: a box standing on the belt, seen from above."""
    depth = np.full((_H, _W), BELT_DEPTH_M, dtype=float)
    depth[50:70, 70:90] = top
    return depth


def _mask_of(depth):
    return _item_mask(depth, BELT_DEPTH_M, MASK_MARGIN_M)


@pytest.mark.parametrize("mode", ["blob", "speckle"])
def test_zero_dropout_leaves_the_frame_untouched(mode):
    """The 0 % row is the control: it must reproduce the untouched measurement."""
    depth = _top_frame_with_a_box()
    holed, dropped = drop_item_returns(depth, 0.0, mode, np.random.default_rng(0))
    assert np.array_equal(holed, depth)
    assert dropped == 0.0


@pytest.mark.parametrize("mode", ["blob", "speckle"])
def test_full_dropout_leaves_no_valid_return_on_the_item(mode):
    """At 100 % the item must be GONE from the depth data, not merely dented — this is
    the black-wrap / transparent-film limit the rig argument rests on."""
    depth = _top_frame_with_a_box()
    mask = _mask_of(depth)
    holed, dropped = drop_item_returns(depth, 1.0, mode, np.random.default_rng(0))
    assert not _mask_of(holed).any()
    assert np.all(holed[mask] == 0.0)
    assert dropped == pytest.approx(1.0)
    # and the belt is untouched: a highlight sits on the product, not on the belt
    assert np.array_equal(holed[~mask], depth[~mask])


@pytest.mark.parametrize("mode", ["blob", "speckle"])
@pytest.mark.parametrize("frac", [0.1, 0.3, 0.5])
def test_the_requested_fraction_is_the_fraction_actually_dropped(mode, frac):
    """A probe whose 50 % row really drops 5 % would report the wrong threshold.

    The blob is exact by construction (nearest-N pixels); the speckle is binomial,
    so it is checked within the sampling spread of a 400 px mask.
    """
    depth = _top_frame_with_a_box()
    mask_px = int(_mask_of(depth).sum())
    holed, dropped = drop_item_returns(depth, frac, mode, np.random.default_rng(1))
    lost_px = int((_mask_of(depth) & (holed == 0.0)).sum())
    assert lost_px == round(dropped * mask_px)
    assert dropped == pytest.approx(frac, abs=0.05)


def test_the_blob_is_connected_and_the_speckle_is_not():
    """The two modes must differ in SHAPE, not just in name: one connected patch vs
    scattered pixels is exactly what segmentation reacts to differently."""
    from scipy.ndimage import label

    depth = _top_frame_with_a_box()
    rng = np.random.default_rng(2)
    blob, _ = drop_item_returns(depth, 0.3, "blob", rng)
    speckle, _ = drop_item_returns(depth, 0.3, "speckle", rng)
    mask = _mask_of(depth)
    structure = np.ones((3, 3), dtype=int)
    assert label(mask & (blob == 0.0), structure=structure)[1] == 1
    assert label(mask & (speckle == 0.0), structure=structure)[1] > 10


@pytest.mark.parametrize("mode", ["blob", "speckle"])
def test_the_same_seed_reproduces_the_same_hole_and_a_different_one_does_not(mode):
    """Without this the table in docs/experiments.md cannot be re-derived from its
    command, which is a jury criterion, not a nicety."""
    depth = _top_frame_with_a_box()
    same_a, _ = drop_item_returns(depth, 0.3, mode, np.random.default_rng(5))
    same_b, _ = drop_item_returns(depth, 0.3, mode, np.random.default_rng(5))
    other, _ = drop_item_returns(depth, 0.3, mode, np.random.default_rng(6))
    assert np.array_equal(same_a, same_b)
    assert not np.array_equal(same_a, other)


@pytest.mark.parametrize("dir_name,slug", [
    ("bag_3cam", "bag"), ("helmet_3cam", "helmet"), ("pen_3cam", "pen"),
    ("bottle_oi0", "bottle"), ("helmet_oi2_node", "helmet"),
])
def test_rig_dump_names_map_to_catalogue_items(dir_name, slug):
    """The rig dumps are named `<slug>_3cam`, which the older probes' slug parser does
    not peel — scoring them against the wrong truth would be silent."""
    assert slug_of_dump(dir_name) == slug


def test_an_unknown_dump_dir_is_loud_instead_of_scoring_against_the_wrong_truth():
    with pytest.raises(ValueError):
        slug_of_dump("mystery_3cam")


def test_an_unknown_mode_is_loud_instead_of_silently_dropping_nothing():
    """A typo in --modes must not produce a clean table measured on untouched frames."""
    with pytest.raises(ValueError):
        drop_item_returns(_top_frame_with_a_box(), 0.3, "blobb", np.random.default_rng(0))


def test_a_lost_item_is_a_distinct_outcome_from_a_mismeasured_one():
    """The whole point of the probe: "not found" and "found but wrong" must never be
    tallied together. A total dropout gives the first, a heavy blob the second."""
    depth = _top_frame_with_a_box()
    truth = (57.0, 57.0, 200.0)     # deliberately wrong truth: every measurement is out
    gone = probe_dropout(depth, [], [], truth, 1.0, "blob", 2, np.random.default_rng(0))
    assert (gone.lost, gone.in_tol, gone.out_tol) == (2, 0, 0)
    assert gone.side_errs == ()
    kept = probe_dropout(depth, [], [], truth, 0.0, "blob", 2, np.random.default_rng(0))
    assert (kept.lost, kept.in_tol, kept.out_tol) == (0, 0, 2)
    assert len(kept.side_errs) == 2


def test_a_lost_item_reports_no_dims_at_all_rather_than_a_fused_guess():
    """`measure_under_dropout` must return None for BOTH heads when the top head found
    nothing — the side clouds are cropped to a measurement that does not exist
    (src/perception_node.py:96-115), so any dims here would be invented."""
    depth = _top_frame_with_a_box()
    top_dims, fused_dims, bodies, dropped = measure_under_dropout(
        depth, [depth], [CAMERA_SIDE_NEG_Y_POSE_M], 1.0, "blob", np.random.default_rng(0))
    assert (top_dims, fused_dims, bodies) == (None, None, 0)
    assert dropped == pytest.approx(1.0)


def test_the_probe_is_reproducible_from_the_seed_alone():
    depth = _top_frame_with_a_box()
    truth = (57.0, 57.0, 200.0)
    args = ([], [], truth, 0.3, "speckle", 3)
    assert (probe_dropout(depth, *args, np.random.default_rng([7, 1, 300000]))
            == probe_dropout(depth, *args, np.random.default_rng([7, 1, 300000])))


def test_side_frames_are_not_loaded_as_top_frames(tmp_path):
    """`depth_side_neg_y_000.png` matches `depth_*.png`. Letting it into the top list
    would measure a side view against the top head's belt plane and report nonsense —
    and this probe re-uses that loader, so it inherits the trap."""
    pytest.importorskip("cv2")
    from src.perception import save_depth_png

    save_depth_png(_top_frame_with_a_box(), str(tmp_path / "depth_000.png"))
    save_depth_png(np.zeros((_H, _W)), str(tmp_path / "depth_side_neg_y_000.png"))
    top, sides = load_rig_frames(tmp_path)
    assert top.max() == pytest.approx(BELT_DEPTH_M, abs=1e-3)
    assert set(sides) == {"depth_side_neg_y"}


def test_it_runs_on_a_real_single_head_dump_and_says_there_are_no_side_heads(capsys):
    """The shipped dumps carry no side frames; that is the probe's NORMAL mode, and a
    row must still be produced (skipped when the dump is not in the checkout)."""
    pytest.importorskip("cv2")
    from probe_depth_dropout import main

    dump = ROOT / "runs" / "frames" / "bottle_oi0"
    if not dump.is_dir():
        pytest.skip("runs/frames/bottle_oi0 not in this checkout")
    assert main([str(dump), "--trials", "1", "--fracs", "0", "--modes", "blob"]) == 0
    out = capsys.readouterr().out
    assert "no side heads" in out
    assert "bottle_oi0 / blob" in out
