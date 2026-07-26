# -*- coding: utf-8 -*-
"""The head-views figure must agree with the claim it is drawn to make.

`docs/report/cameras.md` §1 uses the picture to say two things at a glance: the
side heads add height, and they add nothing to the pen. The second is the load
bearing one — it is why the fork does not turn on head count — so the pixel
counts behind it are asserted here instead of read off the image.

Skipped when the rig dumps are not in the checkout: `runs/` is a work dir and
gitignored, so CI never has them (same guard as tests/test_probe_depth_dropout.py).
"""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.perception import BELT_DEPTH_M, MASK_MARGIN_M, load_depth_png  # noqa: E402


def _dump(slug):
    d = ROOT / "runs" / "frames" / f"{slug}_3cam"
    if not d.is_dir():
        pytest.skip(f"{d} not in this checkout (runs/g_dump_3cam.sh)")
    return d


def _px_over_belt(slug):
    """Pixels the top head sees standing over the belt by more than the mask margin."""
    pytest.importorskip("cv2")
    depth = load_depth_png(str(_dump(slug) / "depth_000.png"))
    height_mm = (BELT_DEPTH_M - depth) * 1000.0
    return int(((depth > 0.0) & (height_mm > MASK_MARGIN_M * 1000.0)).sum())


def test_the_pen_is_a_sliver_from_above_and_the_others_are_not():
    """The figure's central claim: 9 mm of relief is two orders below a normal item.

    This is what makes the pen the item the C/B border turns on, and no side head
    changes it — the side heads are looked at separately below.
    """
    pen, bag, helmet = (_px_over_belt(s) for s in ("pen", "bag", "helmet"))
    assert pen < 500, f"pen should be a sliver, got {pen} px"
    assert bag > 2000 and helmet > 2000, f"bag {bag} px, helmet {helmet} px"
    assert pen * 10 < bag < helmet


def test_the_side_heads_do_deliver_frames_on_every_item():
    """Guards against a figure that draws blank panels and reads as 'sides see nothing'.

    The sides DO return depth on all three items — what they do not return is a
    pen separable from the belt. Conflating the two would be the wrong conclusion
    drawn from the right picture.
    """
    pytest.importorskip("cv2")
    for slug in ("helmet", "pen", "bag"):
        for fname in ("depth_side_neg_y_000.png", "depth_side_pos_y_000.png"):
            depth = load_depth_png(str(_dump(slug) / fname))
            assert (depth > 0.0).sum() > 100_000, f"{slug}/{fname} came back near-empty"


def test_figure_renders():
    pytest.importorskip("cv2")
    import tempfile

    from scripts.plot_head_views import main

    _dump("helmet")
    with tempfile.TemporaryDirectory() as d:
        out = Path(d) / "head_views.png"
        assert main([str(out)]) == 0
        assert out.stat().st_size > 5000


def test_the_height_scale_covers_the_tallest_item_drawn():
    """A fixed shared scale is only honest if nothing drawn clips against it."""
    from scripts.plot_head_views import HEIGHT_MAX_MM, ITEMS

    for _slug, dims in ITEMS:
        assert max(float(x) for x in dims.split("x")) <= HEIGHT_MAX_MM
    assert np.isfinite(HEIGHT_MAX_MM)


def test_no_head_body_appears_in_any_side_frame():
    """The figure's caption once said a camera housing was visible. It is not.

    Two independent facts, and either one alone settles it: the camera models in
    the rig world carry a `<sensor>` and no `<visual>`/`<collision>` at all, and
    the reconstructed side clouds hold ZERO points at the opposite head's own
    |y|. The pale slab that was mistaken for a housing is the zone C roll cage —
    model at (3.2, 0.9), wall offset (0, 0.6, 0.4), hence world y = 1.5.

    Locked as a test because a caption is prose and prose drifts back: the
    "housing in frame" failure mode (`cameras.md` §4) is a GEOMETRIC argument
    from a datasheet housing, and calling it photographed overstates our evidence
    to a jury.
    """
    pytest.importorskip("cv2")
    from src.constants import CAMERA_SIDE_NEG_Y_POSE_M, CAMERA_SIDE_POS_Y_POSE_M
    from src.multiview import world_cloud_from_depth
    from src.perception import FX, FY

    world = ROOT / "sim" / "worlds" / "cell_diverter_3cam.sdf"
    text = world.read_text(encoding="utf-8")
    for name in ("camera", "camera_side_neg_y", "camera_side_pos_y"):
        block = text.split(f'<model name="{name}">', 1)[1].split("</model>", 1)[0]
        assert "<visual" not in block, f"{name} grew a visual — re-check the caption"
        assert "<collision" not in block, f"{name} grew a collision"

    for fname, pose in (("depth_side_neg_y_000.png", CAMERA_SIDE_NEG_Y_POSE_M),
                        ("depth_side_pos_y_000.png", CAMERA_SIDE_POS_Y_POSE_M)):
        pts = world_cloud_from_depth(
            load_depth_png(str(_dump("helmet") / fname)), pose, FX, FY)
        at_head_y = (np.abs(pts[:, 1]) >= 0.80) & (np.abs(pts[:, 1]) <= 1.00)
        assert int(at_head_y.sum()) == 0, (
            f"{fname}: {int(at_head_y.sum())} points where a head stands")
