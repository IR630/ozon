# -*- coding: utf-8 -*-
"""The head-count-under-noise probe: the ways it would lie quietly.

A probe that compares rigs fails silently in three specific ways — a rig that
secretly measures the top head alone, a "reproducible" report that moves between
runs, and a side head that carves a correct top measurement away. Each gets a
test by value here; the fusion itself is covered by tests/test_multiview.py.
"""
import numpy as np
import pytest

from scripts.probe_noise_heads import (
    RIGS,
    load_rig_frames,
    measure_with_rig,
    probe_rig,
)
from src.constants import CAMERA_SIDE_NEG_Y_POSE_M
from src.perception import BELT_DEPTH_M, measure_items


# Quarter-size frames: this suite runs measure_items dozens of times over noise
# draws, and the full 640x480 costs ~100 s of CI for geometry none of these tests
# assert on. The probe itself always runs on the dumped 640x480 frames.
_H, _W = 120, 160


def _top_frame_with_a_box(top=1.30):
    """Top depth frame: a box standing on the belt, seen from above."""
    depth = np.full((_H, _W), BELT_DEPTH_M, dtype=float)
    depth[50:70, 70:90] = top
    return depth


def _side_frame_seeing_nothing():
    """A side head whose every pixel is a dropped return — the honest empty view."""
    return np.zeros((_H, _W), dtype=float)


def test_a_rig_with_no_side_frames_measures_exactly_the_top_head():
    """The one-head row must BE the top-only measurement, bit for bit.

    If it drifted, every "the rig wins/loses N mm" number in the report would be
    measured against a baseline that is not the production single-head path.
    """
    depth = _top_frame_with_a_box()
    dims, bodies = measure_with_rig(depth, [], [])
    assert dims == list(measure_items(depth)[0].dims_mm)
    assert bodies == 1


def test_a_blind_side_head_degrades_to_the_top_measurement_instead_of_shrinking_it():
    """A head that returns nothing must cost nothing — this is the lost-head case
    that a rig has to survive on a live line, and the failure mode is silent: an
    empty cloud fused naively would report a smaller item, not an error."""
    depth = _top_frame_with_a_box()
    top_only, _ = measure_with_rig(depth, [], [])
    with_blind_head, _ = measure_with_rig(
        depth, [_side_frame_seeing_nothing()], [CAMERA_SIDE_NEG_Y_POSE_M])
    assert with_blind_head == top_only


def test_the_probe_is_reproducible_from_the_seed_alone():
    """Two runs of the same (seed, sigma) must produce identical numbers, or the
    report cannot be re-derived from the command in docs/experiments.md."""
    depth = _top_frame_with_a_box()
    truth = (208.0, 208.0, 200.0)
    args = ([], [], truth, 0.002, 4)
    first = probe_rig(depth, *args, np.random.default_rng([7, 1, 2000]))
    second = probe_rig(depth, *args, np.random.default_rng([7, 1, 2000]))
    assert first == second


def test_noise_is_actually_injected():
    """sigma > 0 must change the measurement — a probe that silently dropped the
    noise would report a flat, reassuring table at every sigma."""
    depth = _top_frame_with_a_box()
    truth = (208.0, 208.0, 200.0)
    rng = np.random.default_rng([0, 1, 0])
    _d, clean_errs, _p, _b = probe_rig(depth, [], [], truth, 0.0, 3, rng)
    _d, noisy_errs, _p, _b = probe_rig(depth, [], [], truth, 0.01, 3, rng)
    assert len(set(clean_errs)) == 1, "sigma=0 is the control row and must not vary"
    # Spread, not magnitude: noise is free to make one draw LOOK better (a wrong
    # box that happens to sit closer to truth), and asserting "error grows" would
    # encode that accident. What noise cannot do is leave every draw identical.
    assert len(set(noisy_errs)) == 3


def test_side_frames_are_not_loaded_as_top_frames(tmp_path):
    """`depth_side_neg_y_000.png` matches `depth_*.png`. Letting it into the top
    list would measure a side view with the top head's belt plane and quietly
    report a nonsense item."""
    from src.perception import save_depth_png

    save_depth_png(_top_frame_with_a_box(), str(tmp_path / "depth_000.png"))
    save_depth_png(_side_frame_seeing_nothing(), str(tmp_path / "depth_side_neg_y_000.png"))
    top, sides = load_rig_frames(tmp_path)
    assert top.max() == pytest.approx(BELT_DEPTH_M, abs=1e-3)
    assert set(sides) == {"depth_side_neg_y"}


def test_every_rig_keeps_the_top_head():
    """The top head owns detection, position and id in production; a rig defined
    without it here would be comparing a different system than we ship."""
    assert RIGS[0][1] == ()
    names = [name for _label, heads in RIGS for name, _pose in heads]
    assert names.count("depth_side_neg_y") == 2, "the 2- and 3-head rigs share the -y head"
