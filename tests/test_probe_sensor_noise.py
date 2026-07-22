# -*- coding: utf-8 -*-
"""The noise probe must be reproducible and must not invent depth where there is none."""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from probe_sensor_noise import add_depth_noise, probe_frame, slug_of_dir  # noqa: E402


def test_the_same_seed_reproduces_the_same_noise():
    depth = np.full((8, 8), 1.5)
    a = add_depth_noise(depth, 0.01, np.random.default_rng(3))
    b = add_depth_noise(depth, 0.01, np.random.default_rng(3))
    assert np.array_equal(a, b)


def test_a_different_seed_gives_different_noise():
    depth = np.full((8, 8), 1.5)
    a = add_depth_noise(depth, 0.01, np.random.default_rng(3))
    b = add_depth_noise(depth, 0.01, np.random.default_rng(4))
    assert not np.array_equal(a, b)


def test_zero_sigma_leaves_the_frame_untouched():
    """The sigma=0 row is the control: it must reproduce the noise-free measurement."""
    depth = np.array([[1.5, 1.4], [0.0, 1.6]])
    assert np.array_equal(add_depth_noise(depth, 0.0, np.random.default_rng(0)), depth)


def test_pixels_without_a_return_stay_without_a_return():
    """0 means "the sensor saw nothing"; noise must not turn it into a phantom surface."""
    depth = np.zeros((16, 16))
    depth[4:12, 4:12] = 1.5
    noisy = add_depth_noise(depth, 0.02, np.random.default_rng(0))
    assert np.all(noisy[depth == 0.0] == 0.0)
    assert np.all(noisy[depth > 0.0] > 0.0)


def test_noise_scales_with_range_not_with_a_flat_metre():
    """A sensor's error grows with distance — that is why sigma is a FRACTION."""
    rng = np.random.default_rng(0)
    near = add_depth_noise(np.full((200, 200), 0.5), 0.02, rng)
    far = add_depth_noise(np.full((200, 200), 4.0), 0.02, rng)
    assert np.std(far) > 5 * np.std(near)
    # and the magnitude is the datasheet's, not something an order off
    assert 0.06 < np.std(far) < 0.10       # 2 % of 4 m = 0.08 m


def test_an_undetected_item_is_counted_as_lost_not_skipped():
    """Losing the item IS the failure mode; a probe that skips it flatters high sigma."""
    blind = np.zeros((240, 320))          # nothing came back at all
    detected, side, vol, passes = probe_frame(blind, (100.0, 100.0, 100.0), 0.01, 4,
                                              np.random.default_rng(0))
    assert (detected, side, vol, passes) == (0, [], [], 0)


def test_a_bare_belt_is_empty_without_noise_and_grows_phantoms_with_it():
    """A finding, not a fixture: depth noise segments items out of the BELT itself.

    At sigma=0 the bare belt yields nothing, as it must. At 1 % of range the same
    belt starts returning bodies — the 5 mm segmentation margin is thinner than
    the noise (1 % of 1.5 m = 15 mm). On the real line that is a false detection,
    and nothing in our contour rejects it today (see docs/report/path_to_line.md,
    row "самодиагностика").
    """
    belt = np.full((240, 320), 1.5)       # BELT_DEPTH_M: camera 1.9 - belt top 0.4
    clean, _, _, _ = probe_frame(belt, (100.0, 100.0, 100.0), 0.0, 2,
                                 np.random.default_rng(0))
    noisy, _, _, _ = probe_frame(belt, (100.0, 100.0, 100.0), 0.01, 4,
                                 np.random.default_rng(0))
    assert clean == 0
    assert noisy > 0


@pytest.mark.parametrize("dir_name,slug", [
    ("bag_oi1", "bag"),
    ("bag_oi1_dyn", "bag"),
    ("helmet_oi2_node", "helmet"),
    ("bottle_oi0", "bottle"),
    ("box_300x200x200_oi2", "box_300x200x200"),
])
def test_dump_dir_names_map_to_catalogue_items(dir_name, slug):
    assert slug_of_dir(dir_name) == slug


def test_an_unknown_dump_dir_is_loud_instead_of_scoring_against_the_wrong_truth():
    with pytest.raises(ValueError):
        slug_of_dir("mystery_oi0")
