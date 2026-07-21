# -*- coding: utf-8 -*-
"""The camera-count probe must not flatter — or penalise — a config by construction.

The whole verdict of this probe rests on two mechanics: rotating one canonical
cloud per item must place a body exactly like re-sampling it would, and the
calibration-error model must be a genuine RIGID misregistration (not a stretch,
which would fake a dimension error out of thin air).
"""
import numpy as np
import pytest

from scripts.probe_camera_count import (
    BELT_TOP_Z_M,
    CALIBRATIONS,
    CONFIGS,
    misregister,
    place,
)


def _cloud_mm(n=2000, seed=0):
    rng = np.random.default_rng(seed)
    return rng.normal(0.0, 40.0, size=(n, 3))


def test_perfect_calibration_leaves_the_cloud_untouched():
    """The geometric-limit row must be the identity, or it is not a limit."""
    pts = _cloud_mm() / 1000.0
    out = misregister(pts, 0.0, 0.0, np.random.default_rng(0))
    assert out is pts


def test_calibration_error_is_rigid():
    """Misregistration may move and rotate a head's cloud, never resize it.

    A non-rigid perturbation would inflate measured extents directly, and the
    probe would "discover" that more heads measure worse — an artifact of the
    error model rather than of the rig.
    """
    pts = _cloud_mm() / 1000.0
    out = misregister(pts, 3.0, 0.3, np.random.default_rng(1))
    idx = np.arange(0, len(pts), 7)
    before = np.linalg.norm(pts[idx][:, None, :] - pts[idx][None, :, :], axis=-1)
    after = np.linalg.norm(out[idx][:, None, :] - out[idx][None, :, :], axis=-1)
    assert np.allclose(before, after, atol=1e-9)


def test_calibration_error_actually_moves_the_cloud():
    """Guard against a silently no-op error model."""
    pts = _cloud_mm() / 1000.0
    out = misregister(pts, 3.0, 0.3, np.random.default_rng(2))
    assert np.linalg.norm(out.mean(axis=0) - pts.mean(axis=0)) > 1e-5


def test_calibration_error_is_reproducible_from_its_seed():
    """Karpathy #5: same seed, same perturbation, or config rows are not comparable."""
    pts = _cloud_mm() / 1000.0
    a = misregister(pts, 2.0, 0.2, np.random.default_rng(7))
    b = misregister(pts, 2.0, 0.2, np.random.default_rng(7))
    assert np.array_equal(a, b)


def test_place_rests_the_body_on_the_belt_and_centres_it():
    """Same placement contract as render_depth / probe_side_camera."""
    box = np.array([[x, y, z] for x in (-50.0, 50.0)
                    for y in (-30.0, 30.0) for z in (-20.0, 20.0)])
    out = place(box, (0.0, 0.0, 0.0, 1.0))
    assert out[:, 2].min() == pytest.approx(BELT_TOP_Z_M)
    assert (out[:, 0].min() + out[:, 0].max()) / 2 == pytest.approx(1.5)
    assert (out[:, 1].min() + out[:, 1].max()) / 2 == pytest.approx(0.0)


def test_place_rotation_preserves_the_body():
    """Rotating the canonical cloud must not resize it — extents only permute."""
    box = np.array([[x, y, z] for x in (-50.0, 50.0)
                    for y in (-30.0, 30.0) for z in (-20.0, 20.0)])
    flat = place(box, (0.0, 0.0, 0.0, 1.0))
    # 90 deg about Z: (x, y) extents swap, z is unchanged
    turned = place(box, (0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)))
    ext_flat = sorted(np.ptp(flat, axis=0))
    ext_turned = sorted(np.ptp(turned, axis=0))
    assert ext_flat == pytest.approx(ext_turned, abs=1e-9)


def test_configs_are_ordered_by_head_count_and_share_the_top_head():
    """Every config must extend the SAME top-down head, else rows compare rigs
    that differ in more than head count and the verdict is confounded."""
    top = CONFIGS["1: top"][0]
    for name, cams in CONFIGS.items():
        assert cams[0] == top, f"{name} does not start from the production top head"
        assert int(name[0]) == len(cams), f"{name} label disagrees with its head count"


def test_calibration_sweep_starts_from_the_geometric_limit():
    assert CALIBRATIONS[0][1] == 0.0 and CALIBRATIONS[0][2] == 0.0
    sigmas = [c[1] for c in CALIBRATIONS]
    assert sigmas == sorted(sigmas), "calibration sweep must be monotone"
