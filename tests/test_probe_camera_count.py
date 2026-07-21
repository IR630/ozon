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
    fuse_dims,
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


def _box_faces_mm(lx, ly, lz, per_face=900, seed=0):
    """Surface cloud of an axis-aligned box, in metres on the belt."""
    rng = np.random.default_rng(seed)
    half = np.array([lx, ly, lz]) / 2.0
    pts = []
    for axis in range(3):
        for sign in (-1.0, 1.0):
            p = rng.uniform(-1.0, 1.0, size=(per_face, 3)) * half
            p[:, axis] = sign * half[axis]
            pts.append(p)
    pts = np.vstack(pts) / 1000.0
    pts[:, 2] += BELT_TOP_Z_M + lz / 2000.0
    return pts


def test_loo_min_falls_back_to_union_below_three_heads():
    """Documented contract: with 1-2 heads the rule IS union, bit for bit.

    The 1- and 2-head columns of the loo-min table are therefore a free check
    that the plumbing did not silently change what the baseline measures.
    """
    parts = [_box_faces_mm(300, 200, 100, seed=1), _box_faces_mm(300, 200, 100, seed=2)]
    for n in (1, 2):
        assert fuse_dims(parts[:n], "loo-min") == fuse_dims(parts[:n], "union")


def test_loo_min_rejects_one_misregistered_head():
    """The point of the rule: one head's rigid shift must not reach the dims.

    Union takes a MAXIMUM over heads, so a shifted head pushes the box outward
    and can only inflate. loo-min keeps the leave-one-out candidate that dropped
    that head. NOTE a leave-one-out MEDIAN cannot do this and was rejected on
    exactly this case: the bad head sits in 2 of the 3 candidates, a majority,
    so the median preserves the inflation (measured 20.0 against union's 20.0).
    """
    clean = [_box_faces_mm(300, 200, 100, seed=s) for s in (1, 2, 3)]
    truth = fuse_dims(clean, "union")

    shifted = list(clean)
    shifted[2] = clean[2] + np.array([0.02, 0.0, 0.0])   # 20 mm out, one head

    union_err = abs(fuse_dims(shifted, "union")[0] - truth[0])
    loo_err = abs(fuse_dims(shifted, "loo-min")[0] - truth[0])
    assert union_err > 10.0, f"union should inflate on a 20 mm shift, got {union_err:.1f}"
    assert loo_err < union_err / 2.0, f"loo-min {loo_err:.1f} vs union {union_err:.1f}"


def test_loo_min_deflates_when_every_head_is_clean():
    """The cost side of loo-min, stated as a test rather than discovered later.

    With no misregistration at all it still drops a head's worth of coverage, so
    it can only read the same or SMALLER than union. That is the mechanism by
    which it could push a genuinely-at-threshold item under the threshold — the
    risk the 165-pose sweep exists to price.
    """
    clean = [_box_faces_mm(300, 200, 100, seed=s) for s in (1, 2, 3)]
    union = fuse_dims(clean, "union")
    loo = fuse_dims(clean, "loo-min")
    assert all(lo <= u + 1e-9 for lo, u in zip(loo, union)), f"{loo} vs {union}"


def test_unknown_fusion_rule_is_loud():
    """A typo in a rule name must not silently fall through to some default."""
    parts = [_box_faces_mm(300, 200, 100, seed=s) for s in (1, 2, 3)]
    with pytest.raises(ValueError):
        fuse_dims(parts, "median-ish")
