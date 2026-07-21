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
    near_threshold,
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


def test_near_threshold_reads_both_ends_of_the_sorter_window():
    """The band must guard the floor and the ceiling — the pen fails against
    10 mm, a box would fail against 450/320/320."""
    assert near_threshold([148.0, 13.0, 10.4], 1.0)      # pen, against the floor
    assert near_threshold([449.0, 300.0, 300.0], 2.0)    # box, against the ceiling
    assert not near_threshold([200.0, 150.0, 60.0], 4.0)  # lunchbox, clear of both


def test_near_threshold_is_sorted_before_comparing():
    """Dims arrive in any order; the sorter window is defined on sorted dims, so
    an unsorted comparison would guard the wrong axis against the wrong limit."""
    assert near_threshold([300.0, 449.0, 300.0], 2.0)
    assert near_threshold([449.0, 300.0, 300.0], 2.0)


def test_zero_band_flags_nothing_away_from_a_threshold():
    """A degenerate band must not quietly flag the whole flow."""
    assert not near_threshold([200.0, 150.0, 60.0], 0.0)


def _dome_cloud_m(radius_mm=140.0, n=3000, seed=0):
    """Upper half-shell resting on the belt: high relief, the top view under-reads."""
    rng = np.random.default_rng(seed)
    v = rng.normal(size=(n, 3))
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    v[:, 2] = np.abs(v[:, 2])
    pts = v * radius_mm / 1000.0
    pts[:, 2] += BELT_TOP_Z_M
    return pts


def _flat_slab_m(lx=148.0, ly=13.0, lz=9.0, n=3000, seed=0):
    """Pen-like slab: near-zero relief, the top view's shadow IS the footprint."""
    rng = np.random.default_rng(seed)
    pts = rng.uniform(-0.5, 0.5, size=(n, 3)) * np.array([lx, ly, lz]) / 1000.0
    pts[:, 2] = lz / 2000.0          # flat lid: one height, no relief
    pts[:, 2] += BELT_TOP_Z_M + lz / 2000.0
    return pts


def test_structural_gate_keeps_a_flat_body_on_the_top_head_alone():
    """The pen's failure is inflation, so extra heads must not touch its footprint.

    A misregistered side head is handed in; under union it widens the slab, under
    the gate it may only raise the height, never the footprint.
    """
    top = _flat_slab_m()
    side = _flat_slab_m(seed=1) + np.array([0.0, 0.006, 0.0])   # 6 mm out, sideways
    parts = [top, side, _flat_slab_m(seed=2)]

    gated = fuse_dims(parts, "structural-gated")
    union = fuse_dims(parts, "union")
    assert gated[1] < union[1], f"gate must not inherit the union widening: {gated} vs {union}"
    assert gated[1] == pytest.approx(fuse_dims([top], "structural-gated")[1], abs=0.5)


def test_structural_gate_admits_every_head_on_a_domed_body():
    """The helmet's failure is the opposite — the top view cannot see its flanks,
    so the gate must fall through to the full union there."""
    parts = [_dome_cloud_m(seed=s) for s in (0, 1, 2)]
    assert fuse_dims(parts, "structural-gated") == fuse_dims(parts, "union")


def test_structural_gate_reads_relief_from_the_top_head_only():
    """The gate asks whether the TOP head is trustworthy; a side head's own relief
    must not decide that, or a flat item flanked by side views flips to union."""
    parts = [_flat_slab_m(), _dome_cloud_m(seed=1), _dome_cloud_m(seed=2)]
    assert fuse_dims(parts, "structural-gated") != fuse_dims(parts, "union")


def test_union_prod_lets_the_shadow_box_win_when_it_is_smaller():
    """The production contract the probe's own cloud_dims_mm switches off.

    cloud_dims_mm hands _body_obb_dims_mm a 1e4 legacy box so the tilted box
    always wins; measure_item passes the real shadow box and keeps the smaller
    volume. On a flat slab the tilted search can only match or overshoot, so
    union-prod must not read LARGER than the probe's own union there.
    """
    parts = [_flat_slab_m(seed=s) for s in (0, 1, 2)]
    prod = fuse_dims(parts, "union-prod")
    probe = fuse_dims(parts, "union")
    assert prod is not None
    assert np.prod(prod) <= np.prod(probe) * 1.001, f"{prod} vs {probe}"


def test_union_prod_still_measures_a_dome():
    """The guard must not break the case the body-OBB exists for: a dome's
    tilted box is genuinely smaller than its shadow box, so it should win."""
    parts = [_dome_cloud_m(seed=s) for s in (0, 1, 2)]
    dims = fuse_dims(parts, "union-prod")
    assert dims is not None
    assert dims[0] == pytest.approx(280.0, rel=0.15)   # 2R of a 140 mm half-shell
