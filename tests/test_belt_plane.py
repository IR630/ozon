# -*- coding: utf-8 -*-
"""The belt plane must be MEASURED, and must refuse to guess when it cannot be.

Two consumers depend on the distinction. Runtime self-diagnosis (path_to_line.md
#8) reads the residual to this plane as its drift signal, so a plane quietly
dragged by the item on it would report "all clear" while the extrinsics walk off.
Calibration as a procedure (#16) reports the same residual as its acceptance
number. Both are worthless if a bad fit is returned as if it were good — hence
`fitted`, and hence the fail-closed geometry behind it.

Thresholds are calibrated on LIVE frames, never on the analytic clouds here; the
analytic clouds pin BEHAVIOUR. `test_the_live_dumps_fit_inside_the_shipped_thresholds`
is the one that ties the two together, and it skips when runs/ is not in the
checkout (gitignored work dir).
"""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.belt_plane import (  # noqa: E402
    CAPTURE_M,
    MAX_RMS_M,
    MIN_INLIER_FRAC,
    NOMINAL_BELT_PLANE,
    PlaneFit,
    fit_belt_plane,
    height_above_plane_m,
    residual_to_plane_m,
)
from src.perception import BELT_TOP_Z_M  # noqa: E402


def _belt(n=6000, z_m=BELT_TOP_Z_M, tilt_deg=0.0, noise_m=0.0, seed=0):
    """A rectangle of belt in world metres, optionally tilted about +x and noised."""
    rng = np.random.default_rng(seed)
    x = rng.uniform(1.0, 2.0, n)
    y = rng.uniform(-0.25, 0.25, n)
    z = z_m + y * np.tan(np.radians(tilt_deg)) + rng.normal(0.0, noise_m, n)
    return np.column_stack([x, y, z])


def test_a_clean_belt_is_recovered_to_a_fraction_of_a_millimetre():
    fit = fit_belt_plane(_belt())
    assert fit.fitted
    assert fit.offset_m == pytest.approx(BELT_TOP_Z_M, abs=1e-4)
    assert fit.normal[2] == pytest.approx(1.0, abs=1e-4)
    assert fit.rms_m < 1e-4


def test_the_normal_always_points_up_whatever_the_point_order():
    """Consumers read `height_above_plane_m` as "over the belt"; a flipped normal
    would silently invert that sign for every one of them."""
    fit = fit_belt_plane(_belt()[::-1])
    assert fit.normal[2] > 0.9


def test_a_tilted_belt_is_reported_as_tilted_and_not_flattened():
    """The drift signal IS the tilt, so the fit must not fail closed on it.

    A well-fitted plane that has moved is exactly what self-diagnosis needs to
    see; only a fit that cannot be trusted (few inliers, high residual) is
    refused. Rejecting tilt would throw away the measurement it exists to make.
    """
    fit = fit_belt_plane(_belt(tilt_deg=0.5))
    assert fit.fitted
    tilt = np.degrees(np.arccos(abs(float(np.asarray(fit.normal) @ [0, 0, 1.0]))))
    assert tilt == pytest.approx(0.5, abs=0.05)


def test_contamination_is_trimmed_instead_of_dragging_the_plane():
    """The load-bearing case: one plain least-squares pass is NOT enough.

    A bare SVD over the captured strip minimises the residual to everything in
    it, so the item's own points tilt and lift the plane; the design pass that
    preceded this module measured rms 5.5 mm and MORE belt leaking through than
    with no fit at all. The MAD trim + refit is therefore mandatory, and this
    test is what says so.
    """
    belt = _belt(n=6000)
    lid = belt[:700].copy()
    lid[:, 2] += 0.015           # a lid inside the capture window, 15 mm up
    contaminated = np.vstack([belt, lid])

    fit = fit_belt_plane(contaminated)

    assert fit.fitted
    assert abs(fit.offset_m - BELT_TOP_Z_M) < 5e-4, "the item dragged the plane"
    assert fit.inlier_frac < 1.0, "the contamination should be trimmed, not fitted"


def test_a_frame_without_a_belt_fails_closed_to_the_nominal_plane():
    """No belt in the capture window must never become an invented plane."""
    away = _belt(z_m=BELT_TOP_Z_M + 0.5)
    fit = fit_belt_plane(away)
    assert not fit.fitted
    assert fit.normal == NOMINAL_BELT_PLANE.normal
    assert fit.offset_m == NOMINAL_BELT_PLANE.offset_m


def test_a_fit_too_rough_to_trust_keeps_its_evidence_while_failing_closed():
    """Fail-closed returns NOMINAL GEOMETRY but the MEASURED quality numbers.

    Self-diagnosis has to say WHY it stopped trusting the plane; a bare `None`
    (the first sketch of this module) throws that away and makes every consumer
    reinvent the fallback.
    """
    fit = fit_belt_plane(_belt(noise_m=0.02, seed=3))
    assert not fit.fitted
    assert fit.offset_m == NOMINAL_BELT_PLANE.offset_m
    assert fit.rms_m > MAX_RMS_M
    assert np.isfinite(fit.rms_m)


def test_height_above_the_plane_is_signed_and_measured_from_the_fit():
    fit = fit_belt_plane(_belt())
    pts = np.array([[1.5, 0.0, BELT_TOP_Z_M + 0.1],
                    [1.5, 0.0, BELT_TOP_Z_M],
                    [1.5, 0.0, BELT_TOP_Z_M - 0.02]])
    h = height_above_plane_m(pts, fit)
    assert h[0] == pytest.approx(0.1, abs=1e-3)
    assert h[1] == pytest.approx(0.0, abs=1e-3)
    assert h[2] == pytest.approx(-0.02, abs=1e-3)


def test_the_residual_is_the_unsigned_height():
    fit = fit_belt_plane(_belt())
    pts = np.array([[1.5, 0.0, BELT_TOP_Z_M + 0.1], [1.5, 0.0, BELT_TOP_Z_M - 0.1]])
    assert np.allclose(residual_to_plane_m(pts, fit), np.abs(height_above_plane_m(pts, fit)))


def test_an_empty_cloud_fails_closed_rather_than_raising():
    fit = fit_belt_plane(np.empty((0, 3)))
    assert fit == NOMINAL_BELT_PLANE


def test_decimation_does_not_move_the_plane():
    """The fit is capped for the frame budget; the cap must not cost accuracy.

    Live clouds bring 75k-90k belt points and an uncapped fit costs 14-38 ms
    against a 66.7 ms period. The cap is only admissible because it leaves the
    plane where it was.
    """
    big = _belt(n=90_000, seed=5)
    assert fit_belt_plane(big).offset_m == pytest.approx(
        fit_belt_plane(big[::20]).offset_m, abs=1e-4)


def test_the_capture_window_is_narrow_on_purpose():
    """Widening the capture is not free, so the constant is pinned with its reason.

    Measured on the live dumps: at +-20 mm the trimmed fit gives 0.44-1.02 mm rms
    on the side heads, at +-50 mm it degrades to 3.6-8.0 mm and the tilt runs to
    1.6 deg — the wider window admits belt-side structure the trim cannot recover
    from. 20 mm still covers the whole calibration budget (2 mm offset, 0.2 deg
    over the half-width of the belt).
    """
    assert CAPTURE_M == 0.020


@pytest.mark.parametrize("slug", ["helmet", "pen", "bag"])
def test_the_live_dumps_fit_inside_the_shipped_thresholds(slug):
    """Every head of every rig dump must pass the thresholds the module ships.

    This is where MAX_RMS_M and MIN_INLIER_FRAC are answerable to reality rather
    than to taste: measured worst case is 1.02 mm rms and 0.87 inliers, and both
    thresholds sit outside that with room.
    """
    pytest.importorskip("cv2")
    from src.constants import (
        CAMERA_SIDE_NEG_Y_POSE_M,
        CAMERA_SIDE_POS_Y_POSE_M,
        CAMERA_TOP_POSE_M,
    )
    from src.multiview import world_cloud_from_depth
    from src.perception import FX, FY, load_depth_png

    dump = ROOT / "runs" / "frames" / f"{slug}_3cam"
    if not dump.is_dir():
        pytest.skip(f"{dump} not in this checkout (runs/g_dump_3cam.sh)")

    for fname, pose in (("depth_000.png", CAMERA_TOP_POSE_M),
                        ("depth_side_neg_y_000.png", CAMERA_SIDE_NEG_Y_POSE_M),
                        ("depth_side_pos_y_000.png", CAMERA_SIDE_POS_Y_POSE_M)):
        pts = world_cloud_from_depth(load_depth_png(str(dump / fname)), pose, FX, FY)
        fit = fit_belt_plane(pts)
        assert fit.fitted, f"{slug}/{fname}: rms {fit.rms_m*1000:.2f} mm, inliers {fit.inlier_frac:.3f}"
        assert fit.rms_m < MAX_RMS_M
        assert fit.inlier_frac > MIN_INLIER_FRAC
        # The plane the heads agree on is the belt, to within the calibration budget.
        assert abs(fit.offset_m - BELT_TOP_Z_M) < 0.002


def test_the_nominal_plane_is_the_geometry_the_shipped_pipeline_already_assumes():
    """belt_plane is a NEW consumer, not a replacement: its fallback must be the
    very plane `src.perception` measures against, or the two would disagree on a
    frame where the fit fails."""
    assert isinstance(NOMINAL_BELT_PLANE, PlaneFit)
    assert NOMINAL_BELT_PLANE.normal == (0.0, 0.0, 1.0)
    assert NOMINAL_BELT_PLANE.offset_m == BELT_TOP_Z_M
    assert not NOMINAL_BELT_PLANE.fitted
