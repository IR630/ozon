# -*- coding: utf-8 -*-
"""The belt plane, measured from a head's own point cloud instead of assumed.

WHY THIS EXISTS. `src.perception` segments an item off the belt by comparing
depth against the SCALAR `BELT_DEPTH_M` — a number from the world file. That is
correct for the shipped top-view path and is deliberately left alone here: the
163/165 census was taken on it. But two of the uncovered real-line failure modes
need the belt as a MEASURED plane, not as a constant:

  * runtime self-diagnosis (`docs/report/path_to_line.md`, optics degradation and
    extrinsics drift): between items the camera sees bare belt, and bare belt is
    obliged to lie in a plane. Residual to the fitted plane is a free, continuous
    drift signal — but only if the fit does not quietly follow the drift;
  * calibration as an operating procedure: the acceptance number of a calibration
    run is exactly this residual, and a rig outside budget has to REFUSE to run
    rather than absorb the error into a margin.

This module is a new consumer of the geometry, never a replacement for it. On any
frame it cannot trust it returns the NOMINAL plane — the same z the shipped
pipeline already assumes — flagged `fitted=False`, so a caller can never mistake
a guess for a measurement, and so the two never disagree about where the belt is.

ONE LEAST-SQUARES PASS IS NOT ENOUGH, AND THAT IS THE WHOLE DESIGN. A bare SVD
minimises the residual to everything inside the capture window, item included, so
the item tilts and lifts the very plane it is supposed to be measured against.
The design pass that preceded this module measured rms 5.5 mm that way and got
MORE belt leaking into the item mask than with no fit at all. Hence: capture a
narrow strip around where the belt is expected, fit, trim by MAD, refit.

Measured on the live rig dumps (`runs/frames/*_3cam`, `docs/experiments.md` 26.07):
trimming takes the side heads from 3.8-4.3 mm rms to 0.44-1.02 mm, and the top
head reads the belt at 398.8 mm against the side heads' 400.9 mm — a 2.1 mm
disagreement between heads about one physical plane, which is the number a
calibration procedure exists to remove.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from src.perception import BELT_TOP_Z_M


@dataclass(frozen=True)
class PlaneFit:
    """A belt plane and how much it deserves to be believed.

    The plane is `normal . p == offset_m`, with `normal` a unit vector pointing
    UP, so `height_above_plane_m` is positive over the belt for every head.
    `fitted` is the only field a segmentation consumer may branch on; `rms_m` and
    `inlier_frac` are the evidence and stay populated even when `fitted` is False,
    because self-diagnosis has to report WHY the plane stopped being trustworthy.
    """
    normal: tuple
    offset_m: float
    inlier_frac: float
    rms_m: float
    fitted: bool


# Fail-closed geometry: the plane `src.perception` already assumes. Quality fields
# say "nothing was measured" rather than "measured perfectly".
NOMINAL_BELT_PLANE = PlaneFit((0.0, 0.0, 1.0), BELT_TOP_Z_M, 0.0, float("inf"), False)

# Half-width of the strip captured around the expected belt height. Narrow ON
# PURPOSE: measured on the live dumps, +-20 mm gives 0.44-1.02 mm rms on the side
# heads while +-50 mm degrades to 3.6-8.0 mm and up to 1.6 deg of tilt — the wider
# window admits belt-side structure the trim cannot recover from. 20 mm still
# covers the whole calibration budget (2 mm of offset, 0.2 deg over the belt).
CAPTURE_M = 0.020

# Points actually fitted. A live belt strip brings 75k-90k points and fitting all
# of them costs 14-38 ms against a 66.7 ms period at 15 Hz; capping at 8000 brings
# the fit itself to 2.0-3.6 ms and moves the plane by less than 0.03 mm.
# `fit_belt_plane` as a whole then measures 4.9-9.4 ms on a live cloud — the rest
# is the capture scan over the full 200k-300k points, which no cap can avoid.
# Deterministic stride rather than a sample: the project's reproducibility rule
# says randomness enters only through an explicit seed, and this way the
# production path takes no seed at all.
MAX_FIT_POINTS = 8000

# Trim width. Standard robust-statistics choice; MAD is scaled to a sigma by the
# usual 1.4826 first.
TRIM_SIGMAS = 3.0

# Refusal thresholds, calibrated on the live dumps (worst case there: 1.02 mm rms,
# 0.87 inliers). MAX_RMS_M is the calibration budget itself — a plane whose own
# residual exceeds +-2 mm carries no information the nominal plane does not
# already carry, and stays well under the 5 mm segmentation margin it feeds.
MAX_RMS_M = 0.002
MIN_INLIER_FRAC = 0.60

# Below this there is no belt in the window worth calling a measurement. Live
# frames deliver tens of thousands; three points would define a plane and mean
# nothing.
_MIN_FIT_POINTS = 200


def _fit_once(pts_m):
    """(unit normal pointing up, offset) of the least-squares plane through pts."""
    centroid = pts_m.mean(axis=0)
    # Smallest-eigenvalue direction of the covariance = the direction of least
    # spread = the plane normal. eigh on the 3x3 covariance rather than an SVD of
    # the N x 3 matrix: same answer, and the cost stops depending on N.
    _vals, vecs = np.linalg.eigh(np.cov((pts_m - centroid).T))
    normal = vecs[:, 0]
    if normal[2] < 0.0:
        normal = -normal
    return normal, float(normal @ centroid)


def fit_belt_plane(pts_m, expected_z_m=BELT_TOP_Z_M, capture_m=CAPTURE_M):
    """Belt plane measured from a world-frame cloud, or the nominal plane refused.

    `pts_m` is (N, 3) in world metres — what `src.multiview.world_cloud_from_depth`
    produces for any head, which is why this works for a side head as well as the
    top one.
    """
    pts_m = np.asarray(pts_m, dtype=float)
    if pts_m.ndim != 2 or len(pts_m) == 0:
        return NOMINAL_BELT_PLANE

    near = pts_m[np.abs(pts_m[:, 2] - expected_z_m) <= capture_m]
    if len(near) < _MIN_FIT_POINTS:
        return NOMINAL_BELT_PLANE
    if len(near) > MAX_FIT_POINTS:
        near = near[::len(near) // MAX_FIT_POINTS]

    normal, offset = _fit_once(near)
    residual = near @ normal - offset
    # MAD, not std: the contamination this trim exists to remove would inflate a
    # std and widen the very window meant to exclude it.
    # Centred on the MEDIAN residual, not on zero — the first fit is the dragged
    # one, so the belt itself sits at a nonzero residual and a window around zero
    # throws the belt away and keeps nothing at all.
    centre = float(np.median(residual))
    sigma = max(1.4826 * float(np.median(np.abs(residual - centre))), 1e-6)
    inliers = np.abs(residual - centre) <= TRIM_SIGMAS * sigma
    if inliers.sum() >= _MIN_FIT_POINTS:
        normal, offset = _fit_once(near[inliers])
        residual = near[inliers] @ normal - offset

    rms = float(np.sqrt(np.mean(residual ** 2)))
    inlier_frac = float(inliers.mean())
    if rms > MAX_RMS_M or inlier_frac < MIN_INLIER_FRAC:
        return PlaneFit(NOMINAL_BELT_PLANE.normal, NOMINAL_BELT_PLANE.offset_m,
                        inlier_frac, rms, False)
    return PlaneFit(tuple(float(c) for c in normal), offset, inlier_frac, rms, True)


def height_above_plane_m(pts_m, fit):
    """Signed distance from the plane, positive above the belt (metres)."""
    pts_m = np.asarray(pts_m, dtype=float)
    if not len(pts_m):
        return np.empty(0)
    return pts_m @ np.asarray(fit.normal, dtype=float) - fit.offset_m


def residual_to_plane_m(pts_m, fit):
    """Unsigned distance from the plane — the flatness signal, sign discarded."""
    return np.abs(height_above_plane_m(pts_m, fit))
