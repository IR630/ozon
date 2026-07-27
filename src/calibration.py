# -*- coding: utf-8 -*-
"""Calibration as an operating procedure: measure the rig, or refuse to run it.

WHY THIS EXISTS. `docs/report/path_to_line.md` #16 and `rig_decision.md` §7.1:
the third head's whole advantage is calibration-borne (+14 pp of tolerance at
perfect calibration, +2 pp at a mediocre 3 mm / 0.3 deg — where TWO heads beat
three). A rig nobody re-checks between shifts therefore does not have the rig we
argued for; it has whatever the last bump left behind. `SIDE_BELT_MARGIN_M = 8 mm`
today buys silence about that drift instead of removing it.

THE TARGET IS THE BARE BELT, NOT A CHECKERBOARD, and that is a decision rather
than a shortcut. What the pipeline consumes is one number per head — where the
belt plane lies in that head's world frame. A checkerboard would estimate the
full extrinsics, and this stack has nowhere to put the answer: `camera_axes`
builds a head's basis as `forward x world_up` (`src/multiview.py:43`), five
degrees of freedom of six, so roll is inexpressible in our reconstruction
(`rig_decision.md` §6). Calibrating what we cannot represent produces a report
nobody can act on. The belt, by contrast, is present every shift, needs no
hardware, and is exactly the surface the segmentation thresholds are measured
from.

WHAT THE PROCEDURE REFUSES ON. Two numbers, and neither is invented here:

  * offset at the reference point — the +-2 mm calibration budget this project
    has quoted since 25.07, the same number `belt_plane.MAX_RMS_M` already uses;
  * tilt — the +-0.2 deg half of that same budget.

THE HEAD-TO-HEAD SPREAD IS REPORTED AND DELIBERATELY NOT GATED. It is the number
a calibration exists to remove — the heads' disagreement about one physical
plane, 2.1 mm on the live dumps today — but a separate ceiling for it would be
unreachable by construction: with every head held inside +-2 mm the spread cannot
exceed 4 mm, and 4 mm is already inside the `perception.MASK_MARGIN_M` = 5 mm
that absorbs it downstream. So the per-head gate ALREADY guarantees what a spread
gate would claim to, and adding one would be a check that can never fire. It is
printed because an operator watching it climb across shifts sees the drift long
before either head crosses its own limit.

A head whose plane came back `fitted=False` is an automatic refusal. Not
measuring is not the same as measuring zero, and a procedure that treats them
alike is the procedure that certified the drift.

This module decides nothing at runtime and touches no shipped path: it reads
clouds, it reports, and `scripts/calibrate_shift.py` turns the report into an
exit code an operator can be stopped by.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from src.belt_plane import fit_belt_plane
from src.perception import BELT_TOP_Z_M, CAMERA_X_M, MASK_MARGIN_M

# Where every head is asked "how high is the belt here?". One physical point for
# the whole rig, on the belt centreline under the top head — otherwise heads
# would be compared at different places and a tilt would read as a disagreement.
REFERENCE_POINT_M = (CAMERA_X_M, 0.0)

# The calibration budget, in the units the report prints.
MAX_OFFSET_MM = 2.0
MAX_TILT_DEG = 0.2
# Not a threshold — the bound the gate above already implies, printed next to the
# measured spread so the report shows its own headroom (see module docstring).
IMPLIED_SPREAD_CEILING_MM = 2.0 * MAX_OFFSET_MM
# What absorbs that spread downstream. Imported rather than restated: if the
# segmentation margin ever shrinks below the implied ceiling, this assertion is
# the thing that notices, and it costs nothing.
_ABSORBING_MARGIN_MM = MASK_MARGIN_M * 1000.0
assert IMPLIED_SPREAD_CEILING_MM <= _ABSORBING_MARGIN_MM, (
    f"per-head budget +-{MAX_OFFSET_MM} mm admits a {IMPLIED_SPREAD_CEILING_MM} mm "
    f"head disagreement, past the {_ABSORBING_MARGIN_MM} mm margin that absorbs it")


@dataclass(frozen=True)
class HeadCalibration:
    """One head's answer about the belt, in the units a fitter reads."""

    head: str
    belt_z_mm: float | None  # measured belt height at REFERENCE_POINT_M; None if unfitted
    offset_mm: float | None  # belt_z_mm minus nominal, signed
    tilt_deg: float | None   # angle between the fitted normal and world up
    rms_mm: float            # residual of the fit — populated even when unfitted
    inlier_frac: float
    fitted: bool


@dataclass(frozen=True)
class CalibrationReport:
    """The whole rig, and whether it is allowed to run this shift."""

    heads: tuple
    spread_mm: float | None  # max-min of belt_z_mm across fitted heads
    accepted: bool
    refusals: tuple  # human-readable reasons; empty if and only if accepted


def _plane_z_at(fit, point_xy_m):
    """Height of the fitted plane over a belt point, metres."""
    nx, ny, nz = fit.normal
    x, y = point_xy_m
    return (fit.offset_m - nx * x - ny * y) / nz


def calibrate_head(head, pts_m, reference_point_m=REFERENCE_POINT_M):
    """Fit one head's belt plane and express it as the numbers a report shows."""
    fit = fit_belt_plane(pts_m)
    rms_mm = fit.rms_m * 1000.0 if np.isfinite(fit.rms_m) else float("inf")
    if not fit.fitted:
        return HeadCalibration(head, None, None, None, rms_mm, fit.inlier_frac, False)

    belt_z_mm = _plane_z_at(fit, reference_point_m) * 1000.0
    # The normal is unit and points up, so its z component is the cosine of the
    # tilt away from world up; clipped because floating point can exceed 1.0.
    tilt_deg = float(np.degrees(np.arccos(np.clip(fit.normal[2], -1.0, 1.0))))
    return HeadCalibration(
        head=head,
        belt_z_mm=belt_z_mm,
        offset_mm=belt_z_mm - BELT_TOP_Z_M * 1000.0,
        tilt_deg=tilt_deg,
        rms_mm=rms_mm,
        inlier_frac=fit.inlier_frac,
        fitted=True,
    )


def calibrate_rig(clouds_by_head, reference_point_m=REFERENCE_POINT_M):
    """Calibrate every head and decide whether the rig may run.

    `clouds_by_head` maps a head name to its world-frame cloud (N, 3) in metres —
    what `src.multiview.world_cloud_from_depth` returns, for the top head and the
    side heads alike.
    """
    heads = tuple(calibrate_head(name, pts, reference_point_m)
                  for name, pts in clouds_by_head.items())

    refusals = []
    if not heads:
        refusals.append("REFUSE: no head delivered a cloud")

    for head in heads:
        if not head.fitted:
            refusals.append(
                f"REFUSE: {head.head}: belt plane not measured "
                f"(rms {head.rms_mm:.2f} mm, inliers {head.inlier_frac:.2f})")
            continue
        if abs(head.offset_mm) > MAX_OFFSET_MM:
            refusals.append(
                f"REFUSE: {head.head}: belt offset {head.offset_mm:+.2f} mm "
                f"outside +-{MAX_OFFSET_MM:.1f} mm")
        if head.tilt_deg > MAX_TILT_DEG:
            refusals.append(
                f"REFUSE: {head.head}: belt tilt {head.tilt_deg:.3f} deg "
                f"outside +-{MAX_TILT_DEG:.1f} deg")

    fitted_z = [head.belt_z_mm for head in heads if head.fitted]
    spread_mm = max(fitted_z) - min(fitted_z) if len(fitted_z) > 1 else None

    return CalibrationReport(heads, spread_mm, not refusals, tuple(refusals))


def format_report(report):
    """The report an operator reads, one head per line then the verdict."""
    lines = [f"{'head':<20} {'belt z, mm':>11} {'offset, mm':>11} "
             f"{'tilt, deg':>10} {'rms, mm':>9} {'inliers':>8}"]
    for head in report.heads:
        if head.fitted:
            lines.append(
                f"{head.head:<20} {head.belt_z_mm:>11.2f} {head.offset_mm:>+11.2f} "
                f"{head.tilt_deg:>10.3f} {head.rms_mm:>9.2f} {head.inlier_frac:>8.2f}")
        else:
            lines.append(
                f"{head.head:<20} {'NOT MEASURED':>11} {'-':>11} "
                f"{'-':>10} {head.rms_mm:>9.2f} {head.inlier_frac:>8.2f}")
    if report.spread_mm is not None:
        lines.append(f"head-to-head spread: {report.spread_mm:.2f} mm "
                     f"(not gated; the per-head budget implies at most "
                     f"{IMPLIED_SPREAD_CEILING_MM:.1f} mm)")
    lines.append("ACCEPTED: rig may run this shift" if report.accepted
                 else "\n".join(report.refusals))
    return "\n".join(lines)
