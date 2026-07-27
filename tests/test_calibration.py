# -*- coding: utf-8 -*-
"""Calibration must produce a REFUSAL, not a margin (path_to_line.md #16).

The point of the procedure is the case where it says no. `SIDE_BELT_MARGIN_M`
already absorbs drift silently; a calibration report that also absorbs it adds a
ritual and changes nothing. So the tests that matter here are the refusals: a
head out of budget, a head tilted out of budget, and — the one that is easiest to
get wrong — a head whose plane could not be measured at all, which must refuse
rather than be quietly dropped from the average.

Behaviour is pinned on analytic clouds. The live numbers live in
`docs/experiments.md`; `test_the_live_dumps_pass_the_shipped_budget` is the one
tie between the two and skips when runs/ is absent (gitignored work dir).
"""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.calibration import (  # noqa: E402
    IMPLIED_SPREAD_CEILING_MM,
    MAX_OFFSET_MM,
    MAX_TILT_DEG,
    calibrate_head,
    calibrate_rig,
    format_report,
)
from src.perception import BELT_TOP_Z_M  # noqa: E402


def _belt(n=6000, z_m=BELT_TOP_Z_M, tilt_deg=0.0, seed=0):
    """A rectangle of belt in world metres, tilted about +x — as test_belt_plane."""
    rng = np.random.default_rng(seed)
    x = rng.uniform(1.0, 2.0, n)
    y = rng.uniform(-0.25, 0.25, n)
    z = z_m + y * np.tan(np.radians(tilt_deg))
    return np.column_stack([x, y, z])


def test_a_rig_inside_budget_is_accepted_and_reports_what_it_measured():
    report = calibrate_rig({"top": _belt(), "side_neg_y": _belt(seed=1)})

    assert report.accepted
    assert report.refusals == ()
    assert all(head.fitted for head in report.heads)
    for head in report.heads:
        assert head.belt_z_mm == pytest.approx(BELT_TOP_Z_M * 1000.0, abs=0.1)
        assert head.offset_mm == pytest.approx(0.0, abs=0.1)
    assert report.spread_mm == pytest.approx(0.0, abs=0.1)


def test_a_head_outside_the_offset_budget_is_refused_by_name():
    """3 mm of drift on one head — inside no budget, and the report says which."""
    report = calibrate_rig({
        "top": _belt(),
        "side_neg_y": _belt(z_m=BELT_TOP_Z_M + 0.003, seed=1),
    })

    assert not report.accepted
    assert len(report.refusals) == 1
    assert "side_neg_y" in report.refusals[0]
    assert "offset" in report.refusals[0]


def test_a_head_tilted_out_of_budget_is_refused_even_though_its_height_is_right():
    """A plane pivoting about the reference point keeps offset ~0 and is still bad."""
    report = calibrate_rig({"top": _belt(tilt_deg=0.5)})

    assert not report.accepted
    assert "tilt" in report.refusals[0]
    head = report.heads[0]
    assert head.tilt_deg == pytest.approx(0.5, abs=0.02)
    assert head.tilt_deg > MAX_TILT_DEG


def test_a_head_that_could_not_be_measured_refuses_instead_of_being_dropped():
    """Not measuring is not measuring zero — the failure the procedure exists for."""
    report = calibrate_rig({"top": _belt(), "side_pos_y": np.empty((0, 3))})

    assert not report.accepted
    assert "side_pos_y" in report.refusals[0]
    assert "not measured" in report.refusals[0]
    # ...and the unmeasured head must not have silently contributed a spread.
    assert report.spread_mm is None


def test_an_empty_rig_refuses_rather_than_accepting_a_rig_it_never_saw():
    report = calibrate_rig({})

    assert not report.accepted
    assert report.refusals == ("REFUSE: no head delivered a cloud",)


def test_the_spread_the_gate_admits_stays_inside_the_margin_that_absorbs_it():
    """Two heads at the opposite edges of budget — accepted, and bounded as claimed."""
    report = calibrate_rig({
        "top": _belt(z_m=BELT_TOP_Z_M + 0.0019),
        "side_neg_y": _belt(z_m=BELT_TOP_Z_M - 0.0019, seed=1),
    })

    assert report.accepted
    assert report.spread_mm == pytest.approx(3.8, abs=0.1)
    assert report.spread_mm <= IMPLIED_SPREAD_CEILING_MM


def test_the_offset_is_measured_at_the_reference_point_not_along_the_normal():
    """A tilted plane's `offset_m` is not its height; the report must not confuse them."""
    head = calibrate_head("top", _belt(tilt_deg=0.15))

    assert head.fitted
    # The belt pivots about y=0, and the reference point sits on y=0, so the
    # height there is unchanged however far the plane has tilted.
    assert head.offset_mm == pytest.approx(0.0, abs=0.05)
    assert head.tilt_deg == pytest.approx(0.15, abs=0.02)


def test_a_plane_tilted_along_the_lever_arm_is_read_at_the_belt_not_at_the_origin():
    """The exact error that put a wrong 2.1 mm into the report (27.07).

    `PlaneFit.offset_m` is the distance from the WORLD ORIGIN along the normal.
    The top head looks 1.5 m along +x, so its tilt times that lever arm is a
    millimetre-scale offset that belongs to the origin, not to the belt. A belt
    tilted about +y here is flat at the origin and 1.75 mm high at the reference
    point; the report must show what is under the rig.
    """
    rng = np.random.default_rng(0)
    x = rng.uniform(1.0, 2.0, 6000)
    y = rng.uniform(-0.25, 0.25, 6000)
    tilt_deg = 0.0668
    # z = 0 at x = 0, so the plane passes through the nominal height at the origin
    # and rises along the belt — the geometry of the top head's real fit.
    z = BELT_TOP_Z_M + x * np.tan(np.radians(tilt_deg))
    head = calibrate_head("top", np.column_stack([x, y, z]))

    assert head.fitted
    lever_mm = 1.5 * np.tan(np.radians(tilt_deg)) * 1000.0
    assert lever_mm == pytest.approx(1.75, abs=0.05)
    # The height under the rig, NOT the plane's height at the origin.
    assert head.belt_z_mm == pytest.approx(BELT_TOP_Z_M * 1000.0 + lever_mm, abs=0.05)
    assert head.offset_mm == pytest.approx(lever_mm, abs=0.05)


def test_the_report_names_the_refusal_in_the_text_an_operator_reads():
    text = format_report(calibrate_rig({"top": _belt(z_m=BELT_TOP_Z_M + 0.004)}))

    assert "REFUSE" in text
    assert "ACCEPTED" not in text
    assert "top" in text


def test_an_accepted_report_says_so_and_prints_no_refusal():
    text = format_report(calibrate_rig({"top": _belt(), "side_neg_y": _belt(seed=1)}))

    assert "ACCEPTED: rig may run this shift" in text
    assert "REFUSE" not in text
    assert "head-to-head spread" in text


@pytest.mark.parametrize("offset_mm", [MAX_OFFSET_MM - 0.3, -(MAX_OFFSET_MM - 0.3)])
def test_a_head_just_inside_budget_is_accepted_in_both_directions(offset_mm):
    report = calibrate_rig({"top": _belt(z_m=BELT_TOP_Z_M + offset_mm / 1000.0)})

    assert report.accepted, report.refusals


@pytest.mark.parametrize("slug", ["bag", "helmet", "pen"])
def test_the_live_dumps_pass_the_shipped_budget(slug):
    """The one tie between the analytic clouds above and the real rig.

    Skips when runs/ is not in the checkout (gitignored work dir), exactly as
    tests/test_belt_plane.py does — the thresholds are calibrated on these frames
    and a silently absent dump must not read as a pass.
    """
    pytest.importorskip("cv2")
    sys.path.insert(0, str(ROOT / "scripts"))
    from calibrate_shift import clouds_of, head_frames

    dump = ROOT / "runs" / "frames" / f"{slug}_3cam"
    if not dump.is_dir():
        pytest.skip(f"{dump} not in this checkout (runs/g_dump_3cam.sh)")

    report = calibrate_rig(clouds_of(head_frames(dump)))

    assert report.accepted, report.refusals
    assert len(report.heads) == 3
    # Measured 27.07: 0.47-0.49 mm across the three dumps. Asserted loosely — this
    # pins "the heads agree at the belt", not the third decimal of one run.
    assert report.spread_mm < 1.0
