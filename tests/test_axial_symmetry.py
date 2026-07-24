# -*- coding: utf-8 -*-
"""The axial-symmetry route to D for a compact round body.

Before this gate the pipeline had NO route to D for a ball or an upright can: the
flatness gate vetoed every thick round silhouette to protect Мешок, and the
elongation gate closed the cross-section route. The discriminator is relief, not
outline — a body of revolution's height depends on radius alone, a slumped bag's
does not (docs/probe-models.md).
"""
from pathlib import Path

import numpy as np
import pytest

from src.classification import classify_conservative
from src.constants import CATEGORY_B
from src.perception import (
    AXIAL_SYMMETRY_MAX,
    _axial_symmetry_residual,
    load_depth_png,
    measure_item,
)

FRAMES = Path(__file__).resolve().parent / "fixtures" / "frames"
REPO = Path(__file__).resolve().parent.parent

# Every Мешок depth frame the repo holds. The branch that introduced this gate had
# only one and flagged a second pose as mandatory (a slumped bag reading round is
# exactly how solidity fooled us before); these four close that — all stay B.
BAG_FRAMES = [
    FRAMES / "bag_2_depth.png",
    REPO / "docs" / "report" / "img" / "day4_bag_depth.png",
    REPO / "docs" / "report" / "img" / "day11_bag_oi1_depth.png",
    REPO / "docs" / "report" / "img" / "day11_bag_oi2_depth.png",
]


def _disc(n=60):
    """Pixel grid over a unit disc: (xs, ys, radius) in pixel units."""
    g = np.arange(-n, n + 1, dtype=float)
    xx, yy = np.meshgrid(g, g)
    r = np.hypot(xx, yy)
    keep = r <= n
    return xx[keep], yy[keep], r[keep] / n


class TestResidual:
    def test_ball_depends_on_radius_alone(self):
        """A sphere ON THE BELT, as the camera meets it: the mask ends at the
        silhouette (radius R) and height runs 0..2R, so heights normalise by the
        DIAMETER. Getting that geometry wrong is what makes a hemisphere-shaped
        synthetic read far rougher than the real ball frame (0.0236)."""
        xs, ys, r = _disc()
        heights = 1.0 + np.sqrt(np.clip(1.0 - r**2, 0.0, None))  # 0..2, h_max = 2
        got = _axial_symmetry_residual(xs, ys, heights)
        assert got < AXIAL_SYMMETRY_MAX, f"a ball must read symmetric, got {got}"

    def test_flat_top_cylinder_is_symmetric(self):
        """An upright can: one constant height across the whole footprint. There is
        no low ring outside it — beyond the silhouette the mask stops, it does not
        continue at belt height."""
        xs, ys, _ = _disc()
        heights = np.ones(len(xs))
        assert _axial_symmetry_residual(xs, ys, heights) < AXIAL_SYMMETRY_MAX

    def test_tilted_surface_is_not_symmetric(self):
        """Height varying with ANGLE at fixed radius — the lump signature."""
        xs, ys, r = _disc()
        heights = 1.0 + 0.6 * (xs / (np.abs(xs).max()))  # ramp across the disc
        got = _axial_symmetry_residual(xs, ys, heights)
        assert got > AXIAL_SYMMETRY_MAX, f"a ramp must not read symmetric, got {got}"

    def test_degenerate_input_returns_none(self):
        xs = np.zeros(20)
        assert _axial_symmetry_residual(xs, xs, np.zeros(20)) is None


class TestRealFrames:
    """The gate may not drag the bag into D — that is what it is balanced against."""

    @pytest.mark.parametrize("frame", BAG_FRAMES, ids=lambda p: p.stem)
    def test_bag_stays_out_of_d(self, frame):
        m = measure_item(load_depth_png(frame))
        assert m is not None
        assert classify_conservative(m.dims_mm, m.k) == CATEGORY_B, (
            f"Мешок must stay B on {frame.name}, got K={m.k}")

    @pytest.mark.parametrize("frame", ["helmet_2_depth", "helmet_tilt_depth"])
    def test_helmet_stays_out_of_d(self, frame):
        """The dome is MORE axially symmetric than the bag (0.055 vs 0.070); it is
        held out of D by the K gate upstream, never by symmetry. Locks that order."""
        m = measure_item(load_depth_png(FRAMES / f"{frame}.png"))
        assert m is not None
        assert classify_conservative(m.dims_mm, m.k) == CATEGORY_B, (
            f"Шлем must stay B, got K={m.k}")
