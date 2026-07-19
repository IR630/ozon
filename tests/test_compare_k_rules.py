# -*- coding: utf-8 -*-
"""K measured by projection vs by cross-section, and the inscribed-radius trap.

The 2026-07-19 expert session fixed the rule as "inscribed/circumscribed of a
PROJECTION, three projections, round if any". These tests pin the estimator's
behaviour on shapes whose K is known analytically, and lock the finding that the
centroid-distance inscribed radius (scripts/analyze_models.py, the source of the
models.md reference table) systematically UNDERSTATES K on asymmetric outlines.
"""
import numpy as np
import pytest
import trimesh

from scripts.analyze_models import section_circle_ratio
from scripts.compare_k_rules import _k_of_points, k_by_projection, k_by_section


def _obb_frame(mesh):
    m = mesh.copy()
    m.apply_transform(np.linalg.inv(m.bounding_box_oriented.primitive.transform))
    return m


class TestKEstimator:
    def test_circle_reads_round(self):
        t = np.linspace(0, 2 * np.pi, 256, endpoint=False)
        k = _k_of_points(np.c_[np.cos(t), np.sin(t)])
        assert k == pytest.approx(1.0, abs=0.01), f"circle must read K~1, got {k}"

    def test_square_reads_its_analytic_ratio(self):
        # inscribed r = a/2, circumscribed R = a/sqrt(2)  ->  K = 1/sqrt(2)
        sq = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
        assert _k_of_points(sq) == pytest.approx(1 / np.sqrt(2), abs=0.01)

    def test_degenerate_input_returns_none(self):
        assert _k_of_points(np.array([[0.0, 0.0], [1.0, 1.0]])) is None


class TestProjectionVsSection:
    def test_cylinder_is_round_both_ways(self):
        m = _obb_frame(trimesh.creation.cylinder(radius=50.0, height=200.0))
        k_proj, _ = k_by_projection(m)
        assert k_proj == pytest.approx(1.0, abs=0.02)
        assert k_by_section(m) == pytest.approx(1.0, abs=0.02)

    def test_box_is_not_round_either_way(self):
        m = _obb_frame(trimesh.creation.box(extents=(300.0, 200.0, 100.0)))
        k_proj, _ = k_by_projection(m)
        assert k_proj < 0.8, f"a box must not read round: K={k_proj}"
        assert k_by_section(m) < 0.8

    def test_projection_reports_all_three_axes(self):
        m = _obb_frame(trimesh.creation.box(extents=(300.0, 200.0, 100.0)))
        _, per_axis = k_by_projection(m)
        assert len(per_axis) == 3 and all(k is not None for k in per_axis)


class TestInscribedRadiusDefinition:
    """The estimator choice moves K more than section-vs-projection does."""

    def test_centroid_radius_understates_k_on_asymmetric_outline(self):
        # A dome: flat bottom, round top. Centroid sits well above the flat edge,
        # so the centroid->edge distance is far below the true inscribed radius.
        t = np.linspace(0, np.pi, 128)
        dome = np.c_[np.cos(t), np.sin(t)]
        dome = np.vstack([dome, [[-1.0, 0.0]]])
        k_chebyshev = _k_of_points(dome)

        hp = dome[np.argsort(np.arctan2(dome[:, 1], dome[:, 0]))]
        c = hp.mean(axis=0)
        r_centroid = min(
            np.linalg.norm(c - (a + np.clip(np.dot(c - a, b - a) / np.dot(b - a, b - a), 0, 1) * (b - a)))
            for a, b in zip(hp, np.roll(hp, -1, axis=0))
        )
        k_centroid = r_centroid / np.linalg.norm(hp - c, axis=1).max()

        assert k_chebyshev > k_centroid, (
            f"Chebyshev must not understate: {k_chebyshev:.3f} vs {k_centroid:.3f}")

    def test_symmetric_shape_hides_the_difference(self):
        """Why the gap went unnoticed: on a symmetric section both agree."""
        cyl = _obb_frame(trimesh.creation.cylinder(radius=50.0, height=200.0))
        normal = np.array([0.0, 0.0, 1.0])
        k_centroid = section_circle_ratio(cyl, np.zeros(3), normal)
        k_chebyshev = k_by_section(cyl)
        assert k_centroid == pytest.approx(k_chebyshev, abs=0.02)
