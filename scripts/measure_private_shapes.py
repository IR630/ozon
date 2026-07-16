#!/usr/bin/env python3
"""Procedural private-set proxy through the production depth pipeline.

The organizer may evaluate unknown objects, so this gate deliberately uses no
STL, slug or lookup table from the released 11 models. Analytic silhouettes are
rendered into calibrated depth frames and passed through the same
measure_items -> classify_conservative path as the ROS nodes.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.classification import classify_conservative  # noqa: E402
from src.perception import BELT_DEPTH_M, IMG_H, IMG_W, measure_items  # noqa: E402


@dataclass(frozen=True)
class PrivateCase:
    name: str
    expected: str
    depth_m: np.ndarray


@dataclass(frozen=True)
class PrivateResult:
    name: str
    expected: str
    actual: str
    dims_mm: tuple[float, float, float]
    k: float


def _shape_depth(kind: str, long_px: float, short_px: float, yaw_deg: float, top_m: float):
    depth = np.full((IMG_H, IMG_W), BELT_DEPTH_M, dtype=float)
    rows, cols = np.mgrid[0:IMG_H, 0:IMG_W]
    dx = cols - IMG_W / 2
    dy = rows - IMG_H / 2
    yaw = np.deg2rad(yaw_deg)
    along = np.cos(yaw) * dx + np.sin(yaw) * dy
    across = -np.sin(yaw) * dx + np.cos(yaw) * dy
    if kind == "rectangle":
        mask = (np.abs(along) <= long_px / 2) & (np.abs(across) <= short_px / 2)
    elif kind == "ellipse":
        mask = (along / (long_px / 2)) ** 2 + (across / (short_px / 2)) ** 2 <= 1.0
    else:
        raise ValueError(f"unsupported procedural shape: {kind}")
    depth[mask] = top_m
    return depth


def build_cases() -> list[PrivateCase]:
    """Unknown B/C/D objects plus rotations and both size-priority traps."""
    cases = []
    for yaw in (0.0, 27.0, 61.0):
        cases.append(
            PrivateCase(
                f"rectangular_prism_yaw{yaw:.0f}",
                "B",
                _shape_depth("rectangle", 130, 82, yaw, 1.38),
            )
        )
        cases.append(
            PrivateCase(
                f"near_round_ellipse_yaw{yaw:.0f}",
                "B",
                _shape_depth("ellipse", 150, 116, yaw, 1.42),
            )
        )
        cases.append(
            PrivateCase(
                f"oversized_prism_yaw{yaw:.0f}",
                "C",
                _shape_depth("rectangle", 205, 80, yaw, 1.35),
            )
        )

    cases.extend(
        (
            PrivateCase(
                "flat_round_disc",
                "D",
                _shape_depth("ellipse", 110, 110, 0, 1.47),
            ),
            PrivateCase(
                "undersized_flat_tag",
                "C",
                _shape_depth("rectangle", 100, 60, 19, 1.491),
            ),
            PrivateCase(
                "oversized_round_disc",
                "C",
                _shape_depth("ellipse", 200, 200, 0, 1.45),
            ),
        )
    )
    return cases


def evaluate(cases: list[PrivateCase]) -> list[PrivateResult]:
    results = []
    for case in cases:
        measured = measure_items(case.depth_m)
        if len(measured) != 1:
            results.append(PrivateResult(case.name, case.expected, "NO_DETECTION", (0, 0, 0), 0.0))
            continue
        item = measured[0]
        results.append(
            PrivateResult(
                name=case.name,
                expected=case.expected,
                actual=classify_conservative(item.dims_mm, item.k),
                dims_mm=tuple(item.dims_mm),
                k=item.k,
            )
        )
    return results


def main() -> int:
    results = evaluate(build_cases())
    passed = sum(result.actual == result.expected for result in results)
    for result in results:
        dims = "x".join(f"{value:.0f}" for value in result.dims_mm)
        verdict = "PASS" if result.actual == result.expected else "FAIL"
        print(
            f"{result.name}: expected={result.expected} actual={result.actual} "
            f"dims={dims}mm K={result.k:.3f} {verdict}"
        )
    print(f"private procedural gate: {passed}/{len(results)}")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
