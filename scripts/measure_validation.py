# -*- coding: utf-8 -*-
"""Measure the frozen real-Gazebo validation slices with the production baseline.

The RGB files are kept for human audit; the current baseline intentionally uses
depth only. Ground truth is the organizer-provided STL analysis in
docs/md/models.md. Run from the repository root:

    python3 scripts/measure_validation.py
"""
from dataclasses import dataclass
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.classification import classify_conservative  # noqa: E402
from src.perception import load_depth_png, measure_item  # noqa: E402


@dataclass(frozen=True)
class ValidationCase:
    name: str
    depth: Path
    rgb: Path | None
    truth_dims_mm: tuple[float, float, float] | None
    truth_k: float | None
    expected_category: str | None
    expected_detected: bool = True


@dataclass(frozen=True)
class Evaluation:
    detected: bool
    dims_mm: tuple[float, float, float] | None
    max_dim_error_mm: float | None
    k: float | None
    k_error: float | None
    category: str | None
    passed: bool


IMG = ROOT / "docs" / "report" / "img"
FIXTURES = ROOT / "tests" / "fixtures" / "frames"
CASES = (
    ValidationCase(
        "Bag oi1 / K-straddle",
        IMG / "day11_bag_oi1_depth.png",
        None,
        (202.0, 176.0, 170.0),
        0.72,
        "B",
    ),
    ValidationCase(
        "Bag oi2 / alternate pose",
        IMG / "day11_bag_oi2_depth.png",
        None,
        (202.0, 176.0, 170.0),
        0.72,
        "B",
    ),
    ValidationCase(
        "Helmet oi2 / near K=0.8",
        IMG / "day11_helmet_oi2_depth.png",
        None,
        (352.0, 298.0, 282.0),
        0.78,
        "B",
    ),
    ValidationCase(
        "Bottle side / hidden round section",
        FIXTURES / "bottle_side_depth.png",
        None,
        (301.0, 91.0, 90.0),
        1.00,
        "D",
    ),
    ValidationCase(
        "Helmet tilted / 3D body OBB",
        FIXTURES / "helmet_tilt_depth.png",
        None,
        (352.0, 298.0, 282.0),
        0.78,
        "B",
    ),
    ValidationCase(
        "Plate oi1 / round flat slice",
        IMG / "day11_plate_oi1_depth.png",
        None,
        (210.0, 209.0, 27.0),
        1.00,
        "D",
    ),
    ValidationCase(
        "Pen / 9 mm minimum",
        IMG / "validation_pen" / "depth_000.png",
        IMG / "validation_pen" / "rgb_000.png",
        (148.0, 13.0, 9.0),
        0.99,
        "C",
    ),
    ValidationCase(
        "Partial box / border reject",
        IMG / "validation_partial_box" / "depth_000.png",
        IMG / "validation_partial_box" / "rgb_000.png",
        None,
        None,
        None,
        expected_detected=False,
    ),
)


def evaluate(case: ValidationCase, measurement) -> Evaluation:
    """Compare one production measurement with the frozen slice contract."""
    if measurement is None:
        return Evaluation(False, None, None, None, None, None,
                          passed=not case.expected_detected)

    dims = tuple(float(x) for x in measurement.dims_mm)
    category = classify_conservative(dims, measurement.k)
    dim_error = None
    if case.truth_dims_mm is not None:
        dim_error = max(abs(actual - truth)
                        for actual, truth in zip(dims, case.truth_dims_mm))
    k_error = None if case.truth_k is None else abs(float(measurement.k) - case.truth_k)
    passed = case.expected_detected and category == case.expected_category
    return Evaluation(
        True,
        dims,
        dim_error,
        float(measurement.k),
        k_error,
        category,
        passed,
    )


def _fmt(value, digits=2):
    return "—" if value is None else f"{value:.{digits}f}"


def main() -> int:
    results = []
    for case in CASES:
        if not case.depth.exists():
            raise FileNotFoundError(case.depth)
        if case.rgb is not None and not case.rgb.exists():
            raise FileNotFoundError(case.rgb)
        results.append((case, evaluate(case, measure_item(load_depth_png(case.depth)))))

    print("| Slice | Detection | Measured dims, mm | max \\|Δdim\\|, mm | K | \\|ΔK\\| | Route | Result |")
    print("|---|---:|---|---:|---:|---:|---:|---:|")
    for case, result in results:
        dims = "—" if result.dims_mm is None else "×".join(
            f"{value:.0f}" for value in result.dims_mm)
        detection = "detected" if result.detected else "rejected"
        route = result.category or "—"
        print(
            f"| {case.name} | {detection} | {dims} | "
            f"{_fmt(result.max_dim_error_mm, 1)} | {_fmt(result.k)} | "
            f"{_fmt(result.k_error)} | {route} | {'PASS' if result.passed else 'FAIL'} |"
        )

    visible = [(case, result) for case, result in results if case.expected_detected]
    detected = sum(result.detected for _, result in visible)
    correct = sum(result.passed for _, result in visible)
    partial = [(case, result) for case, result in results if not case.expected_detected]
    rejected = sum(result.passed for _, result in partial)
    print(
        f"\nvisible recall {detected}/{len(visible)}; "
        f"category {correct}/{len(visible)}; "
        f"partial reject {rejected}/{len(partial)}"
    )
    return 0 if all(result.passed for _, result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
