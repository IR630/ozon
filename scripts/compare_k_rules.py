# -*- coding: utf-8 -*-
"""How far is our K from the rule the jury actually stated?

The 2026-07-19 expert session answered the question we had guessed at: K is the
inscribed/circumscribed ratio of a PROJECTION, and three projections are checked
(XY, XZ, YZ) — round iff any one of them qualifies. Our STL reference path
(scripts/analyze_models.py) instead maximises K over planar CROSS-SECTIONS along
the OBB axes.

This script measures the gap on the 11 released models instead of arguing about
it: same K estimator for both (src.perception._roundness_k — the production one),
so the only difference is section-vs-projection. It changes no rule and no
verdict; it prints the number the team needs to decide with.

    python3 scripts/compare_k_rules.py
"""
from pathlib import Path
import sys

import numpy as np
import trimesh
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.classification import classify  # noqa: E402
from src.constants import ROUND_K_THRESHOLD  # noqa: E402
from src.perception import _roundness_k  # noqa: E402

SECTION_FRACS = (0.25, 0.4, 0.5, 0.6, 0.75)  # same offsets analyze_models.py uses


def _k_of_points(pts):
    """K of a 2-D point cloud's convex hull, or None if degenerate."""
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 3:
        return None
    try:
        hull = ConvexHull(pts)
    except Exception:
        return None
    return _roundness_k(pts, hull)


def k_by_projection(mesh):
    """Jury rule: max K over the three OBB-axis projections (XY, XZ, YZ)."""
    best, per_axis = 0.0, []
    for axis in range(3):
        keep = [i for i in range(3) if i != axis]
        k = _k_of_points(mesh.vertices[:, keep])
        per_axis.append(k)
        if k is not None:
            best = max(best, k)
    return best, per_axis


def k_by_section(mesh):
    """Our current STL rule: max K over cross-sections along the OBB axes."""
    lo, hi = mesh.bounds
    best = 0.0
    for axis in range(3):
        normal = np.zeros(3)
        normal[axis] = 1.0
        for frac in SECTION_FRACS:
            origin = np.zeros(3)
            origin[axis] = lo[axis] + frac * (hi[axis] - lo[axis])
            try:
                sec = mesh.section(plane_origin=origin, plane_normal=normal)
                if sec is None:
                    continue
                planar, _ = sec.to_2D()
            except Exception:
                continue
            k = _k_of_points(np.asarray(planar.vertices))
            if k is not None:
                best = max(best, k)
    return best


def analyze(path):
    """Both K's and both verdicts for one STL, in the OBB frame."""
    mesh = trimesh.load(str(path), force="mesh")
    mesh.apply_transform(np.linalg.inv(mesh.bounding_box_oriented.primitive.transform))
    dims = np.sort(mesh.bounding_box_oriented.primitive.extents)[::-1]
    k_proj, per_axis = k_by_projection(mesh)
    k_sec = k_by_section(mesh)
    return {
        "name": Path(path).stem,
        "dims": dims,
        "k_proj": k_proj,
        "k_proj_axes": per_axis,
        "k_sec": k_sec,
        "cat_proj": classify(dims, k_proj),
        "cat_sec": classify(dims, k_sec),
    }


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    rows = [analyze(f) for f in sorted((ROOT / "docs" / "Stl").iterdir())]

    print(f"{'модель':<22} {'K проекц':>9} {'K сечен':>9} {'Δ':>7}  проекц  сечен")
    for r in rows:
        flag = "  <-- расходятся" if r["cat_proj"] != r["cat_sec"] else ""
        print(f"{r['name']:<22} {r['k_proj']:>9.3f} {r['k_sec']:>9.3f} "
              f"{r['k_proj'] - r['k_sec']:>+7.3f}  {r['cat_proj']:^6} {r['cat_sec']:^5}{flag}")

    diverged = [r for r in rows if r["cat_proj"] != r["cat_sec"]]
    print(f"\nпорог круга K > {ROUND_K_THRESHOLD}")
    print(f"расхождений по категории: {len(diverged)} из {len(rows)}")
    for r in diverged:
        print(f"  {r['name']}: проекции -> {r['cat_proj']}, сечения -> {r['cat_sec']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
