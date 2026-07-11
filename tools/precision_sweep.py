# -*- coding: utf-8 -*-
"""Offline precision cross-check: render each of the 11 STL items as a top-down
depth frame and run the production perception + classification on it.

Purpose (day 4, P3 — WITHOUT the simulator): sweep measure_item() over every
item shape, including the ones only ever exercised in Gazebo (helmet, pouf, bag,
pen), and tabulate measured dims / K / routed category against the STL reference
(scripts/analyze_models.py). It answers "does perception blow up or mis-route on
any shape, and where does silhouette-K vs section-K land" from a laptop.

HONESTY CAVEAT (docs/decisions.md — the team moved off offline STL renders for
tuning): this synthetic depth is CLEANER than Gazebo (no sensor noise, no
physics settling, an idealized rest pose). It is a SANITY CROSS-CHECK, not the
authoritative precision table and NOT a surface to tune thresholds against — the
real numbers come from the Gazebo matrix (scripts/run_matrix.sh). The rest pose
is a proxy: the item's smallest OBB extent laid vertical (flat rest), which does
not reproduce a rocking helmet or a draped bag.

Run: python tools/precision_sweep.py            # table for all 11
     python tools/precision_sweep.py box_300x200x200   # one item
"""
import sys
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from analyze_models import analyze_file  # noqa: E402
from build_item_models import ITEMS  # noqa: E402
from src.classification import classify  # noqa: E402
from src.constants import CATEGORY_B, CATEGORY_C, CATEGORY_D  # noqa: E402
from src.perception import (  # noqa: E402
    BELT_DEPTH_M, BELT_TOP_Z_M, CAMERA_X_M, CAMERA_Y_M, CAMERA_Z_M,
    FX, FY, IMG_H, IMG_W, measure_item,
)

CX, CY = IMG_W / 2.0, IMG_H / 2.0
STL_DIR = ROOT / "docs" / "Stl"


def settle_mesh(mesh):
    """Flat-rest proxy pose in world meters: smallest OBB extent laid vertical,
    footprint centered under the camera, lowest point on the belt top."""
    m = mesh.copy()
    m.apply_scale(0.001)  # STL authored in mm; sim world is meters (SDF scale 0.001)
    m.apply_transform(np.linalg.inv(m.bounding_box_oriented.primitive.transform))
    order = np.argsort(m.extents)[::-1]  # long, mid, short -> X, Y, Z(short=vertical)
    perm = np.eye(4)
    perm[:3, :3] = np.eye(3)[order]
    m.apply_transform(perm)
    lo, hi = m.bounds
    m.apply_translation([CAMERA_X_M - (lo[0] + hi[0]) / 2,
                         CAMERA_Y_M - (lo[1] + hi[1]) / 2,
                         BELT_TOP_Z_M - lo[2]])
    return m


def render_depth(mesh):
    """Top-down perspective depth image (meters) via the same pinhole model as
    perception: project every triangle, z-buffer the nearest (top) surface.
    Background stays at the empty-belt depth."""
    v = mesh.vertices
    depth_v = CAMERA_Z_M - v[:, 2]
    u = CX - (v[:, 1] - CAMERA_Y_M) * FX / depth_v
    w = CY - (v[:, 0] - CAMERA_X_M) * FY / depth_v
    px = np.column_stack([u, w])
    zbuf = np.full((IMG_H, IMG_W), BELT_DEPTH_M)
    for tri in mesh.faces:
        p, d = px[tri], depth_v[tri]
        x0 = max(int(np.floor(p[:, 0].min())), 0)
        x1 = min(int(np.ceil(p[:, 0].max())), IMG_W - 1)
        y0 = max(int(np.floor(p[:, 1].min())), 0)
        y1 = min(int(np.ceil(p[:, 1].max())), IMG_H - 1)
        if x1 < x0 or y1 < y0:
            continue
        gx, gy = np.meshgrid(np.arange(x0, x1 + 1), np.arange(y0, y1 + 1))
        ax, ay = p[0]
        bx, by = p[1]
        cx2, cy2 = p[2]
        denom = (by - cy2) * (ax - cx2) + (cx2 - bx) * (ay - cy2)
        if abs(denom) < 1e-9:
            continue
        la = ((by - cy2) * (gx - cx2) + (cx2 - bx) * (gy - cy2)) / denom
        lb = ((cy2 - ay) * (gx - cx2) + (ax - cx2) * (gy - cy2)) / denom
        lc = 1.0 - la - lb
        inside = (la >= -1e-6) & (lb >= -1e-6) & (lc >= -1e-6)
        if not inside.any():
            continue
        pd = la * d[0] + lb * d[1] + lc * d[2]
        sub = zbuf[y0:y1 + 1, x0:x1 + 1]
        upd = inside & (pd < sub)
        sub[upd] = pd[upd]
    return zbuf


def render_item_depth(slug):
    """Depth frame (meters) of one item slug in its flat-rest pose."""
    stem = ITEMS[slug][0]
    mesh = trimesh.load(str(STL_DIR / f"{stem}.stl"), force="mesh")
    return render_depth(settle_mesh(mesh))


def sweep_item(slug):
    """Measured vs reference row for one item, or None if perception saw nothing."""
    stem = ITEMS[slug][0]
    ref = analyze_file(STL_DIR / f"{stem}.stl")
    ref_dims = sorted((float(x) for x in ref["dims"]), reverse=True)
    ref_cat = classify(ref_dims, ref["k"])
    m = measure_item(render_item_depth(slug))
    if m is None:
        return {"slug": slug, "stem": stem, "measured": None,
                "ref_dims": ref_dims, "ref_k": float(ref["k"]), "ref_cat": ref_cat}
    meas_cat = classify(m.dims_mm, m.k)
    dim_err = max(abs(a - b) for a, b in zip(m.dims_mm, ref_dims))
    return {"slug": slug, "stem": stem,
            "measured_dims": [float(x) for x in m.dims_mm], "measured_k": float(m.k),
            "measured_cat": meas_cat, "dim_err_mm": float(dim_err),
            "ref_dims": ref_dims, "ref_k": float(ref["k"]), "ref_cat": ref_cat}


def _fmt_dims(dims):
    return "×".join(f"{d:.0f}" for d in dims)


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    slugs = sys.argv[1:] or list(ITEMS)
    print("SYNTHETIC top-down render (trimesh) — sanity cross-check, NOT the Gazebo table.\n")
    header = f"{'item':<17} {'meas dims':>13} {'ref dims':>13} {'Δmm':>5}  " \
             f"{'measK':>6} {'refK':>5}  {'meas':>4} {'ref':>4}  ok"
    print(header)
    print("-" * len(header))
    rows = []
    for slug in slugs:
        r = sweep_item(slug)
        rows.append(r)
        if r.get("measured") is None and "measured_dims" not in r:
            print(f"{slug:<17} {'— no measurement —':>34}   "
                  f"{'':>6} {r['ref_k']:>5.2f}  {'':>4} {r['ref_cat']:>4}  ✗")
            continue
        ok = "✓" if r["measured_cat"] == r["ref_cat"] else "✗"
        print(f"{slug:<17} {_fmt_dims(r['measured_dims']):>13} "
              f"{_fmt_dims(r['ref_dims']):>13} {r['dim_err_mm']:>5.0f}  "
              f"{r['measured_k']:>6.3f} {r['ref_k']:>5.2f}  "
              f"{r['measured_cat']:>4} {r['ref_cat']:>4}  {ok}")
    matched = sum(1 for r in rows if r.get("measured_cat") == r.get("ref_cat"))
    print(f"\ncategory match: {matched}/{len(rows)} "
          f"(offline render; treat mismatches as leads for the Gazebo run, not verdicts)")
    _ = (CATEGORY_B, CATEGORY_C, CATEGORY_D)  # categories come from classify()


if __name__ == "__main__":
    main()
