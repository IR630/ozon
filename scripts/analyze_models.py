# -*- coding: utf-8 -*-
"""Analyze STL models: dimensions, volume, watertightness, circle-in-section criterion.

Classification per task rules:
  1. Gabarits: fits if min-extent sorted dims > 10x10x10 and < 450x320x320 (sorted desc compare).
  2. Circle-in-section: K = r_inscribed / R_circumscribed of a cross-section > 0.8 -> round.
     Approximated via the max over principal-axis cross-sections (convex hull of section,
     R = max distance from centroid, r = min distance from centroid to hull edges).
"""
import sys
from pathlib import Path
import numpy as np
import trimesh

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

MAX_DIMS = np.array([450, 320, 320])  # sorted desc
MIN_DIMS = np.array([10, 10, 10])


def section_circle_ratio(mesh, origin, normal):
    """K = r_inscribed/R_circumscribed for one planar cross-section (convex approximation)."""
    try:
        sec = mesh.section(plane_origin=origin, plane_normal=normal)
        if sec is None:
            return None
        planar, _ = sec.to_2D()
        pts = np.asarray(planar.vertices)
        if len(pts) < 3:
            return None
    except Exception as e:
        print(f"    section fail ({normal}, {origin.round(1)}): {type(e).__name__}: {e}", file=sys.stderr)
        return None
    from scipy.spatial import ConvexHull
    try:
        h = ConvexHull(pts)
    except Exception:
        return None
    hp = pts[h.vertices]
    c = hp.mean(axis=0)
    R = np.linalg.norm(hp - c, axis=1).max()
    # r = min distance from centroid to hull edges
    r = np.inf
    n = len(hp)
    for i in range(n):
        a, b = hp[i], hp[(i + 1) % n]
        ab = b - a
        t = np.clip(np.dot(c - a, ab) / np.dot(ab, ab), 0, 1)
        d = np.linalg.norm(c - (a + t * ab))
        r = min(r, d)
    if not np.isfinite(r) or R == 0:
        return None
    return r / R


def max_circle_ratio(mesh):
    """Max K over cross-sections along 3 principal axes at several offsets."""
    m = mesh.copy()
    # align to principal axes via OBB
    m.apply_transform(np.linalg.inv(m.bounding_box_oriented.primitive.transform))
    lo, hi = m.bounds
    best = 0.0
    for axis in range(3):
        normal = np.zeros(3)
        normal[axis] = 1.0
        for frac in [0.25, 0.4, 0.5, 0.6, 0.75]:
            origin = np.zeros(3)
            origin[axis] = lo[axis] + frac * (hi[axis] - lo[axis])
            k = section_circle_ratio(m, origin, normal)
            if k:
                best = max(best, k)
    return best


rows = []
for f in sorted(Path("docs/Stl").iterdir()):
    m = trimesh.load(str(f), force="mesh")
    # oriented bounding box extents = real dims regardless of orientation in file
    obb = m.bounding_box_oriented.primitive.extents
    dims = np.sort(obb)[::-1]  # desc
    fits_max = bool(np.all(dims < MAX_DIMS))
    fits_min = bool(np.all(np.sort(m.bounding_box_oriented.primitive.extents) > MIN_DIMS))
    k = max_circle_ratio(m)
    if not (fits_max and fits_min):
        cat = "C: не подходит по габаритам"
    elif k > 0.8:
        cat = "D: доупаковка (круг в сечении)"
    else:
        cat = "B: подходит для сортировки"
    rows.append({
        "name": f.stem,
        "file": f.name,
        "dims": dims,
        "aabb": np.sort(m.extents)[::-1],
        "volume": m.volume if m.is_watertight else None,
        "watertight": m.is_watertight,
        "faces": len(m.faces),
        "k": k,
        "cat": cat,
    })
    print(f"{f.stem}: OBB {dims.round(1)} K={k:.3f} watertight={m.is_watertight} faces={len(m.faces)} -> {cat}")

# sanity check against box names
print("\nПроверка: Короб 300х200х200 ->", [r["dims"].round(0) for r in rows if "300" in r["name"]])
print("Проверка: Короб 400х400х300 ->", [r["dims"].round(0) for r in rows if "400" in r["name"]])

# write models.md
lines = [
    "# Тестовый набор 3D-моделей: геометрический анализ",
    "",
    "> Автоматический анализ STL из `docs/Stl/` (trimesh). Габариты — по ориентированному",
    "> ограничивающему боксу (OBB), мм, по убыванию. K — максимальный коэффициент",
    "> r_впис/R_опис по сечениям вдоль главных осей (порог круга: K > 0,8).",
    "> Категория — предварительная оценка по правилам классификации из task.md;",
    "> пограничные случаи требуют ручной проверки.",
    "",
    "| Модель | Габариты OBB, мм | Объём, л | Watertight | Треуг. | K (круг) | Категория (оценка) |",
    "|---|---|---|---|---|---|---|",
]
for r in rows:
    d = " × ".join(f"{x:.0f}" for x in r["dims"])
    vol = f"{r['volume']/1e6:.2f}" if r["volume"] else "—"
    wt = "да" if r["watertight"] else "нет"
    lines.append(f"| {r['name']} | {d} | {vol} | {wt} | {r['faces']} | {r['k']:.2f} | {r['cat']} |")
lines += [
    "",
    "## Ограничения сортировщика (для справки)",
    "",
    "- Минимум: 10 × 10 × 10 мм; максимум: 450 × 320 × 320 мм.",
    "- Круг в сечении: r_впис / R_опис > 0,8 → категория D (доупаковка).",
    "- Приоритет: сначала габариты (C), затем форма (D), иначе B.",
    "",
    "*Сгенерировано скриптом анализа, 2026-07-10. STEP-модели (`docs/Step/`) — точная",
    "геометрия тех же объектов для CAD/аналитической проверки.*",
]
Path("docs/md/models.md").write_text("\n".join(lines), encoding="utf-8")
print("\ndocs/md/models.md written")
