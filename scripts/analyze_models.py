# -*- coding: utf-8 -*-
"""Analyze STL models: dimensions, volume, watertightness, circle-in-section criterion.

Classification per task rules (docs/md/task.md, section 2):
  1. Gabarits: fits if sorted-desc dims are within (10x10x10, 450x320x320) mm.
  2. Circle-in-section: K = r_inscribed / R_circumscribed of a cross-section > 0.8 -> round.
     Approximated via the max over principal-axis cross-sections (convex hull of section,
     R = max distance from centroid, r = min distance from centroid to hull edges).
"""
import sys
from pathlib import Path

import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.classification import classify as classify_category  # noqa: E402
from src.constants import CATEGORY_B, CATEGORY_C, CATEGORY_D  # noqa: E402

CAT_B = f"{CATEGORY_B}: подходит для сортировки"
CAT_C = f"{CATEGORY_C}: не подходит по габаритам"
CAT_D = f"{CATEGORY_D}: доупаковка (круг в сечении)"
_LABELS = {CATEGORY_B: CAT_B, CATEGORY_C: CAT_C, CATEGORY_D: CAT_D}


def classify(dims, k):
    """Human-readable label for the report; rules live in src/classification.py."""
    return _LABELS[classify_category(dims, k)]


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
    except Exception:
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
    """Max K over cross-sections along 3 OBB axes at several offsets."""
    m = mesh.copy()
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


def analyze_file(path):
    """Full geometric analysis of one STL file."""
    m = trimesh.load(str(path), force="mesh")
    dims = np.sort(m.bounding_box_oriented.primitive.extents)[::-1]
    k = max_circle_ratio(m)
    return {
        "name": Path(path).stem,
        "file": Path(path).name,
        "dims": dims,
        "volume": m.volume if m.is_watertight else None,
        "watertight": m.is_watertight,
        "faces": len(m.faces),
        "k": k,
        "cat": classify(dims, k),
    }


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    rows = []
    for f in sorted(Path("docs/Stl").iterdir()):
        r = analyze_file(f)
        rows.append(r)
        print(f"{r['name']}: OBB {r['dims'].round(1)} K={r['k']:.3f} "
              f"watertight={r['watertight']} faces={r['faces']} -> {r['cat']}")

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
        vol = f"{r['volume'] / 1e6:.2f}" if r["volume"] else "—"
        wt = "да" if r["watertight"] else "нет"
        lines.append(f"| {r['name']} | {d} | {vol} | {wt} | {r['faces']} | {r['k']:.2f} | {r['cat']} |")
    lines += [
        "",
        "## Пограничные и особые случаи",
        "",
        "- **Цилиндр (K=0.74)** — это не гладкий цилиндр, а корпус с четырьмя продольными",
        "  стяжками и квадратными торцами (~50×43 мм). Выпуклая оболочка сечения — скруглённый",
        "  квадрат, поэтому по формальному критерию K < 0,8 объект **не** круглый → B.",
        "  Явно заложенный организаторами пограничный кейс: «называется цилиндром, но по критерию не круг».",
        "- **Шлем (K=0.78)** — купол почти круглый в горизонтальном сечении, K вплотную к порогу 0,8.",
        "  Результат чувствителен к способу вычисления сечения; требует аккуратной проверки",
        "  и обоснования в отчёте.",
        "- **Ручка (K=0.99, 148 × 13 × 9 мм)** — круглая в сечении, **но** минимальный габарит",
        "  9 мм < 10 мм → по приоритету правил уходит в C (габариты важнее формы).",
        "- **Пуфик (K=1.00, 489 мм)** — круглый, но 489 > 450 мм → по приоритету правил C, не D.",
        "- **Короб 400х400х300 (401 × 400 × 300)** — превышает 450 × 320 × 320 по второму",
        "  и третьему габариту (400 > 320) → C.",
        "- Габариты коробов на ~1 мм больше названий (301 против 300) — это толщина стенок",
        "  в модели; при проверке порогов стоит помнить про допуск.",
        "",
        "## Ограничения сортировщика (для справки)",
        "",
        "- Минимум: 10 × 10 × 10 мм; максимум: 450 × 320 × 320 мм.",
        "- Круг в сечении: r_впис / R_опис > 0,8 → категория D (доупаковка).",
        "- Приоритет: сначала габариты (C), затем форма (D), иначе B.",
        "",
        "*Сгенерировано скриптом анализа. STEP-модели (`docs/Step/`) — точная",
        "геометрия тех же объектов для CAD/аналитической проверки.*",
    ]
    Path("docs/md/models.md").write_text("\n".join(lines), encoding="utf-8")
    print("\ndocs/md/models.md written")


if __name__ == "__main__":
    main()
