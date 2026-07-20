# -*- coding: utf-8 -*-
"""Analyze STL models: dimensions, volume, watertightness, circle-in-section criterion.

Classification per task rules (docs/md/task.md, section 2):
  1. Gabarits: fits if sorted-desc dims are within (10x10x10, 450x320x320) mm.
  2. Roundness: K = r_inscribed / R_circumscribed > 0.8 -> round. K is taken over the
     three OBB-axis PROJECTIONS (XY/XZ/YZ) — the rule as the experts stated it
     (docs/md/expert_session_qa.md [20:34], [52:36], [58:02]) — using the SAME
     estimator as the production depth pipeline (src.perception._roundness_k, both
     radii from one common centre). Reference and production must not measure
     differently; they did until 2026-07-19, see docs/decisions.md.

The cross-section K is still reported as a diagnostic, because task.md's WRITTEN
wording says "круг в любом из сечений" while the experts answered "по проекции",
and the two disagree on Шлем (0.78 by projection -> B, 0.84 by section -> D). That
contradiction is an open question for the organizers, not something to bury.
"""
import sys
from pathlib import Path

import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.classification import classify as classify_category  # noqa: E402
from src.constants import (  # noqa: E402
    CATEGORY_B,
    CATEGORY_C,
    CATEGORY_D,
    SANE_DIM_MM_MAX,
    SANE_DIM_MM_MIN,
)

CAT_B = f"{CATEGORY_B}: подходит для сортировки"
CAT_C = f"{CATEGORY_C}: не подходит по габаритам"
CAT_D = f"{CATEGORY_D}: доупаковка (круг в сечении)"
_LABELS = {CATEGORY_B: CAT_B, CATEGORY_C: CAT_C, CATEGORY_D: CAT_D}


def classify(dims, k):
    """Human-readable label for the report; rules live in src/classification.py."""
    return _LABELS[classify_category(dims, k)]


# The category->executive contract the live controller acts on
# (src/controller_node.py: C -> /pusher_c/cmd, D -> /pusher_d/cmd, B rides to the
# belt end). Mirrored here so the STL entry point closes the contour end to end —
# load -> params -> classify -> HAND OFF to execution — which the expert asked for
# (docs/md/expert_session_qa.md [35:29]-[36:12]); a simulated hand-off is allowed.
_EXEC_HANDOFF = {
    CATEGORY_B: "категория B — едет до конца ленты, без дивёрта",
    CATEGORY_C: "категория C — дивёртер зоны C (/pusher_c/cmd)",
    CATEGORY_D: "категория D — доупаковка, дивёртер зоны D (/pusher_d/cmd)",
}


def executive_handoff(category):
    """Routing command the executive part would receive for a raw B/C/D category."""
    return _EXEC_HANDOFF[category]


def check_scale(dims_mm, name=""):
    """Fail LOUD on an implausibly-scaled mesh (Karpathy #6: no silent wrong answer).

    STL carries no units. A metre-scale export (300 mm -> 0.3) or a 10x-inflated one
    would pass silently through every mm threshold and misroute the item. Real items
    span 9-489 mm, well inside the sane per-dim band [SANE_DIM_MM_MIN, SANE_DIM_MM_MAX];
    anything outside it is almost certainly a unit error, not a real product.
    """
    lo, hi = float(min(dims_mm)), float(max(dims_mm))
    if lo < SANE_DIM_MM_MIN or hi > SANE_DIM_MM_MAX:
        shown = tuple(round(float(d), 1) for d in dims_mm)
        raise ValueError(
            f"{name or 'mesh'} dims {shown} mm are outside the sane range "
            f"[{SANE_DIM_MM_MIN}, {SANE_DIM_MM_MAX}] mm — looks like a unit/scale error "
            f"(metres? inches?). STL has no units; rescale the model to millimetres.")


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
    """Full geometric analysis of one STL file.

    `k` decides the category (projections, the experts' rule); `k_section` rides
    along as the diagnostic for the written-wording reading — see module docstring.
    """
    from scripts.compare_k_rules import k_by_projection, k_by_section

    m = trimesh.load(str(path), force="mesh")
    dims = np.sort(m.bounding_box_oriented.primitive.extents)[::-1]
    check_scale(dims, Path(path).stem)
    m.apply_transform(np.linalg.inv(m.bounding_box_oriented.primitive.transform))
    k, _ = k_by_projection(m)
    return {
        "name": Path(path).stem,
        "file": Path(path).name,
        "dims": dims,
        "volume": m.volume if m.is_watertight else None,
        "watertight": m.is_watertight,
        "faces": len(m.faces),
        "k": k,
        "k_section": k_by_section(m),
        "category": classify_category(dims, k),
        "cat": classify(dims, k),
    }


def main(argv=None):
    """No args: analyze docs/Stl and regenerate models.md (the reference run).

    With STL paths: analyze just those and print the verdict — this is the
    "expert loads their own model" entry point the organizers asked for
    (docs/md/expert_session_qa.md, [35:29]). models.md is NOT touched then,
    so an ad-hoc check can never overwrite the reference table.
    """
    import argparse

    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("paths", nargs="*", type=Path,
                        help="STL files to classify (default: the docs/Stl reference set)")
    args = parser.parse_args(argv)

    for p in args.paths:
        if not p.is_file():
            parser.error(f"not a file: {p}")

    files = args.paths or sorted(Path("docs/Stl").iterdir())
    rows = []
    for f in files:
        r = analyze_file(f)
        rows.append(r)
        print(f"{r['name']}: OBB {r['dims'].round(1)} K={r['k']:.3f} "
              f"watertight={r['watertight']} faces={r['faces']} -> {r['cat']}")
        print(f"    -> исполнительная часть: {executive_handoff(r['category'])}")

    if args.paths:  # ad-hoc run: report only, keep the reference table intact
        return 0

    lines = [
        "# Тестовый набор 3D-моделей: геометрический анализ",
        "",
        "> Автоматический анализ STL из `docs/Stl/` (trimesh). Габариты — по ориентированному",
        "> ограничивающему боксу (OBB), мм, по убыванию. K — максимальный коэффициент",
        "> r_впис/R_опис по трём проекциям OBB (XY/XZ/YZ) — правило в формулировке",
        "> экспертов; обе окружности из ОДНОГО центра, тем же оценщиком, что и прод",
        "> (`src.perception._roundness_k`). Порог круга: K > 0,8.",
        "> Колонка «K сечен» — диагностика: письменная формулировка task.md говорит",
        "> «круг в любом из сечений», и на Шлеме два чтения расходятся (B против D).",
        "> Категория — предварительная оценка по правилам классификации из task.md;",
        "> пограничные случаи требуют ручной проверки.",
        "",
        "| Модель | Габариты OBB, мм | Объём, л | Watertight | Треуг. | K (проекции) | K сечен | Категория (оценка) |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        d = " × ".join(f"{x:.0f}" for x in r["dims"])
        vol = f"{r['volume'] / 1e6:.2f}" if r["volume"] else "—"
        wt = "да" if r["watertight"] else "нет"
        lines.append(f"| {r['name']} | {d} | {vol} | {wt} | {r['faces']} | {r['k']:.2f} | "
                     f"{r['k_section']:.2f} | {r['cat']} |")
    lines += [
        "",
        "## Пограничные и особые случаи",
        "",
        "- **Цилиндр (K=0.75)** — это не гладкий цилиндр, а корпус с четырьмя продольными",
        "  стяжками и квадратными торцами (~50×43 мм). Выпуклая оболочка проекции — скруглённый",
        "  квадрат, поэтому по формальному критерию K < 0,8 объект **не** круглый → B.",
        "  Явно заложенный организаторами пограничный кейс: «называется цилиндром, но по критерию не круг».",
        "  Устойчив к способу счёта: по сечениям 0,74 — тот же вердикт.",
        "- **Шлем (K=0.78 по проекциям → B, но 0.84 по сечениям → D)** — САМЫЙ РИСКОВАННЫЙ",
        "  объект набора. Купол почти круглый, и вердикт зависит от того, читать ли критерий",
        "  по проекции (так ответили эксперты) или по сечению (так написано в task.md).",
        "  Вопрос вынесен организаторам, см. `docs/md/expert_session_qa.md`.",
        "- **Мешок (K=0.8004)** — на волосок ВЫШЕ порога, то есть по чистой геометрии формально D.",
        "  Запас 0,0004 лежит внутри любой погрешности измерения, так что этот вердикт —",
        "  подбрасывание монеты. Прод сознательно удерживает Мешок в **B**: мягкий ком не катится,",
        "  а его выпуклая оболочка круглая лишь потому, что сглаживает мятый контур, — поэтому",
        "  силуэтная K ограничивается порогом для НЕплоских тел (`src/perception.py`,",
        "  провенанс в `docs/decisions.md`). Таблица показывает геометрию, прод — политику;",
        "  расхождение намеренное и задокументированное, а не рассинхрон.",
        "- **Ручка (K=0.59 по проекциям, 0.99 по сечениям, 148 × 13 × 9 мм)** — ствол 13×9 мм",
        "  сплющен, поэтому в проекции это не круг; круглым он выглядит только в отдельных",
        "  сечениях. В любом случае минимальный габарит 9 мм < 10 мм → C: габариты важнее формы.",
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
