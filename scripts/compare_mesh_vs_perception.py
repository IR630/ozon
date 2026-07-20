# -*- coding: utf-8 -*-
"""STL-path vs camera-path equivalence: does the depth pipeline recover the mesh's
true OBB dims, and route to the same category, on a CLEAN render of each model?

Two entry points measure the eleven organizer models differently on purpose:
`analyze_models.py` reads the mesh OBB directly (exact geometry, the reference
table `docs/md/models.md`); the production pipeline `src.perception.measure_item`
sees only a top-down depth frame and reconstructs the hidden half. This script
renders each STL off its mesh (`render_depth`, the SAME pinhole the cell uses),
feeds the real `measure_item`, and compares the result to the OBB truth with the
organizers' own accuracy rule (`within_measurement_tolerance`, 5 mm OR 10 % vol).

HONESTY LIMIT (`docs/decisions.md`, camera-probe entry 2026-07-20; render_depth's
own HONESTY LIMIT): a rendered frame has NO sensor noise, so this measures the
ALGORITHM on clean geometry, not real-frame accuracy. A model outside tolerance
here (Шлем: the dome height is reconstructed, not observed) is a genuine algorithm
limit; a model inside here may still drift on a real Gazebo frame. Category
agreement is the routing-relevant number; the mm error is the diagnostic.

    python scripts/compare_mesh_vs_perception.py            # all 11 docs/Stl
    python scripts/compare_mesh_vs_perception.py a.stl b.stl
"""
import sys
from pathlib import Path

import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from analyze_models import analyze_file  # noqa: E402
from render_depth import render_depth  # noqa: E402
from src.classification import (  # noqa: E402
    classify,
    measurement_error,
    within_measurement_tolerance,
)
from src.perception import measure_item  # noqa: E402

# Identity pose, dropped on the belt — reproducible, but NOT the settled stable pose
# the census spawns (scripts/spawn_orientations.py). Category disagreements for
# round/thin items (Тарелка, Бутылка, Ручка read B here) are dominated by this pose
# choice — those route correctly in their census poses — not by algorithm error;
# and Мешок D->B is the deliberate prod policy, not a miss. The pose-ROBUST rigid
# bodies (boxes, ЛанчБокс, Моющее, Цилиндр) are the honest equivalence subset.
RESTING = (0.0, 0.0, 0.0, 1.0)


def compare_one(path):
    """One record: OBB truth vs perception on a clean top-down render of `path`."""
    ref = analyze_file(path)  # OBB dims (mm, desc) + reference category
    truth = tuple(float(x) for x in ref["dims"])
    mesh = trimesh.load(str(path), force="mesh")
    m = measure_item(render_depth(mesh, RESTING))
    if m is None:
        return {"name": ref["name"], "detected": False, "truth": truth,
                "ref_cat": ref["category"]}
    dims = tuple(float(x) for x in m.dims_mm)
    side, vol = measurement_error(dims, truth)
    return {
        "name": ref["name"],
        "detected": True,
        "truth": truth,
        "dims": dims,
        "side_err": side,
        "vol_err": vol,
        "in_tol": within_measurement_tolerance(dims, truth),
        "ref_cat": ref["category"],
        "perc_cat": classify(dims, float(m.k)),
        "cat_match": ref["category"] == classify(dims, float(m.k)),
    }


def main(argv=None):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    paths = [Path(p) for p in (argv if argv is not None else sys.argv[1:])]
    files = paths or sorted((ROOT / "docs" / "Stl").iterdir())
    recs = [compare_one(p) for p in files]

    print(f"{'model':22} {'truth mm':>18} {'perception mm':>18} {'side':>7} {'vol%':>6} "
          f"{'tol':>4} {'ref->perc':>10}")
    in_tol = cat_ok = measured = 0
    for r in recs:
        if not r["detected"]:
            print(f"{r['name']:22} {'—':>18} {'not detected':>18}")
            continue
        measured += 1
        in_tol += r["in_tol"]
        cat_ok += r["cat_match"]
        t = "×".join(f"{d:.0f}" for d in r["truth"])
        d = "×".join(f"{x:.0f}" for x in r["dims"])
        route = f"{r['ref_cat']}->{r['perc_cat']}" + ("" if r["cat_match"] else " ✗")
        print(f"{r['name']:22} {t:>18} {d:>18} {r['side_err']:6.1f} {r['vol_err']*100:5.1f} "
              f"{'in' if r['in_tol'] else 'OUT':>4} {route:>10}")
    print(f"\norganizer tolerance {in_tol}/{measured}; category agreement {cat_ok}/{measured} "
          f"(clean IDENTITY-pose renders — see the pose caveat in-file)")
    print("routing truth for round/thin items lives in the census (164/165) and "
          "measure_validation, NOT in this identity-pose diagnostic.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
