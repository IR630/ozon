# -*- coding: utf-8 -*-
"""Organizer measurement tolerance of a census, recomputed from its own logs.

The census (`scripts/run_matrix.sh`) already measures every cell through the
PRODUCTION contour and prints the result — `item 1: 347x299x278 mm K=0.70 at
(1.87, 0.08) heads=2`. That line is the measurement the jury's accuracy rule
applies to, so the tolerance number needs no new Gazebo run: parse the logs,
take mesh truth from the STL, apply `within_measurement_tolerance`.

WHAT THIS NUMBER IS AND IS NOT. It is measurement accuracy of an item IN FLIGHT
on the belt, through the full contour — not routing (the census reports that
separately) and not a static bench measurement. A neighbouring branch reports
27/33 and 30/33 from a different protocol (items spawned statically in their
resting rpy, frames dumped and fed to `measure_items` offline); those numbers
are NOT comparable cell-for-cell with these. Comparable here is one census
directory against another, because the protocol is identical.

ONE MEASUREMENT PER CELL. Some cells log several measurement lines — the same
item seen on consecutive frames as it travels. The headline counts the LAST line
of each cell, the freshest measurement before the verdict; the per-measurement
count over every line rides along so a cell cannot hide a drifting reading.

    python3 scripts/census_tolerance.py                       # the four default runs
    python3 scripts/census_tolerance.py runs/census_2cam_clean
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from analyze_models import analyze_file  # noqa: E402
from build_item_models import ITEMS, STL_DIR  # noqa: E402
from src.classification import (  # noqa: E402
    measurement_error,
    within_measurement_tolerance,
)

# `item 1: 347x299x278 mm K=0.70 at (1.87, 0.08) heads=2`. `heads=` is matched
# separately because only the multi-head node prints it — the single-camera
# baseline logs the same line without it and must parse as one head.
MEASURE_RE = re.compile(r"item \d+: (\d+)x(\d+)x(\d+) mm K=([\d.]+)")
HEADS_RE = re.compile(r"heads=(\d+)")

DEFAULT_RUNS = ("runs/b0_census_9cd6312", "runs/census_2cam_clean",
                "runs/census_2cam_miscal", "runs/census_3cam_clean_fix2")


def cell_measurements(log_path):
    """[(dims_mm desc, k, heads)] logged for one census cell, in frame order."""
    out = []
    for line in log_path.read_text(encoding="utf-8", errors="replace").splitlines():
        m = MEASURE_RE.search(line.rstrip())
        if m is None:
            continue
        dims = sorted((float(m.group(i)) for i in (1, 2, 3)), reverse=True)
        h = HEADS_RE.search(line)
        out.append((dims, float(m.group(4)), int(h.group(1)) if h else 1))
    return out


def census_cells(run_dir):
    """[(slug, orientation index, measurements)] of every cell of a census run.

    `summary.log` is the run's own roll-up, not a cell, and is skipped. A cell
    whose retained log excerpt carries no measurement line is kept with an empty
    list so it cannot silently shrink the denominator: those cells routed and
    PASSed, the perception line simply is not in what the runner kept, and WHICH
    cells lose it differs from run to run. That is why runs are compared on the
    intersection of their measured cells, not on their raw ratios.
    """
    cells = []
    for log_path in sorted(Path(run_dir).glob("matrix_*.log")):
        stem = log_path.stem[len("matrix_"):]
        slug, _, oi = stem.rpartition("_")
        if slug not in ITEMS:
            continue
        cells.append((slug, int(oi), cell_measurements(log_path)))
    return cells


def score_run(run_dir, truth):
    """Tolerance of one census: per-cell (last line) and per-measurement."""
    cells = census_cells(run_dir)
    heads = set()
    per_cell_ok = per_meas_ok = per_meas_total = multi = 0
    misses, unmeasured, by_cell = [], [], {}
    for slug, oi, measurements in cells:
        if not measurements:
            unmeasured.append((slug, oi))
            continue
        multi += len(measurements) > 1
        for dims, _k, n_heads in measurements:
            heads.add(n_heads)
            per_meas_total += 1
            per_meas_ok += within_measurement_tolerance(dims, truth[slug])
        dims = measurements[-1][0]
        by_cell[(slug, oi)] = within_measurement_tolerance(dims, truth[slug])
        if by_cell[(slug, oi)]:
            per_cell_ok += 1
        else:
            side, vol = measurement_error(dims, truth[slug])
            misses.append((slug, oi, dims, side, vol))
    return {
        "dir": run_dir, "cells": len(cells), "measured": len(cells) - len(unmeasured),
        "per_cell_ok": per_cell_ok, "per_meas_ok": per_meas_ok,
        "per_meas_total": per_meas_total, "multi": multi, "heads": sorted(heads),
        "misses": misses, "unmeasured": unmeasured, "by_cell": by_cell,
    }


def main(argv=None):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    runs = list(argv if argv is not None else sys.argv[1:]) or list(DEFAULT_RUNS)
    truth = {slug: tuple(float(x) for x in analyze_file(STL_DIR / f"{stem}.stl")["dims"])
             for slug, (stem, _mass) in ITEMS.items()}

    print("правило: допуск организаторов (5 мм по стороне ИЛИ 10 % по объёму — "
          "более мягкий), истина из меша STL")
    print("на ячейку засчитывается ПОСЛЕДНЯЯ строка измерения; per-measurement "
          "считает все строки\n")
    scored = [score_run(run, truth) for run in runs]
    for r in scored:
        print(f"{r['dir']}: organizer tolerance {r['per_cell_ok']}/{r['measured']} "
              f"(per-measurement {r['per_meas_ok']}/{r['per_meas_total']}, "
              f"ячеек с 2+ измерениями {r['multi']}, heads={r['heads']})")
        for slug, oi, dims, side, vol in r["misses"]:
            t = "×".join(f"{d:.0f}" for d in truth[slug])
            d = "×".join(f"{x:.0f}" for x in dims)
            print(f"    OUT {slug}_{oi:<2} {d:>18}  истина {t:>18}  "
                  f"сторона {side:5.1f} мм  объём {vol * 100:5.1f} %")
        for slug, oi in r["unmeasured"]:
            print(f"    --- {slug}_{oi}: измерения в логе нет")
    if len(scored) > 1:
        print_common(scored)
    return 0


def print_common(scored):
    """Compare runs on the cells EVERY run measured — the only fair comparison.

    Raw ratios above have different denominators, because a different subset of
    cells loses its perception line in each run's retained log. Ranking runs by
    those ratios would rank the logging, not the rig.
    """
    common = set.intersection(*(set(r["by_cell"]) for r in scored))
    print(f"\nобщее подмножество ячеек, измеренных ВО ВСЕХ прогонах: {len(common)}")
    for r in scored:
        ok = sum(r["by_cell"][cell] for cell in common)
        print(f"    {ok}/{len(common)}  heads={r['heads']}  {r['dir']}")


if __name__ == "__main__":
    raise SystemExit(main())
