#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Do the fusion-rule rankings survive on LIVE Gazebo frames — the gate before the port.

WHY THIS EXISTS. `scripts/probe_camera_count.py` measured five fusion rules over
165 poses and ranked them clearly: `structural-gated` takes zero misroutes at
every head count and every calibration and carries the smallest |error|, while
production still uses the component-wise maximum (`src/multiview.py:141`). That
makes porting the rule the single largest measured improvement available — larger
than any camera. It also makes it a kernel change, and this project has already
paid once for trusting an offline ranking: `cv/section-k-geometric` scored 39/48
against 32/48 on rendered meshes "with zero regressions" and then produced 30/33
against 33/33 in Gazebo (`docs/defense/council_cameras.md`, postscript).

The difference between the two runs is the frames. probe_camera_count samples
400k points off the MESH and hides every head behind a z-buffer; a Gazebo depth
frame carries quantisation, a real projection, self-occlusion and the belt. So
this probe re-ranks the same five rules on `runs/frames/*_3cam` before anyone
edits the kernel. Same rule implementations, imported not copied — a re-ranking
that used a second implementation would prove nothing about the port.

THE SIDE CLOUDS ARE THE INDEPENDENT-DETECTION ONES, and that is deliberate. The
shipped path crops side points to the box the top head already claimed, so the
side cloud cannot disagree with the top head by construction (`cameras.md` §8 T3).
The rig this ranking is for is 3A WITH independent detection, so the side clouds
come from `probe_side_fallback.find_side_items` — each head segmenting its own
frame. That is also the only configuration in which the fusion rule can matter.

WHAT THIS IS NOT. Three items of eleven, one resting pose each, one frame each,
fusion at dt = 0. It cannot rank rules by routing — that needs the contour. It
answers exactly one question: does the offline ORDER of the rules hold up on real
frames, or does it invert the way section-k did.

    python3 scripts/probe_fusion_live.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from analyze_models import analyze_file  # noqa: E402
from build_item_models import ITEMS, STL_DIR  # noqa: E402
from probe_camera_count import FUSIONS, fuse_dims  # noqa: E402
from probe_depth_dropout import slug_of_dump  # noqa: E402
from probe_noise_heads import RIGS, load_rig_frames  # noqa: E402
from probe_side_fallback import find_side_items, side_world_points  # noqa: E402

from src.classification import measurement_error, within_measurement_tolerance  # noqa: E402
from src.constants import CAMERA_TOP_POSE_M  # noqa: E402
from src.perception import (  # noqa: E402
    BELT_DEPTH_M,
    FX,
    FY,
    MASK_MARGIN_M,
    _item_mask,
)

# Head counts compared, by the names the report uses. "2" is the shipping rig plus
# one flank; "3A" is the two opposing flanks the 26.07 decision selected.
CONFIGS = (("2: top+бок", 1), ("3A: top+2 встречных бока", 2))


def top_item_points(depth_m):
    """World points of the goods the TOP head sees, by the production mask.

    Not the prism `find_side_items` uses: the top path is the shipped one and it
    segments against `MASK_MARGIN_M` = 5 mm. Borrowing the side heads' 8 mm floor
    here would drop the pen, which stands 9 mm over the belt and is the whole
    reason the margins differ in the first place.
    """
    pts, vs, us = side_world_points(depth_m, CAMERA_TOP_POSE_M, FX, FY)
    if not len(pts):
        return None
    mask = _item_mask(np.asarray(depth_m, dtype=float), BELT_DEPTH_M, MASK_MARGIN_M)
    keep = mask[vs, us]
    return pts[keep] if keep.any() else None


def main(argv=None):
    dirs = sorted(d for d in (ROOT / "runs" / "frames").glob("*_3cam") if d.is_dir())
    if not dirs:
        print("no rig dumps — run runs/g_dump_3cam.sh first (needs Gazebo)")
        return 1

    truth = {slug: tuple(float(x) for x in analyze_file(STL_DIR / f"{stem}.stl")["dims"])
             for slug, (stem, _mass) in ITEMS.items()}

    print("Fusion rules re-ranked on LIVE Gazebo frames (runs/frames/*_3cam).")
    print("Side clouds come from INDEPENDENT per-head detection, not from the shipped")
    print("crop — the shipped crop makes the fusion rule almost unobservable.")
    print("truth = mesh OBB; tolerance = organizers' rule (5 mm/side OR 10 % volume)")
    print("dt = 0: no sync penalty, so every rig above one head is optimistic\n")

    header = f"{'правило слияния':<18}"
    for name, _n in CONFIGS:
        header += f"{name:>28}"
    print(header)

    scores = {}
    for fusion in FUSIONS:
        row = f"{fusion:<18}"
        for cfg_name, n_side in CONFIGS:
            errs, in_tol, total = [], 0, 0
            for dump_dir in dirs:
                try:
                    top, sides = load_rig_frames(dump_dir)
                except FileNotFoundError:
                    continue
                heads = [(name, pose) for name, pose in RIGS[-1][1] if name in sides]
                if len(heads) < n_side:
                    continue
                top_pts = top_item_points(top)
                if top_pts is None or len(top_pts) < 4:
                    continue
                parts = [top_pts]
                for name, pose in heads[:n_side]:
                    found = find_side_items(sides[name], pose)
                    if found:
                        parts.append(found[0].points_m)
                dims = fuse_dims(parts, fusion)
                if dims is None:
                    continue
                truth_dims = truth[slug_of_dump(dump_dir.name)]
                total += 1
                errs.append(measurement_error(dims, truth_dims)[0])
                in_tol += within_measurement_tolerance(dims, truth_dims)
            if not total:
                row += f"{'—':>28}"
                continue
            scores[(fusion, cfg_name)] = (float(np.median(errs)), in_tol, total)
            row += f"{np.median(errs):>18.1f} мм {in_tol:>3d}/{total}"
        print(row)

    print("\nЛучшее правило по медиане ошибки на каждой конфигурации:")
    for cfg_name, _n in CONFIGS:
        ranked = sorted(((v[0], k[0]) for k, v in scores.items() if k[1] == cfg_name))
        if ranked:
            best = ", ".join(f"{name} {err:.1f} мм" for err, name in ranked[:3])
            print(f"  {cfg_name:<28} {best}")
    print("\nРАМКА: 3 товара из 11, по одной позе покоя, один кадр, dt = 0.")
    print("Ранжирует ПРАВИЛА на живых кадрах и НЕ ранжирует маршрутизацию —")
    print("для неё нужен контур, и урок section-k именно об этом.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
