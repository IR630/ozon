#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline probe: 2 heads or 3? Routing and measurement across camera counts.

WHY THIS EXISTS. `probe_side_camera.py` answered "does a second view carry
information at all" (yes: tolerance 14/33 -> 30/33) and swept counts on 33 poses,
where routing saturated at 2 misroutes for 3+ heads. Two things were missing for a
build/don't-build decision on the THIRD head:

  1. SAMPLE SIZE. 33 poses is one seed. The cell has five seeded censuses on disk
     (runs/sweep_seed0..4), i.e. 165 settled poses — the same set the reported
     164/165 comes from. A one-cell difference between configs is noise at 33 and
     signal at 165.
  2. THE COST THAT SCALES WITH HEAD COUNT. Every probe so far modelled a PERFECT
     rig: clouds fused in exact world coordinates. That is the one assumption under
     which more heads can never hurt, so it cannot answer "2 or 3". A real rig
     registers each extra head to the first by extrinsic calibration, and that
     error is per-head and cumulative. Modelling it is what makes the comparison
     mean anything.

HONESTY LIMITS. Still no sensor noise, no belt texture, no dropout on dark or
specular packaging, and no frame-to-frame sync skew (the cell's two sensors are
untriggered at 15 Hz, worth up to 66.7 mm of belt travel at 1 m/s — that penalty
also scales with head count and is NOT modelled here, so the 3-head numbers below
are OPTIMISTIC). K is held at mesh truth for every config: a fixed side head does
not feed the production K path, which is yaw-invariant by construction. So this
isolates the DIMENSION effect of head count, which is the only thing extra heads
were ever claimed to buy.

    python scripts/probe_camera_count.py                    # all 5 seeds, 165 poses
    python scripts/probe_camera_count.py runs/sweep_seed0   # one census
"""
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent))

from analyze_models import analyze_file  # noqa: E402
from build_item_models import ITEMS, STL_DIR, set_belt_origin  # noqa: E402
from probe_side_camera import census_resting_quats, cloud_dims_mm, visible_points  # noqa: E402

from src.classification import classify, within_measurement_tolerance  # noqa: E402
from src.perception import BELT_TOP_Z_M, CAMERA_X_M, CAMERA_Y_M, CAMERA_Z_M  # noqa: E402

N_SURFACE_SAMPLES = 400_000

TOP = ((CAMERA_X_M, CAMERA_Y_M, CAMERA_Z_M), (CAMERA_X_M, CAMERA_Y_M, BELT_TOP_Z_M))
# Side heads clear of the diverter pivots at |y| = 0.28 and of the roll cages,
# at item height rather than gantry height so the flank fills the frame.
SIDE_NEG_Y = ((CAMERA_X_M, -0.90, BELT_TOP_Z_M + 0.35), (CAMERA_X_M, 0.0, BELT_TOP_Z_M + 0.10))
SIDE_POS_Y = ((CAMERA_X_M, +0.90, BELT_TOP_Z_M + 0.35), (CAMERA_X_M, 0.0, BELT_TOP_Z_M + 0.10))
# Down-belt head: sees the YZ face. Placed past the diverters, looking upstream.
SIDE_NEG_X = ((CAMERA_X_M + 1.4, 0.0, BELT_TOP_Z_M + 0.35), (CAMERA_X_M, 0.0, BELT_TOP_Z_M + 0.10))

CONFIGS = {
    "1: top": [TOP],
    "2: top+бок": [TOP, SIDE_NEG_Y],
    "3: top+2 встречных бока": [TOP, SIDE_NEG_Y, SIDE_POS_Y],
    "3: top+бок+вдоль ленты": [TOP, SIDE_NEG_Y, SIDE_NEG_X],
}

# (label, translation sigma mm, rotation sigma degrees) applied to EVERY head
# after the first. Typical target-based extrinsic calibration of a multi-camera
# rig lands near 1-3 mm / 0.1-0.3 deg; at ~1 m working distance 0.2 deg is
# already ~3.5 mm of lateral error, i.e. the organizers' whole 5 mm budget.
CALIBRATIONS = [
    ("идеальная (геом. предел)", 0.0, 0.0),
    ("хорошая 1 мм / 0.1°", 1.0, 0.1),
    ("типичная 2 мм / 0.2°", 2.0, 0.2),
    ("посредственная 3 мм / 0.3°", 3.0, 0.3),
]
CALIB_SEEDS = [0, 1, 2]


def sampled_points_mm(slug):
    """One deterministic surface cloud per item, in the belt-origin frame (mm).

    Sampled ONCE per item and rotated per pose instead of re-sampling: sampling is
    rotation-invariant, and reusing the identical point set across poses and
    configs removes sampling jitter from a comparison whose whole point is a
    one-cell difference.
    """
    import trimesh

    stem, _ = ITEMS[slug]
    mesh = trimesh.load(str(STL_DIR / f"{stem}.stl"), force="mesh")
    set_belt_origin(mesh)
    return trimesh.sample.sample_surface(mesh, N_SURFACE_SAMPLES, seed=0)[0]


def place(pts_mm, quat):
    """World-frame metres: rotate the canonical cloud into `quat`, rest it on the belt."""
    import trimesh

    x, y, z, w = quat
    rot = trimesh.transformations.quaternion_matrix([w, x, y, z])[:3, :3]
    pts = (pts_mm @ rot.T) / 1000.0
    pts[:, 0] += CAMERA_X_M - (pts[:, 0].min() + pts[:, 0].max()) / 2
    pts[:, 1] += CAMERA_Y_M - (pts[:, 1].min() + pts[:, 1].max()) / 2
    pts[:, 2] += BELT_TOP_Z_M - pts[:, 2].min()
    return pts


def misregister(pts_m, sigma_mm, sigma_deg, rng):
    """Apply one head's extrinsic calibration error: small rigid transform."""
    if sigma_mm <= 0.0 and sigma_deg <= 0.0:
        return pts_m
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)
    angle = np.radians(rng.normal(0.0, sigma_deg))
    c, s = np.cos(angle), np.sin(angle)
    cross = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    rot = c * np.eye(3) + s * cross + (1 - c) * np.outer(axis, axis)
    centre = pts_m.mean(axis=0)
    shift = rng.normal(0.0, sigma_mm / 1000.0, size=3)
    return (pts_m - centre) @ rot.T + centre + shift


def truth_of(slug):
    """(dims mm desc, true K, true category) from the mesh — pose-independent."""
    stem, _ = ITEMS[slug]
    ref = analyze_file(STL_DIR / f"{stem}.stl")
    dims = tuple(float(x) for x in ref["dims"])
    return dims, float(ref["k"]), ref["category"]


def load_poses(dirs):
    """[(slug, pose_key, quat)] over every census directory given."""
    poses = []
    for d in dirs:
        by_slug = census_resting_quats(d)
        for slug, entries in by_slug.items():
            for oi, quat in entries:
                poses.append((slug, f"{Path(d).name}:{oi}", quat))
    return poses


def main(argv=None):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    args = list(argv if argv is not None else sys.argv[1:])
    root = Path(__file__).resolve().parents[1]
    dirs = args or sorted(str(p) for p in (root / "runs").glob("sweep_seed*"))
    poses = load_poses(dirs)
    if not poses:
        sys.exit(f"не найдено поз покоя (`resting rpy:`) в {dirs}")
    print(f"поз покоя: {len(poses)} из {len(dirs)} переписей")

    truth = {slug: truth_of(slug) for slug in {p[0] for p in poses}}
    clouds = {slug: sampled_points_mm(slug) for slug in {p[0] for p in poses}}

    # visible point set per (pose, head) — computed once, reused by every config
    seen = {}
    for i, (slug, key, quat) in enumerate(poses):
        pts = place(clouds[slug], quat)
        for cfg in CONFIGS.values():
            for cam in cfg:
                if (i, cam) in seen:
                    continue
                seen[(i, cam)] = visible_points(pts, cam[0], cam[1])
        if (i + 1) % 20 == 0:
            print(f"  ... видимость {i + 1}/{len(poses)}", flush=True)

    results = {}
    for cfg_name, cams in CONFIGS.items():
        for calib_name, s_mm, s_deg in CALIBRATIONS:
            seeds = [0] if s_mm == 0.0 and s_deg == 0.0 else CALIB_SEEDS
            mis, tol, total, culprits = 0, 0, 0, {}
            for seed in seeds:
                rng = np.random.default_rng(seed)
                for i, (slug, key, _quat) in enumerate(poses):
                    t_dims, t_k, t_cat = truth[slug]
                    parts = []
                    for head, cam in enumerate(cams):
                        pts = seen[(i, cam)]
                        if not len(pts):
                            continue
                        parts.append(pts if head == 0 else misregister(pts, s_mm, s_deg, rng))
                    if not parts:
                        continue
                    dims = cloud_dims_mm(np.vstack(parts))
                    if dims is None:
                        continue
                    total += 1
                    tol += within_measurement_tolerance(dims, t_dims)
                    if classify(dims, t_k) != t_cat:
                        mis += 1
                        culprits[slug] = culprits.get(slug, 0) + 1
            results[(cfg_name, calib_name)] = (mis, tol, total, culprits)
            print(f"готово: {cfg_name:26} | {calib_name}", flush=True)

    print("\n\n### Мисроуты от истинной зоны (меньше — лучше), на прогон из "
          f"{len(poses)} поз")
    print(f"\n{'калибровка':30}" + "".join(f"{c:>26}" for c in CONFIGS))
    for calib_name, *_ in CALIBRATIONS:
        row = f"{calib_name:30}"
        for cfg_name in CONFIGS:
            mis, _tol, total, _c = results[(cfg_name, calib_name)]
            n_runs = 1 if calib_name.startswith("идеальная") else len(CALIB_SEEDS)
            row += f"{mis / n_runs:>21.1f}/{len(poses)}"
        print(row)

    print("\n\n### В допуске организаторов (больше — лучше), доля прогонов")
    print(f"\n{'калибровка':30}" + "".join(f"{c:>26}" for c in CONFIGS))
    for calib_name, *_ in CALIBRATIONS:
        row = f"{calib_name:30}"
        for cfg_name in CONFIGS:
            _mis, tol, total, _c = results[(cfg_name, calib_name)]
            row += f"{tol / max(total, 1) * 100:>25.0f}%"
        print(row)

    print("\n\n### Кто именно мисроутит (режим «типичная 2 мм / 0.2°»)")
    for cfg_name in CONFIGS:
        _mis, _tol, _total, culprits = results[(cfg_name, "типичная 2 мм / 0.2°")]
        items = ", ".join(f"{k} ×{v}" for k, v in sorted(culprits.items(),
                                                         key=lambda kv: -kv[1])) or "—"
        print(f"{cfg_name:26} {items}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
