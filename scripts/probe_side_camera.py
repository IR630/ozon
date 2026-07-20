#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline probe: what would a SECOND (side) depth camera buy us?

WHY THIS EXISTS. The expert tied projections to sensing: cameras may be any number
and need not point straight down, "design the sensing to cover every face of the
item" (docs/md/expert_session_qa.md [40:19], [70:58], [71:27]). The council that
ruled "do not add cameras" never saw that line (docs/defense/council_cameras.md).
Probe A then measured that one of three tolerance misses -- the helmet -- IS a
viewing problem: top-down under-reads the dome (356 -> 337 mm) while a side view
sees 348-353 mm (docs/decisions.md 2026-07-20).

Probe A measured RAW VISIBILITY per viewpoint. This probe answers the decision
question instead: FUSE a top-down and a side cloud, measure the fused body, and
compare against both the mesh truth and today's top-down-only number. If the
helmet comes into tolerance and the other ten do not regress, a side camera buys
accuracy; if not, the "one camera" limitation is settled with a number.

HOW IT STAYS HONEST. The oriented box is not reimplemented: the frozen production
routine src.perception._body_obb_dims_mm builds its cloud as
((xs-cx)*depth/fx, (ys-cy)*depth/fy, heights), so passing fx=fy=1, depth=1,
cx=cy=0 feeds it an arbitrary world cloud and runs the SAME flush-face search the
cell runs. Occlusion is honest: each view keeps the nearest surface sample per
pixel (a z-buffer), so a side camera sees only the near flank, exactly as a real
one would.

HONESTY LIMITS. Rendered surfaces carry no sensor noise, no belt texture and no
registration error between the two cameras -- a real two-camera rig must be
extrinsically calibrated, and that error is NOT modelled here. So this probe
measures the GEOMETRIC upper bound of a second view, not the achievable field
accuracy. Its numbers may only be read as "does the information exist at all".

    python scripts/probe_side_camera.py
"""
import re
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent))

from build_item_models import ITEMS, set_belt_origin  # noqa: E402
from render_depth import load_mesh  # noqa: E402

from src.classification import measurement_error, within_measurement_tolerance  # noqa: E402
from src.perception import (  # noqa: E402
    BELT_TOP_Z_M,
    CAMERA_X_M,
    CAMERA_Y_M,
    CAMERA_Z_M,
    FX,
    FY,
    IMG_H,
    IMG_W,
    _body_obb_dims_mm,
)

N_SURFACE_SAMPLES = 400_000
N_POSES = 4  # identity + 3 seeded tumbles, per item

# Side camera: horizontal look across the belt from -Y, clear of the diverter
# pivots at |y| = 0.28 and of the roll cages. Height near the item, not the
# gantry, so the flank fills the frame.
SIDE_CAM_POS_M = (CAMERA_X_M, -0.90, BELT_TOP_Z_M + 0.35)
SIDE_CAM_TARGET_M = (CAMERA_X_M, 0.0, BELT_TOP_Z_M + 0.10)

TOP_CAM_POS_M = (CAMERA_X_M, CAMERA_Y_M, CAMERA_Z_M)
TOP_CAM_TARGET_M = (CAMERA_X_M, CAMERA_Y_M, BELT_TOP_Z_M)


def place_on_belt(mesh, quat):
    """World-frame surface samples (m) of `mesh` in pose `quat`, resting on the belt.

    Same placement contract as scripts/render_depth.py: model origin convention,
    rotate, then drop so the lowest point touches the belt and centre under the
    top camera.
    """
    import trimesh

    mesh = mesh.copy()
    set_belt_origin(mesh)
    x, y, z, w = quat
    mesh.apply_transform(trimesh.transformations.quaternion_matrix([w, x, y, z]))
    pts = trimesh.sample.sample_surface(mesh, N_SURFACE_SAMPLES, seed=0)[0] / 1000.0
    pts[:, 0] += CAMERA_X_M - (pts[:, 0].min() + pts[:, 0].max()) / 2
    pts[:, 1] += CAMERA_Y_M - (pts[:, 1].min() + pts[:, 1].max()) / 2
    pts[:, 2] += BELT_TOP_Z_M - pts[:, 2].min()
    return pts


def visible_points(pts_m, cam_pos_m, cam_target_m):
    """Subset of `pts_m` a pinhole depth camera at that pose would actually see.

    One nearest sample per pixel: the z-buffer IS the occlusion model, so a side
    camera returns the near flank only and never the far one.
    """
    cam = np.asarray(cam_pos_m, dtype=float)
    forward = np.asarray(cam_target_m, dtype=float) - cam
    forward /= np.linalg.norm(forward)
    world_up = np.array([0.0, 0.0, 1.0])
    if abs(forward @ world_up) > 0.99:  # looking straight down: pick another up
        world_up = np.array([1.0, 0.0, 0.0])
    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)

    rel = pts_m - cam
    z = rel @ forward
    in_front = z > 1e-6
    if not in_front.any():
        return pts_m[:0]
    u = np.rint(IMG_W / 2.0 + (rel @ right) * FX / np.where(in_front, z, 1.0)).astype(int)
    v = np.rint(IMG_H / 2.0 - (rel @ up) * FY / np.where(in_front, z, 1.0)).astype(int)
    inside = in_front & (u >= 0) & (u < IMG_W) & (v >= 0) & (v < IMG_H)
    if not inside.any():
        return pts_m[:0]

    idx = np.flatnonzero(inside)
    pixel = v[idx] * IMG_W + u[idx]
    # nearest sample per pixel, vectorized: sort by (pixel, depth), keep first
    order = np.lexsort((z[idx], pixel))
    idx, pixel = idx[order], pixel[order]
    first = np.ones(len(pixel), dtype=bool)
    first[1:] = pixel[1:] != pixel[:-1]
    return pts_m[idx[first]]


def cloud_dims_mm(pts_m):
    """Dims (mm, desc) of the smallest box around a world cloud, via PROD code.

    fx=fy=1, depth=1, cx=cy=0 make _body_obb_dims_mm's internal backprojection the
    identity, so it searches the same hull-facet orientations on our points. A huge
    `legacy_dims_mm` keeps the belt-aligned fallback from ever winning; None (the
    near-flat relief gate) falls back to the axis-aligned extent.
    """
    if len(pts_m) < 4:
        return None
    heights_m = pts_m[:, 2] - BELT_TOP_Z_M
    dz_mm = float(heights_m.max() * 1000.0)
    dims = _body_obb_dims_mm(
        xs=pts_m[:, 0], ys=pts_m[:, 1], depth_col_m=np.ones(len(pts_m)),
        heights_m=heights_m, fx=1.0, fy=1.0, cx=0.0, cy=0.0,
        legacy_dims_mm=(1e4, 1e4, 1e4), dz_mm=dz_mm, px_pad_mm=0.0)
    if dims is None:
        extent = (pts_m.max(axis=0) - pts_m.min(axis=0)) * 1000.0
        extent[2] = max(extent[2], dz_mm)
        return sorted(float(e) for e in extent)[::-1]
    return dims


def truth_dims_mm(mesh):
    """True dims (mm, desc): extents of the mesh's own oriented bounding box."""
    return sorted(float(e) for e in mesh.bounding_box_oriented.primitive.extents)[::-1]


def census_resting_quats(logdir):
    """{slug: [(orient_index, quat), ...]} of the poses the CELL actually settled into.

    The seeded tumbles below are deliberately harsh and include physically
    impossible rests (a 400 mm box balanced on a corner), so their pass rate is
    NOT comparable with the census. Every census cell log records the orientation
    the item came to rest in as `resting rpy:`; reading those makes the probe
    measure the same poses the routing matrix scored.
    """
    from body_pose import quat_from_rpy

    out = {}
    for log in sorted(Path(logdir).glob("matrix_*.log")):
        stem = log.stem[len("matrix_"):]
        slug, _, oi = stem.rpartition("_")
        if slug not in ITEMS:
            continue
        m = re.search(r"resting rpy: r=(\S+) p=(\S+) y=(\S+)",
                      log.read_text(encoding="utf-8", errors="replace"))
        if m:
            rpy = [float(v) for v in m.groups()]
            out.setdefault(slug, []).append((int(oi), quat_from_rpy(*rpy)))
    return out


def seeded_quats(n, seed=0):
    """Identity plus n-1 reproducible tumbles (Karpathy #5: seed, never wall clock)."""
    rng = np.random.default_rng(seed)
    quats = [(0.0, 0.0, 0.0, 1.0)]
    while len(quats) < n:
        q = rng.normal(size=4)
        quats.append(tuple(q / np.linalg.norm(q)))
    return quats


def main():
    census_dir = None
    if len(sys.argv) == 3 and sys.argv[1] == "--from-census":
        census_dir = sys.argv[2]
    elif len(sys.argv) != 1:
        sys.exit("usage: probe_side_camera.py [--from-census <census logdir>]")

    poses_by_slug = census_resting_quats(census_dir) if census_dir else None
    if poses_by_slug is not None and not poses_by_slug:
        sys.exit(f"no `resting rpy:` found in {census_dir} — wrong log directory?")
    print("позы:", "покоя из переписи" if census_dir else "сеяные кувырки")

    rows = []
    for slug in ITEMS:
        mesh = load_mesh(slug)
        truth = truth_dims_mm(mesh)
        poses = (poses_by_slug.get(slug, []) if poses_by_slug is not None
                 else list(enumerate(seeded_quats(N_POSES))))
        for pose_i, quat in poses:
            pts = place_on_belt(mesh, quat)
            top = visible_points(pts, TOP_CAM_POS_M, TOP_CAM_TARGET_M)
            side = visible_points(pts, SIDE_CAM_POS_M, SIDE_CAM_TARGET_M)
            d_top = cloud_dims_mm(top)
            d_fused = cloud_dims_mm(np.vstack([top, side]) if len(side) else top)
            if d_top is None or d_fused is None:
                print(f"{slug:>16} pose{pose_i}  EMPTY VIEW")
                continue
            rows.append((slug, pose_i, truth, d_top, d_fused, len(side)))
            e_top = measurement_error(d_top, truth)
            e_fused = measurement_error(d_fused, truth)
            print(
                f"{slug:>16} pose{pose_i} "
                f"| truth {truth[0]:6.1f}x{truth[1]:6.1f}x{truth[2]:6.1f} "
                f"| top {d_top[0]:6.1f}x{d_top[1]:6.1f}x{d_top[2]:6.1f} "
                f"err {e_top[0]:5.1f}mm {'IN ' if within_measurement_tolerance(d_top, truth) else 'OUT'} "
                f"| fused {d_fused[0]:6.1f}x{d_fused[1]:6.1f}x{d_fused[2]:6.1f} "
                f"err {e_fused[0]:5.1f}mm "
                f"{'IN ' if within_measurement_tolerance(d_fused, truth) else 'OUT'}"
            )

    n_top = sum(within_measurement_tolerance(r[3], r[2]) for r in rows)
    n_fused = sum(within_measurement_tolerance(r[4], r[2]) for r in rows)
    print(f"\norganizer tolerance: top-down {n_top}/{len(rows)}, fused {n_fused}/{len(rows)}")
    worse = [(r[0], r[1]) for r in rows
             if within_measurement_tolerance(r[3], r[2])
             and not within_measurement_tolerance(r[4], r[2])]
    if worse:
        print(f"REGRESSED by fusing: {worse}")


if __name__ == "__main__":
    main()
