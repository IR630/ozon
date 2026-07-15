#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Where does the rule break across stable supports beyond seed 0's three poses?

The census tests 33 cells. Every residual risk on the milestone list is a claim about a
pose that did NOT come up: "Тарелка on edge", "Шлем 317 mm from a 320 limit", "Мешок's K
sits on the 0.8 threshold". Those are guesses until someone measures them, and a Gazebo
cell costs ~30 s. Rendered off the mesh a pose costs milliseconds (scripts/render_depth.py,
whose domain gap against real Gazebo frames is measured at 3 mm / 0.01 K), so this sweep
screens the question directly: per item, over a bounded area-weighted sample of stable
supports and random yaw, how often is the category right, and how close are misses to a
threshold?

Every pose here is one the item can physically REST in — the mesh is dropped onto the belt
in that rotation, exactly as the simulator drops it. What the sweep cannot model is the
settling: Gazebo would tip an unstable pose over before the camera saw it (a plate spawned
on edge lies flat), so a failure here is a WORST CASE — "if the item presents this way" —
not a prediction of census frequency. That is the honest reading, and it is the useful one:
a rule that survives every presentable pose needs no luck.

MEASURED LIMIT of that reading (2026-07-14): a statically stable hull facet is not always
a pose Gazebo SETTLES into. The bottle's exactly-horizontal rest — 69% of its area weight,
and this sweep's headline "break" (K=0.00 -> B) — was never realized in three independent
Gazebo settles: the hull's neck cone props the bottle ~8.5 deg tilted, where the vertical-
section K reads 1.00 -> D (real frame frozen as tests/fixtures/frames/bottle_side; census
oi=1/2 measured the same). A sweep miss is a lead, not a verdict: cross-check it against a
real settle (scripts/dump_item_frame.sh) before calling it a hole.

Usage:
    python3 scripts/pose_sweep.py [n_poses] [seed]
"""
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent))

from build_item_models import ITEMS  # noqa: E402
from render_depth import load_mesh, render_depth  # noqa: E402

from src.classification import classify_conservative  # noqa: E402
from src.constants import MAX_DIMS_MM, ROUND_K_THRESHOLD  # noqa: E402
from src.perception import measure_item  # noqa: E402

# The reference category of every item, from the task statement (same table run_matrix.sh
# routes by — not re-derived here, or the sweep would grade itself against its own guess).
REFERENCE = {
    "bottle": "D", "box_300x200x200": "B", "box_400x400x300": "C", "lunchbox": "B",
    "bag": "B", "detergent": "B", "pouf": "C", "pen": "C", "plate": "D",
    "cylinder": "B", "helmet": "B",
}

# A fine collision hull may have hundreds of individually stable triangles. Rendering all
# of them would make the default 8-yaw sweep unbounded by mesh tessellation. Systematic
# weighted resampling approximates the normalized support-area distribution while capping
# one item at 16 * 8 = 128 rendered frames. The sampled weights still sum to one; `seed`
# remains solely responsible for yaw sampling.
MAX_REST_SAMPLES = 16


def stable_rests(mesh):
    """The orientations the item can actually COME TO REST in, with a rough weight each.

    Sweeping uniformly random rotations is the obvious thing and it is wrong — it grades the
    rule on orientations the item cannot hold. A box balanced on a corner projects a 380 mm
    silhouette and duly "fails" as C; the simulator tips it onto a face long before the
    camera sees it. Swept that way this script reported box_300 at 16/40 and Бутылка at
    6/40 — numbers that describe nothing physical and would have sent the team hunting
    phantom CV bugs.

    The classical criterion, on the convex hull the simulator actually collides with: a
    face is a stable rest iff the centre of mass projects INSIDE that face. (trimesh has
    compute_stable_poses, but it pulls in networkx, which is not on the organizers'
    allowed-software list — docs/md/software.md — and the solution has to deploy in their
    environment.) The weight is the face's area share: a coarse stand-in for how often a
    tumbling item lands on it, honest enough to rank risks, not a probability.
    """
    import trimesh

    hull = mesh.convex_hull
    com = np.asarray(hull.center_mass)
    support_faces = [np.asarray(facet, dtype=int) for facet in hull.facets]
    support_normals = [np.asarray(normal) for normal in hull.facets_normal]
    grouped = np.zeros(len(hull.faces), dtype=bool)
    for facet in support_faces:
        grouped[facet] = True
    for face_id in np.flatnonzero(~grouped):
        support_faces.append(np.array([face_id], dtype=int))
        support_normals.append(hull.face_normals[face_id])

    rests = []
    for facet, normal in zip(support_faces, support_normals):
        poly = hull.vertices[np.unique(hull.faces[facet].ravel())]
        if len(poly) < 3:
            continue
        # basis of the facet plane, to test support in 2D
        u = poly[1] - poly[0]
        if not np.linalg.norm(u):
            continue
        u = u / np.linalg.norm(u)
        v = np.cross(normal, u)
        flat = np.column_stack([(poly - poly[0]) @ u, (poly - poly[0]) @ v])
        com_flat = np.array([(com - poly[0]) @ u, (com - poly[0]) @ v])
        if not _point_in_convex_polygon(com_flat, flat):
            continue  # the item topples off this face instead of resting on it
        # turn this face into the bottom: its outward normal points down
        transform = trimesh.geometry.align_vectors(normal, [0, 0, -1])
        rests.append((transform, float(hull.area_faces[facet].sum())))
    total = sum(w for _, w in rests) or 1.0
    return [(t, w / total) for t, w in rests]


def _point_in_convex_polygon(pt, poly_pts):
    """Is `pt` inside the convex hull of `poly_pts` (both 2D)? Pure numpy."""
    from scipy.spatial import ConvexHull, Delaunay

    if len(poly_pts) < 3:
        return False
    try:
        hull = ConvexHull(poly_pts)
    except Exception:  # noqa: BLE001  (degenerate facet)
        return False
    return Delaunay(poly_pts[hull.vertices]).find_simplex(pt) >= 0


def presentable_poses(mesh, yaws, seed):
    """A bounded area-weighted sample of stable rests, each sampled over yaw.

    Yaw about the vertical does not change stability but does change the pixels, so each
    selected rest is measured at several yaws. Fine meshes are reduced by deterministic
    systematic resampling rather than an individual-weight cutoff: sampled weights sum to
    one, CDF strata approximate the whole support-area distribution without deleting every
    individually small rest, and tessellation cannot explode render count.
    """
    import trimesh

    rng = np.random.RandomState(seed)
    out = []
    rests = stable_rests(mesh)
    if len(rests) > MAX_REST_SAMPLES:
        cumulative = np.cumsum([weight for _, weight in rests])
        cumulative[-1] = 1.0  # protect searchsorted from floating-point normalization drift
        targets = (np.arange(MAX_REST_SAMPLES) + 0.5) / MAX_REST_SAMPLES
        indices = np.searchsorted(cumulative, targets)
        rests = [(rests[int(index)][0], 1.0 / MAX_REST_SAMPLES) for index in indices]
    for transform, weight in rests:
        for _ in range(yaws):
            yaw = trimesh.transformations.rotation_matrix(rng.uniform(0, 2 * np.pi), [0, 0, 1])
            w, x, y, z = trimesh.transformations.quaternion_from_matrix(yaw @ transform)
            out.append(((float(x), float(y), float(z), float(w)), weight / yaws))
    return out


def sweep_item(slug, yaws, seed):
    """(weighted correctness, misses, margins) of the rule over an item's presentable poses."""
    mesh = load_mesh(slug)
    ref = REFERENCE[slug]
    poses = presentable_poses(mesh.copy(), yaws, seed)
    total_w = sum(w for _, w in poses)
    good_w, misses, margins = 0.0, [], []
    for quat, weight in poses:
        got = measure_item(render_depth(mesh, quat))
        if got is None:
            misses.append(("no_detect", None, None, weight))
            continue
        cat = classify_conservative(got.dims_mm, got.k)
        dims = sorted(got.dims_mm, reverse=True)
        # how close this pose came to flipping: distance to the size limit and to K's line
        margins.append((min(MAX_DIMS_MM[i] - dims[i] for i in range(3)),
                        got.k - ROUND_K_THRESHOLD))
        if cat == ref:
            good_w += weight
        else:
            misses.append((cat, dims, got.k, weight))
    return good_w / total_w if total_w else 0.0, misses, margins, len(poses)


def main():
    yaws = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 0

    print(
        f"=== pose sweep: up to {MAX_REST_SAMPLES} area-weighted STABLE rests "
        f"x {yaws} yaws, seed={seed} ==="
    )
    print(f"{'item':18} {'ref':>3} {'correct':>9} {'poses':>6}  "
          f"{'tightest size margin':>21} {'tightest K margin':>18}")
    print("-" * 84)
    worst = defaultdict(list)
    for slug in ITEMS:
        share, misses, margins, n_poses = sweep_item(slug, yaws, seed)
        sm = min((m[0] for m in margins), default=float("nan"))
        km = min((abs(m[1]) for m in margins), default=float("nan"))
        flag = "" if share > 0.999 else "   <-- BREAKS"
        print(f"{slug:18} {REFERENCE[slug]:>3} {100 * share:>8.1f}% {n_poses:>6}  "
              f"{sm:>18.0f} mm {km:>18.2f}{flag}")
        worst[slug] = misses

    print()
    for slug, ms in worst.items():
        if not ms:
            continue
        by_cat = defaultdict(float)
        for cat, _, _, weight in ms:
            by_cat[cat] += weight
        print(f"{slug} -> ref {REFERENCE[slug]}: " +
              ", ".join(f"{c} in {100 * w:.0f}% of drops" for c, w in sorted(by_cat.items())))
        for cat, dims, k, _ in ms[:2]:
            if dims is not None:
                print(f"    got {cat}: dims={[round(v) for v in dims]} K={k:.2f}")


if __name__ == "__main__":
    main()
