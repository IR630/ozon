#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Deterministic spawn orientations for the day-4 matrix (P1+P2).

The day-3 skeleton spawned every item in its default STL pose — no randomness,
so nothing to seed. Day 4 exercises perception under varied poses: this is the
FIRST source of randomness in the contour, so it carries the run's seed (the
milestone's `seed` knob, docs/decisions.md).

A cell of the 11-items x N-orientations matrix is identified by
(seed, item_index, orient_index). Its orientation is reproducible from those
three numbers alone, independent of iteration order, so a single failing cell
can be replayed in isolation. orient_index 0 is the identity (default STL pose),
so the matrix is a superset of the day-3 single-item runs.

Usage (called per cell from scripts/run_matrix.sh):
    python3 scripts/spawn_orientations.py <seed> <item_index> <orient_index> [slug]
    -> prints "x y z w" — plus, given the slug, the spawn HEIGHT and the sideways
       OFFSET that place that pose onto the middle of the belt.

Neither is a constant, and both exist for the same reason: the generated model's
origin is its default-pose BOTTOM (build_item_models.set_belt_origin: Z=0 at the
lowest point) with x/y left wherever the source STL had them, and Gazebo rotates
the model about that origin. So a fixed z buries turned/tall items inside the belt
and the solver ejects them at the spawn (the seed-0 census feed_jams), and a fixed
y=0 centres an arbitrary internal origin rather than the goods — a rotated helmet
lands 154 mm off-centre, inside the infeed rail. spawn_pose_for_mesh_m measures the
rotated body and places it: lowest point one clearance above the belt, body centred
across it.
"""
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))          # build_item_models (sibling script)
sys.path.insert(0, str(_HERE.parent))   # src/ (running as a script, cwd is not on the path)
from build_item_models import ITEMS, STL_DIR, set_belt_origin  # noqa: E402  (slug -> STL + origin, single source)

from src.perception import BELT_TOP_Z_M  # noqa: E402  (belt top, single source)

# Real air gap under the item's lowest point at spawn: enough to clear solver
# jitter, small enough that the item lies onto the belt rather than dropping. It
# is a TRUE gap in every pose because spawn_height_m measures the lowest point
# per orientation (see there), not a fudge factor over a mis-centred origin.
SPAWN_CLEARANCE_M = 0.005


def orientation_quat(seed, item_index, orient_index):
    """Reproducible unit quaternion (x, y, z, w) for one matrix cell.

    orient_index 0 -> identity (default STL pose). Otherwise a uniform-random
    rotation: a 4D standard-normal normalized to the unit sphere S^3 is uniform
    over unit quaternions, hence uniform over SO(3).
    """
    if orient_index == 0:
        return (0.0, 0.0, 0.0, 1.0)
    # Combine the three indices into one 32-bit seed: order-independent, and
    # distinct cells get distinct streams.
    cell_seed = (int(seed) * 1_000_003 + int(item_index) * 1009 + int(orient_index)) % (2**32)
    rng = np.random.RandomState(cell_seed)
    q = rng.standard_normal(4)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    if w < 0:  # canonical hemisphere (q and -q are the same rotation)
        x, y, z, w = -x, -y, -z, -w
    return (float(x), float(y), float(z), float(w))


def spawn_pose_for_mesh_m(mesh, quat):
    """(height, y offset) that lay a source mesh onto the MIDDLE of the belt.

    Two corrections with one root: set_belt_origin centres the model on its
    DEFAULT-pose bounding box, and Gazebo rotates about that origin — so for any
    shape that is not symmetric, the rotated body's box is no longer centred on it.

      height  — drops the origin so the item's lowest point in THIS pose rests one
                clearance above the belt (it would otherwise sink into the belt or
                be dropped onto it; docs/decisions.md 2026-07-12).
      y offset — slides the origin so the ROTATED body is centred across the belt.
                Without it we centre the item's default-pose origin rather than the
                goods actually on the belt: the helmet in orient 2 lands with its
                body 154 mm to one side, reaching y=-350 mm against a 252 mm rail
                gap — it spawns INSIDE the infeed rail, the solver extrudes it
                upward for ~40 s, and the belt never carries it. That is the
                helmet's stable census failure. A real infeed puts the item on the
                middle of the belt, not one of its corners.
    """
    import trimesh  # heavy; only the matrix needs it

    set_belt_origin(mesh)
    x, y, z, w = quat
    mesh.apply_transform(trimesh.transformations.quaternion_matrix([w, x, y, z]))
    lowest_m = mesh.bounds[0][2] / 1000.0     # STL is in mm; rel. to the model origin
    body_mid_y_m = (mesh.bounds[0][1] + mesh.bounds[1][1]) / 2.0 / 1000.0
    return (BELT_TOP_Z_M + SPAWN_CLEARANCE_M - lowest_m, -body_mid_y_m)


def spawn_height_for_mesh_m(mesh, quat):
    """Origin height that rests a source mesh just above the belt in this pose."""
    return spawn_pose_for_mesh_m(mesh, quat)[0]


def _load(slug):
    import trimesh

    stem, _ = ITEMS[slug]
    return trimesh.load(str(STL_DIR / f"{stem}.stl"), force="mesh")


def spawn_height_m(slug, quat):
    """Load an organizer STL and calculate its pose-dependent spawn height."""
    return spawn_pose_for_mesh_m(_load(slug), quat)[0]


def spawn_offset_y_m(slug, quat):
    """Sideways shift that centres the item's BODY across the belt in this pose."""
    return spawn_pose_for_mesh_m(_load(slug), quat)[1]


def main():
    if len(sys.argv) not in (4, 5):
        sys.exit("usage: spawn_orientations.py <seed> <item_index> <orient_index> [slug]")
    seed, item_index, orient_index = (int(a) for a in sys.argv[1:4])
    quat = orientation_quat(seed, item_index, orient_index)
    line = " ".join(f"{v:.9f}" for v in quat)
    if len(sys.argv) == 5:
        height_m, offset_y_m = spawn_pose_for_mesh_m(_load(sys.argv[4]), quat)
        line += f" {height_m:.4f} {offset_y_m:.4f}"
    print(line)


if __name__ == "__main__":
    main()
