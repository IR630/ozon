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
    -> prints "x y z w" — plus, given the slug, the spawn HEIGHT for that pose.

The height is not a constant. Gazebo creates the item at its CENTRE, so a fixed
z=0.5 buries anything over 200 mm tall inside the belt (top surface at 0.4) and
the solver ejects it: box_400 turned on edge is 579 mm tall in its oi=2 pose —
it starts 190 mm INSIDE the belt and never leaves the spawn. That, not the rail
gap, is what the seed-0 census scored as feed_jam (proven by replaying the cell
with only this number changed: z=0.5 FAIL at the spawn, z=0.71 PASS).
"""
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))          # build_item_models (sibling script)
sys.path.insert(0, str(_HERE.parent))   # src/ (running as a script, cwd is not on the path)
from build_item_models import ITEMS, STL_DIR  # noqa: E402  (slug -> STL, single source)

from src.perception import BELT_TOP_Z_M  # noqa: E402  (belt top, single source)

# Air gap under the item at spawn. Keep it SMALL: the item is dropped from this
# gap, and 20 mm was enough to bounce round bodies (pouf, helmet) hard enough to
# blow their contacts up — physics_wedge/TIMEOUT cells appeared right after the
# spawn fix landed. Replaying pouf oi=1 with only this number changed: 20 mm
# FAIL (wedged on the chute, cycle 123 s), 5 mm PASS (cycle 69 s).
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


def spawn_height_m(slug, quat):
    """Centre height that rests the item ON the belt in this orientation, not in it."""
    import trimesh  # heavy; only the matrix needs it

    stem, _ = ITEMS[slug]
    mesh = trimesh.load(str(STL_DIR / f"{stem}.stl"), force="mesh")
    x, y, z, w = quat
    mesh.apply_transform(trimesh.transformations.quaternion_matrix([w, x, y, z]))
    height_m = (mesh.bounds[1][2] - mesh.bounds[0][2]) / 1000.0  # STL is in mm
    return BELT_TOP_Z_M + height_m / 2 + SPAWN_CLEARANCE_M


def main():
    if len(sys.argv) not in (4, 5):
        sys.exit("usage: spawn_orientations.py <seed> <item_index> <orient_index> [slug]")
    seed, item_index, orient_index = (int(a) for a in sys.argv[1:4])
    quat = orientation_quat(seed, item_index, orient_index)
    line = " ".join(f"{v:.9f}" for v in quat)
    if len(sys.argv) == 5:
        line += f" {spawn_height_m(sys.argv[4], quat):.4f}"
    print(line)


if __name__ == "__main__":
    main()
