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
    python3 scripts/spawn_orientations.py <seed> <item_index> <orient_index>
    -> prints the quaternion as "x y z w"
"""
import sys

import numpy as np


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


def main():
    if len(sys.argv) != 4:
        sys.exit("usage: spawn_orientations.py <seed> <item_index> <orient_index>")
    seed, item_index, orient_index = (int(a) for a in sys.argv[1:4])
    x, y, z, w = orientation_quat(seed, item_index, orient_index)
    print(f"{x:.9f} {y:.9f} {z:.9f} {w:.9f}")


if __name__ == "__main__":
    main()
