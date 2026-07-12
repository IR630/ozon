#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Can each census cell physically ride the belt at all? (day 10 triage)

The census spawns every item in UNIFORM-RANDOM 3D orientations. For a big item
that is not a harmless variation: the pouf is 489 mm across, the belt is 500 mm
wide and the infeed rails leave a 504 mm gap — so a rotated pouf can present a
cross-section WIDER THAN THE BELT IT RIDES ON. Such a cell is not a routing
error to be fixed in perception or the controller; the item cannot be conveyed
in that pose by any sorter with this belt.

This tool measures, per cell, the item's lateral (y) extent in its seeded spawn
pose and compares it against the belt and the rails. Pure geometry — no Gazebo.

    python3 scripts/check_belt_fit.py [seed] [N]

Prints one line per cell that does not fit, and a summary.
"""
import sys
from pathlib import Path

import trimesh

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent))

from build_item_models import ITEMS, STL_DIR, set_belt_origin  # noqa: E402
from spawn_orientations import orientation_quat  # noqa: E402

# sim/worlds/cell_diverter.sdf: belt box 0.5 m wide (edges at +-0.25); the infeed
# rails sit at y = +-0.272 with 0.04 m thickness, so their inner faces leave 504 mm.
BELT_WIDTH_MM = 500.0
RAIL_GAP_MM = 504.0

SLUGS = ["bottle", "box_300x200x200", "box_400x400x300", "lunchbox", "bag",
         "detergent", "pouf", "pen", "plate", "cylinder", "helmet"]


def lateral_extent_for_mesh_mm(mesh, quat):
    """Width across the belt (y) of a source mesh rotated into its spawn pose."""
    set_belt_origin(mesh)
    x, y, z, w = quat
    mesh.apply_transform(trimesh.transformations.quaternion_matrix([w, x, y, z]))
    lo, hi = mesh.bounds
    return float(hi[1] - lo[1])


def fits_the_belt(width_mm):
    """Why this pose cannot be conveyed, or None if it can."""
    if width_mm > RAIL_GAP_MM:
        return f"WIDER THAN THE RAIL GAP ({RAIL_GAP_MM:.0f} mm)"
    if width_mm > BELT_WIDTH_MM:
        return f"WIDER THAN THE BELT ({BELT_WIDTH_MM:.0f} mm)"
    return None


def lateral_extent_mm(slug, quat):
    """Width across the belt (y) of an organizer item in this spawn pose."""
    stem, _ = ITEMS[slug]
    mesh = trimesh.load(str(STL_DIR / f"{stem}.stl"), force="mesh")
    return lateral_extent_for_mesh_mm(mesh, quat)


def main():
    seed = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 3

    too_wide = []
    for item_index, slug in enumerate(SLUGS):
        for orient_index in range(n):
            quat = orientation_quat(seed, item_index, orient_index)
            width_mm = lateral_extent_mm(slug, quat)
            verdict = fits_the_belt(width_mm)
            if verdict:
                too_wide.append((slug, orient_index, width_mm))
                print(f"{slug} oi={orient_index}: {width_mm:.0f} mm across — {verdict}")

    total = len(SLUGS) * n
    print(f"\n{len(too_wide)}/{total} cells cannot ride this belt in their spawn pose "
          f"(belt {BELT_WIDTH_MM:.0f} mm, rails {RAIL_GAP_MM:.0f} mm).")
    print("Such a cell is a limit of the CELL, not a routing error: no sorter with "
          "this belt can convey an item wider than the belt.")


if __name__ == "__main__":
    main()
