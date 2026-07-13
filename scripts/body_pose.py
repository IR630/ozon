#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Where is the item's BODY, given the pose Gazebo reports for its ORIGIN?

Gazebo reports a model's ORIGIN pose, and ours is the default pose's BOTTOM
(build_item_models.set_belt_origin: XY at the bounding-box centre, Z=0 at the
lowest point). The simulator then ROTATES the model about that origin — so for
any item resting at a turned angle, the reported point is not on the goods at
all. Measured about the seed-0 poses:

    pouf oi=1   origin sits 268 mm above the body's contact point
    pouf oi=2   342 mm
    helmet oi=2 255 mm

That gap is why the same origin convention has already produced two bugs (the
spawn height and the sideways centring, docs/decisions.md 2026-07-12): a number
that reads like "the item" is really "an arbitrary corner of its bounding box in
some other pose". This module is the third and last place that needed to stop
confusing the two — the episode VERDICT, which asks whether the item came to rest
in its container, and until now asked it of the origin.

The verdict-relevant quantities are both properties of the body, not the origin:

    centre       — is the item inside the cage footprint (x/y)?
    lowest point — is it ON the cage floor, or hung up on the chute above it?

The lowest point is the honest test for "delivered": it is size-independent (a
pouf and a pen both rest at z~0), whereas any threshold on a centre or an origin
has to be re-derived per item and per pose, which is exactly how a 489 mm pouf
lying correctly in the cage came to be scored a failure.

Usage (called by scripts/run_skeleton.sh with the resting pose it just polled):
    python3 scripts/body_pose.py <slug> <x> <y> <z> <roll> <pitch> <yaw>
"""
import sys
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))          # build_item_models (sibling script)

from build_item_models import ITEMS, STL_DIR, set_belt_origin  # noqa: E402


def body_offsets_m(mesh, quat):
    """(centre, lowest z) of a source mesh in pose `quat`, RELATIVE to its origin.

    Metres. `mesh` is a source STL in mm about an arbitrary origin; set_belt_origin
    puts it on the model convention first, so the offsets returned are exactly what
    Gazebo's reported origin pose is missing.
    """
    import trimesh  # heavy; only callers with a real mesh need it

    set_belt_origin(mesh)
    x, y, z, w = quat
    mesh.apply_transform(trimesh.transformations.quaternion_matrix([w, x, y, z]))
    lo, hi = mesh.bounds / 1000.0  # STL is in mm
    centre = tuple((lo + hi) / 2)
    return centre, float(lo[2])


def body_pose_m(mesh, quat, origin_xyz):
    """(centre, lowest z) of the body in WORLD metres, from the origin pose Gazebo reports."""
    (cx, cy, cz), low_dz = body_offsets_m(mesh, quat)
    ox, oy, oz = origin_xyz
    return (ox + cx, oy + cy, oz + cz), oz + low_dz


def quat_from_rpy(roll, pitch, yaw):
    """(x, y, z, w) from the RPY radians `ign model --pose` prints."""
    import trimesh

    m = trimesh.transformations.euler_matrix(roll, pitch, yaw, "sxyz")
    w, x, y, z = trimesh.transformations.quaternion_from_matrix(m)
    return (float(x), float(y), float(z), float(w))


def _load(slug):
    import trimesh

    stem, _ = ITEMS[slug]
    return trimesh.load(str(STL_DIR / f"{stem}.stl"), force="mesh")


def dump_hull(slug, path):
    """Write the item's convex hull (mm, about the model origin) as plain 'x y z' lines.

    The episode verdict polls up to 60 times, and importing trimesh costs 1.33 s each
    time (numpy alone costs 0.51 s) — enough to push a cell past its 180 s timeout. So
    the mesh is reduced to its hull ONCE per episode here, and scripts/zone_verdict.py
    then rotates those few hundred points with the standard library. The hull is exactly
    enough: the extreme point in any direction lies on it, so it carries the whole
    bounding box the verdict measures.
    """
    mesh = _load(slug)
    set_belt_origin(mesh)
    with open(path, "w", encoding="utf-8") as f:
        for px, py, pz in mesh.convex_hull.vertices:
            f.write(f"{px:.3f} {py:.3f} {pz:.3f}\n")


def main():
    if len(sys.argv) == 4 and sys.argv[1] == "--dump-hull":
        dump_hull(sys.argv[2], sys.argv[3])
        return
    if len(sys.argv) != 8:
        sys.exit("usage: body_pose.py <slug> <x> <y> <z> <roll> <pitch> <yaw>\n"
                 "       body_pose.py --dump-hull <slug> <path>")
    slug = sys.argv[1]
    try:
        x, y, z, roll, pitch, yaw = (float(v) for v in sys.argv[2:8])
    except ValueError:
        print("body: unknown (item lost — pose polled as nan)")
        return
    quat = quat_from_rpy(roll, pitch, yaw)
    (cx, cy, cz), low_z = body_pose_m(_load(slug), quat, (x, y, z))
    print(f"body: centre x={cx:.3f} y={cy:.3f} z={cz:.3f} | lowest z={low_z:+.3f} "
          f"| origin-to-body dz={cz - z:+.3f}")


if __name__ == "__main__":
    main()
