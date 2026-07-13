#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Did an item end up in its zone? The single source of the episode verdict.

Both runners ask this — the single-item skeleton (scripts/run_skeleton.sh) and
the multi-item stream (scripts/run_stream.sh) — so the bands live here once. They
are the jury-visible success criterion of a run and must not drift apart between
scripts.

WHAT IS SCORED (day 11): the GOODS, not the pose Gazebo prints. Gazebo reports the
model ORIGIN, and ours is the default pose's BOTTOM (build_item_models.set_belt_origin:
XY at the bbox centre, Z=0 at the lowest point). Gazebo ROTATES the model about that
origin, so for a turned bulky item the reported point leaves the body: measured over
the seed-0 matrix, the origin sits up to 349 mm above the item's own contact point
(box_400 oi=2) and up to 234 mm away from its centre horizontally (pouf). Four of the
33 cells were therefore UNPASSABLE — pouf oi=1/oi=2, box_400 oi=2, helmet oi=2 lying
PERFECTLY FLAT in the cage still report an origin above the old z<0.25 gate — and
several more (box_300 oi=2, detergent oi=1, plate oi=1) passed only because the item
happened to tumble into a lower-origin resting pose. That is not physics noise; it is
the ruler. So `body` carries the item's real centre and lowest point, from the resting
orientation (scripts/body_pose.py). It DEFAULTS to the reported pose, which is exactly
right in the default pose — there the origin IS the contact point — so the pusher world,
run_stream and the CI fixture (all identity poses, no STL needed) are unaffected.

EVERY BAND IS A TERMINAL STATE (day 11). B used to read "x >= 3.5, still at belt
height" — but the D blade pivots at x=3.75 and reaches to x=4.15, so EVERY item bound
for D crosses x=3.5 on the belt on its way into the blade. The poll loop stops at the
first YES, so a B-expected item that was then diverted into D had already been scored
PASS. Not a hypothetical: bag oi=1 misrouted into the D cage (the K-straddle IR630 later
fixed) and census #1 scored it PASS for B, final pose y=-1.35, z=0.08 — on the floor, in
the wrong cage. B therefore now starts PAST the last mechanism's reach and requires the
item to still be within the belt's width, which no diverted item can satisfy.

THE BANDS describe the CONTAINER, so they follow the world's cages — they are the
union over both mechanism worlds (day 4 retro widened them: box_400 was routed
correctly yet FAILed by millimetres):
  B: rode past BOTH mechanisms, still on the belt (x >= 4.2, |y| <= 0.3, belt height)
  C: landed in the zone C roll-cage (x 1.9..4.0, y 0.5..1.475, resting on its floor)
  D: landed in the zone D roll-cage (x 2.4..4.5, y -1.475..-0.5, resting on its floor)
The pusher drops an item at its paddle x (cell.sdf patches at C=2.5, D=3.0), the
diverter funnels it down a chute (cell_diverter.sdf), so the x bands span both.
The upper x bounds are the diverter cages' END WALLS (x=4.0 and x=4.5): the cages
were lengthened to 1.6 m so that each one actually covers the chute that feeds it
— the C patch used to stop at x=3.6 while its chute delivered out to x=3.9, which
is how the pouf ended up outside the zone that was meant to catch it. The far-y
bound is the cage WALL's inner face (y=1.475/-1.475), NOT the decorative patch
edge (y=1.3/-1.3): a big item diverted into C settles AGAINST that wall with its
centre past the patch. Past the wall (|y|>1.5) is a real escape and still fails.

FLOOR_Z is the one gate that changed meaning. It used to be "origin below 0.25",
a number derived for a box whose origin happens to sit near its centre. It is now
"the body's LOWEST point is on the cage floor" — which is size-independent (a pouf
and a pen both rest at z~0) and says exactly what delivery means: the item came to
rest IN the container, not hung up on the chute that feeds it (a chute-stuck item
rests 100-160 mm up its ramp). B still requires belt height, so the two stay
unambiguous.

Usage:
    python3 scripts/zone_verdict.py <B|C|D> <x> <y> <z>
    python3 scripts/zone_verdict.py <B|C|D> <x> <y> <z> <slug> <roll> <pitch> <yaw>
    -> prints YES or no. The second form scores the BODY: given the resting
       orientation it measures the item's real centre and lowest point off its mesh.
       Without a usable mesh it falls back to the reported pose and says so on stderr.
"""
import sys

# The body rests ON the cage floor (z=0). Generous enough for solver settling, far
# below the 100-160 mm at which an item hung up on the chute's ramp comes to rest.
FLOOR_Z = 0.05
# Past this x no mechanism can touch the item any more: the D blade pivots at x=3.75
# and its plate is 0.80 m long, so it sweeps out to x=4.15 (sim/worlds/cell_diverter.sdf).
# An item still on the belt beyond it has finished its episode — which is what makes B
# a terminal state and the poll loop's early break sound.
PAST_MECHANISMS_X = 4.2
# The belt's edges are at y=+-0.25; allow a little overhang before calling the item off it.
BELT_HALF_WIDTH_Y = 0.3


def in_zone(zone, x, y, z, body=None):
    """True if the pose (metres, world frame) means the item reached `zone`.

    (x, y, z) is what Gazebo reports: the model ORIGIN. `body` is the goods —
    (centre_x, centre_y, lowest_z) — when the caller knows the resting orientation.
    It defaults to the reported pose, which IS the body's contact point in the
    default pose (see the module docstring).
    """
    bx, by, bz = (x, y, z) if body is None else body
    if zone == "B":
        return (bx >= PAST_MECHANISMS_X and abs(by) <= BELT_HALF_WIDTH_Y
                and 0.35 <= bz <= 1.0)
    if zone == "C":
        return 1.9 <= bx <= 4.0 and 0.5 <= by <= 1.475 and bz < FLOOR_Z
    if zone == "D":
        return 2.4 <= bx <= 4.5 and -1.475 <= by <= -0.5 and bz < FLOOR_Z
    raise ValueError(f"zone must be B, C or D, got '{zone}'")


def in_zone_legacy(zone, x, y, z):
    """The pre-day-11 ruler, verbatim. Kept ONLY to measure what the fix changed.

    It scored the model ORIGIN against a z<0.25 floor gate, and let B pass on a
    TRANSIENT (x>=3.5 at belt height — before the D blade at 3.75, so every D-bound
    item satisfied it in passing). run_skeleton.sh prints both verdicts during the
    transition census so the report can attribute each moved cell to the ruler rather
    than to physics noise. Delete once that census is published.
    """
    if zone == "B":
        return x >= 3.5 and 0.35 <= z <= 1.0
    if zone == "C":
        return 1.9 <= x <= 4.0 and 0.5 <= y <= 1.475 and z < 0.25
    if zone == "D":
        return 2.4 <= x <= 4.5 and -1.475 <= y <= -0.5 and z < 0.25
    raise ValueError(f"zone must be B, C or D, got '{zone}'")


def body_from_resting_pose(slug, x, y, z, roll, pitch, yaw):
    """(centre_x, centre_y, lowest_z) of the goods, or None if the mesh is unavailable.

    The organizer STLs are gitignored, so a clean checkout (CI) has no mesh to measure.
    That is safe to fall back from: CI spawns the default pose, where the origin is the
    contact point already.
    """
    try:
        from body_pose import body_pose_m, quat_from_rpy  # trimesh + the item's STL
        from body_pose import _load  # noqa: PLC2701  (same module's loader)

        quat = quat_from_rpy(roll, pitch, yaw)
        (cx, cy, _), lowest_z = body_pose_m(_load(slug), quat, (x, y, z))
        return (cx, cy, lowest_z)
    except Exception as exc:  # noqa: BLE001  (no mesh, no trimesh, unknown slug)
        print(f"zone_verdict: scoring the reported origin, not the body ({exc})",
              file=sys.stderr)
        return None


def main():
    argv = sys.argv[1:]
    legacy = argv[:1] == ["--legacy"]
    if legacy:
        argv = argv[1:]
    if len(argv) not in (4, 8):
        sys.exit("usage: zone_verdict.py [--legacy] <B|C|D> <x> <y> <z> "
                 "[<slug> <roll> <pitch> <yaw>]")
    zone = argv[0]
    try:
        x, y, z = (float(v) for v in argv[1:4])
    except ValueError:
        print("no")  # a lost item polls as 'nan nan nan' — that is simply not in its zone
        return

    if legacy:
        print("YES" if in_zone_legacy(zone, x, y, z) else "no")
        return

    body = None
    if len(argv) == 8:
        slug = argv[4]
        try:
            roll, pitch, yaw = (float(v) for v in argv[5:8])
        except ValueError:
            print("no")  # orientation unreadable: the item has no pose to score
            return
        body = body_from_resting_pose(slug, x, y, z, roll, pitch, yaw)

    print("YES" if in_zone(zone, x, y, z, body) else "no")


if __name__ == "__main__":
    main()
