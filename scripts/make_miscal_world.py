# -*- coding: utf-8 -*-
"""Bake an extrinsic calibration error into the side heads of the 3-cam world.

WHY THIS EXISTS. In Gazebo the heads sit exactly where `src/constants.py` says,
so the mutual calibration error is ZERO — and that error is the whole mechanism
by which extra heads lose on a real line. The census therefore answered "does the
contour break" on a rig no integrator will ever own. This script moves the two
side heads in the WORLD and leaves `src/constants.py` alone: the world becomes
the truth, the node keeps believing the nominal poses, and the difference is
exactly an extrinsic calibration error. Nothing in the node changes.

THE DIRECTION IS THE WORST CORNER OF THE BUDGET, NOT A RANDOM DRAW. A +-2 mm /
+-0.2 deg spec has to hold at its corner, so both heads are pushed OUTWARD along
y (each head's cloud follows its lens, so the two errors add: +4 mm on the fused
y extent) and tilted 0.2 deg in pitch (~3 mm of lateral spread at the 0.9 m
working distance, landing on the z extent — the dimension the pen's C verdict
hangs on). Surviving here is a stronger statement than surviving a lucky draw.

This is HARSHER than the offline probe's model, and deliberately so: the probe
rotates each cloud about its own centroid (~0.5 mm at an item's radius), while a
real head rotates about its LENS, which is 0.9 m away from what it is looking at.

    python3 scripts/make_miscal_world.py [out.sdf] [src.sdf]

The two-head rig is miscalibrated the same way, by naming its world:

    python3 scripts/make_miscal_world.py \\
        sim/worlds/cell_diverter_2cam_miscal.sdf sim/worlds/cell_diverter_2cam.sdf

THE BUDGET IS A SEAM, because one budget answers only half the question. The
offline matrix (docs/experiments.md 26.07) found the rig ORDER INVERTS between
calibration columns: at the typical 2 mm / 0.2 deg budget 3A leads two heads
92 % against 82 %, and in the worst column it trails, 82-83 % against 86 %. A
live census run at one budget therefore cannot say which rig to ship. The two
budgets below are the matrix's own columns (scripts/probe_camera_count.py
CALIBRATIONS), so the live numbers land on the offline ones:

    --shift-mm 2 --tilt-deg 0.2    # "типичная"      (default)
    --shift-mm 3 --tilt-deg 0.3    # "посредственная" — the column where 3A lost

Only the heads the source world actually carries are moved, and which ones they
were is printed: a rig quietly miscalibrated on fewer heads than intended is a
census nobody can attribute.
"""
import re
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent.parent
SRC_WORLD = _ROOT / "sim" / "worlds" / "cell_diverter_3cam.sdf"
OUT_WORLD = _ROOT / "sim" / "worlds" / "cell_diverter_3cam_miscal.sdf"

# The calibration budget being tested, from scripts/probe_camera_count.py
# CALIBRATIONS "типичная 2 мм / 0.2°".
SHIFT_MM = 2.0
TILT_DEG = 0.2

# (model name, outward sign along y) — the top head is untouched: it is the one
# head whose calibration a single-camera rig also has to get right.
_SIDE_HEADS = (("camera_side_neg_y", -1.0), ("camera_side_pos_y", +1.0))


def heads_present(sdf_text):
    """The subset of the side heads this world actually carries, in rig order."""
    return tuple((name, sign) for name, sign in _SIDE_HEADS
                 if '<model name="%s">' % name in sdf_text)


def miscalibrate(sdf_text, shift_mm=SHIFT_MM, tilt_deg=TILT_DEG, heads=_SIDE_HEADS):
    """Return the world with the named side heads displaced by the calibration error."""
    import math

    out = sdf_text
    for name, sign in heads:
        pattern = re.compile(
            r'(<model name="%s">\s*\n\s*<static>true</static>\s*\n\s*<pose>)'
            r"([^<]+)(</pose>)" % re.escape(name))
        match = pattern.search(out)
        if match is None:
            raise ValueError("side head %r not found in the world" % name)
        x, y, z, roll, pitch, yaw = (float(v) for v in match.group(2).split())
        y += sign * shift_mm / 1000.0
        pitch += math.radians(tilt_deg)
        # 9 significant digits: the tilt is 3.5e-3 rad and a 6-digit round-trip
        # loses a percent of it — the error under test must survive its own file.
        pose = "%.9g %.9g %.9g %.9g %.9g %.9g" % (x, y, z, roll, pitch, yaw)
        out = out[:match.start(2)] + pose + out[match.end(2):]
    return out


def main(argv):
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("out", nargs="?", default=OUT_WORLD, type=Path)
    parser.add_argument("src", nargs="?", default=SRC_WORLD, type=Path)
    parser.add_argument("--shift-mm", type=float, default=SHIFT_MM,
                        help="outward y displacement per side head (default %(default)s)")
    parser.add_argument("--tilt-deg", type=float, default=TILT_DEG,
                        help="pitch error per side head (default %(default)s)")
    args = parser.parse_args(argv)

    src = args.src.read_text(encoding="utf-8")
    heads = heads_present(src)
    if not heads:
        print("ABORT: %s carries no side head to miscalibrate" % args.src)
        return 2
    args.out.write_text(
        miscalibrate(src, shift_mm=args.shift_mm, tilt_deg=args.tilt_deg, heads=heads),
        encoding="utf-8")
    print("miscalibrated world: %s from %s (%+g mm outward, %+g deg pitch, heads: %s)"
          % (args.out, args.src, args.shift_mm, args.tilt_deg,
             ", ".join(name for name, _sign in heads)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
