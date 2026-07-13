#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Did an item end up in its zone? The single source of the episode verdict.

Both runners ask this — the single-item skeleton (scripts/run_skeleton.sh) and
the multi-item stream (scripts/run_stream.sh) — so the bands live here once. They
are the jury-visible success criterion of a run and must not drift apart between
scripts.

The bands describe the CONTAINER, so they follow the world's cages — they are the
union over both mechanism worlds (day 4 retro widened them: box_400 was routed
correctly yet FAILed by millimetres):
  B: rode past the mechanisms on the belt (x >= 3.5, still at belt height)
  C: landed in the zone C roll-cage (x 1.9..4.0, y 0.5..1.475, on the floor)
  D: landed in the zone D roll-cage (x 2.4..4.5, y -1.475..-0.5, on the floor)
The pusher drops an item at its paddle x (cell.sdf patches at C=2.5, D=3.0), the
diverter funnels it down a chute (cell_diverter.sdf), so the x bands span both.
The upper x bounds are the diverter cages' END WALLS (x=4.0 and x=4.5): the cages
were lengthened to 1.6 m so that each one actually covers the chute that feeds it
— the C patch used to stop at x=3.6 while its chute delivered out to x=3.9, which
is how the pouf ended up outside the zone that was meant to catch it. The far-y
bound is the cage WALL's inner face (y=1.475/-1.475), NOT the decorative patch
edge (y=1.3/-1.3): a big item diverted into C settles AGAINST that wall with its
centre past the patch — pouf oi=1 stops at y=1.43, contained by the cage yet 3 cm
outside the old 1.4 line, a deterministic mech_overshoot that census #2 passed only
on physics noise (docs/experiments.md 2026-07-13). Same class as the day-4 box_400
widening: the band describes the CONTAINER (walls), not the flat patch. Past the
wall (|y|>1.5) is a real escape and still fails. z<0.25 clears a 400 mm box standing
on its base at centre z~0.2 yet stays well under belt height z~0.45. B stays
unambiguous: C/D require the floor, B requires the belt.

Usage:
    python3 scripts/zone_verdict.py <B|C|D> <x> <y> <z>   -> prints YES or no
"""
import sys


def in_zone(zone, x, y, z):
    """True if the pose (metres, world frame) means the item reached `zone`."""
    if zone == "B":
        return x >= 3.5 and 0.35 <= z <= 1.0
    if zone == "C":
        return 1.9 <= x <= 4.0 and 0.5 <= y <= 1.475 and z < 0.25
    if zone == "D":
        return 2.4 <= x <= 4.5 and -1.475 <= y <= -0.5 and z < 0.25
    raise ValueError(f"zone must be B, C or D, got '{zone}'")


def main():
    if len(sys.argv) != 5:
        sys.exit("usage: zone_verdict.py <B|C|D> <x> <y> <z>")
    zone = sys.argv[1]
    try:
        x, y, z = (float(v) for v in sys.argv[2:5])
    except ValueError:
        print("no")  # a lost item polls as 'nan nan nan' — that is simply not in its zone
        return
    print("YES" if in_zone(zone, x, y, z) else "no")


if __name__ == "__main__":
    main()
