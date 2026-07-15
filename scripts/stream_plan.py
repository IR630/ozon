#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Feed plan for one multi-item stream episode (day 10, week 2).

Until now every episode carried ONE item (scripts/run_skeleton.sh). A stream
feeds 2-3 items onto the belt one after another, which turns two silent scene
properties into hard constraints — this module states both and refuses a plan
that breaks them, instead of letting Gazebo produce a plausible-looking wrong
run:

1. BELT STROKE. The conveyor is a prismatic slab: it slides, and at its joint
   limit it stops dead. Its stroke is the episode's fuel, and a stream burns the
   feed intervals ON TOP of the routing distance one item needs.

2. MECHANISM HOLD. The diverter blade is a WALL held across the belt for hold_s
   after firing. Any item reaching the blade during that hold is swept into the
   same zone, whatever its own category. So two items bound for DIFFERENT zones
   must be at least (hold + retract) * belt_speed apart, while two bound for the
   SAME zone may share one blade hold only while transport keeps their bodies
   separated. A slow leading item can still be caught by its follower before the
   camera, so measured transport gaps are enforced independently of cross-fire.

Items are fed from ONE point (FIRST_SPAWN_X_M) at intervals in GAZEBO SIMULATION
TIME, the way an infeed actually works — the gap in metres is the interval times
belt speed and therefore does not shrink when the simulator runs below real time.
The first stream run instead queued the items along the belt, each spawning
metres further back, and the item farthest back arrived 190 mm off-centre: the
belt has low lateral friction on purpose (mu2=0.2, so the blade can slide items
sideways), so a long feed lets an item drift until it rolls off the edge on its
own. Same feed point = same feed path for every item, and the only thing the
stream changes is the interval.

Usage (called once per episode from scripts/run_stream.sh):
    python3 scripts/stream_plan.py <world.sdf> <slug:zone:gap_m> ...
    -> one line per item: "<index> <slug> <zone> <spawn_x> <spawn_delay_s>"

gap_m is the distance BEHIND the previous item (the first item's gap is 0: it is
fed the moment the belt reaches full speed).
"""
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # src/ (cwd is not on the path)

from src.constants import BELT_SPEED_M_S  # noqa: E402
from scripts.zone_verdict import PAST_MECHANISMS_X  # noqa: E402

# Feed point — the spot a single-item episode spawns at (run_skeleton.sh), inside
# the infeed rails and far enough upstream that the item settles before the camera.
FIRST_SPAWN_X_M = -1.5
# How far down the belt an item must be carried for its episode to be decided.
# A B item is terminal only after it has passed the last blade's full sweep; keep
# this coupled to the body-scored verdict instead of restating its x threshold.
TARGET_X_M = PAST_MECHANISMS_X
# Belt already spent on the controller's soft-start ramp before the first item is
# fed (8 steps x 0.3 s at a rising speed, src/controller_node.py). Charged to the
# stroke budget so the guard errs on the side of "the belt will still be running".
RAMP_TRAVEL_M = 1.5
# Diverter hold and retract, mirroring the controller's mechanism parameters
# (hold_s, exported by run_skeleton.sh for the diverter world; _STROKE_S in
# src/controller_node.py). Overridable together with them via the environment.
HOLD_S = float(os.environ.get("HOLD_S", 2.5))
RETRACT_S = float(os.environ.get("RETRACT_S", 0.6))

ZONES = ("B", "C", "D")
# The convex Pouf rides this low-friction belt at 0.44 m/s while a flat Pen rides
# at 1.00 m/s. With a 1.0-1.5 m timed feed gap the Pen catches it before the
# camera (measured centre distance 0.17 m), the two bodies become one depth mask,
# and the Pen can reach C only accidentally under the Pouf's still-engaged blade.
# 2.5 m was the first differential replay with a complete 2/2
# perception/classifier/controller roster. This is an infeed constraint, not a
# classifier threshold; a future singulator or belt-physics fix may replace it.
MIN_GAP_AFTER_SLOW_ITEM_M = {"pouf": 2.5}


def min_gap_between_zones_m(front_zone, back_zone,
                            hold_s=HOLD_S, retract_s=RETRACT_S,
                            belt_speed_m_s=BELT_SPEED_M_S):
    """Metres the back item must trail the front one to avoid a cross-fire.

    Zero when the front item fires no mechanism (B rides past), or when both go
    to the same zone — there the blade holding for the front item is exactly the
    wall the back item needs. Otherwise the back item must not reach the blade
    before it has fired, held and retracted.
    """
    if front_zone == "B" or front_zone == back_zone:
        return 0.0
    return (hold_s + retract_s) * belt_speed_m_s


def min_transport_gap_m(front_slug):
    """Measured feed gap that prevents a faster follower catching `front_slug`."""
    return MIN_GAP_AFTER_SLOW_ITEM_M.get(front_slug, 0.0)


def belt_stroke_m(world_path):
    """Travel the conveyor slab has before it hits its joint limit, from the world."""
    world = ET.parse(world_path).getroot()
    joints = world.findall(".//model[@name='conveyor']/joint[@name='belt_joint']")
    if len(joints) != 1:
        raise ValueError(f"{world_path}: expected one conveyor/belt_joint, found {len(joints)}")
    limit = joints[0].find("axis/limit")
    return float(limit.find("upper").text) - float(limit.find("lower").text)


def parse_spec(spec):
    """'slug:zone:gap_m' -> (slug, zone, gap_m)."""
    parts = spec.split(":")
    if len(parts) != 3:
        raise ValueError(f"spec must be slug:zone:gap_m, got '{spec}'")
    slug, zone, gap = parts
    if zone not in ZONES:
        raise ValueError(f"zone must be one of {ZONES}, got '{zone}' in '{spec}'")
    try:
        gap_m = float(gap)
    except ValueError:
        raise ValueError(f"gap must be a number, got '{gap}' in '{spec}'") from None
    if gap_m < 0:
        raise ValueError(f"gap must not be negative, got {gap_m} in '{spec}'")
    return slug, zone, gap_m


def plan_stream(specs, stroke_m, belt_speed_m_s=BELT_SPEED_M_S):
    """Feed delay per item, or ValueError naming the constraint the stream breaks.

    Returns (slug, zone, delay_s) per item — the delay measured from the moment
    the belt reaches full speed. Every item is fed from the same point, so a gap
    of `gap_m` metres between two items is `gap_m / belt_speed` seconds of feed.
    """
    if not specs:
        raise ValueError("stream needs at least one item")

    items = []
    feed_m = 0.0  # belt travelled between the first feed and this one
    for index, spec in enumerate(specs):
        slug, zone, gap_m = parse_spec(spec)
        if index > 0:
            front_slug, front_zone, _ = items[-1]
            zone_required_m = min_gap_between_zones_m(front_zone, zone)
            transport_required_m = min_transport_gap_m(front_slug)
            required_m = max(zone_required_m, transport_required_m)
            if gap_m < required_m:
                if transport_required_m > zone_required_m:
                    raise ValueError(
                        f"item {index} ({slug} -> {zone}) trails slow {front_slug} "
                        f"by {gap_m} m, but measured transport needs "
                        f"{transport_required_m:.1f} m: a faster follower catches it "
                        "before the camera and the two bodies become one depth mask")
                raise ValueError(
                    f"item {index} ({slug} -> {zone}) trails {front_slug} -> {front_zone} "
                    f"by {gap_m} m, but a zone change needs {required_m:.1f} m: the "
                    f"{front_zone} blade is still a wall across the belt and would sweep "
                    f"{slug} into {front_zone} too")
            feed_m += gap_m
        items.append((slug, zone, feed_m / belt_speed_m_s))

    # The belt must still be moving when the LAST item finishes routing: it pays
    # the ramp, then every feed interval, then that item's own run to TARGET_X.
    travel_m = RAMP_TRAVEL_M + feed_m + (TARGET_X_M - FIRST_SPAWN_X_M)
    if travel_m > stroke_m:
        raise ValueError(
            f"stream needs {travel_m:.1f} m of belt travel ({feed_m:.1f} m of feed "
            f"intervals plus {TARGET_X_M - FIRST_SPAWN_X_M:.1f} m of routing), but the "
            f"conveyor slab has {stroke_m:.1f} m of stroke before its joint limit — "
            "it would stop under the stream")
    return items


def main():
    if len(sys.argv) < 3:
        sys.exit("usage: stream_plan.py <world.sdf> <slug:zone:gap_m> ...")
    world_path = sys.argv[1]
    try:
        items = plan_stream(sys.argv[2:], belt_stroke_m(world_path))
    except ValueError as e:
        sys.exit(f"ABORT: {e}")
    for index, (slug, zone, delay_s) in enumerate(items):
        print(f"{index} {slug} {zone} {FIRST_SPAWN_X_M:.3f} {delay_s:.2f}")


if __name__ == "__main__":
    main()
