#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Steady-state throughput and per-stage latency of multi-item stream episodes.

Day 13, week 2 (P1 int): PLAN-WEEK2 criterion 4 asks for takt and items/min as a
MEDIAN and p95 over >=5 runs — not the single numbers day 10 quoted (1.3 s same
zone, 6.7 s zone change, 40 items/min, each from one episode).

WHICH CLOCK TIMES WHAT (there are three, so a segment is only ever a difference
WITHIN one log, never across):
  skeleton.log  ROS-logger wall-clock stamps on node stdout, per item_id the
                perception tracker assigned:
                  camera-first-detect   [perception]: item N: WxHxD mm K=..
                  classification publish [classifier]: item N: <cat> (k=..)
                  mechanism command      [controller]: item N: pusher_x FIRED ..
                => camera->decision and decision->command latencies.
  stream.log    run_stream.sh's own summary, T0-relative (T0 = full belt speed,
                so Gazebo boot and the soft-start ramp are excluded by
                construction), the body-scored verdict arrival per item:
                  itemK slug -> zone: PASS at t=Xs
                => the mechanism TAKT (gap between successive arrivals) and the
                   steady-state items/min.

The two logs are read INDEPENDENTLY: the tracker's item_id ("item 1" in
skeleton.log) and the spawn index ("item0" in stream.log) are different
namespaces, so nothing is joined across them — each log answers only the segments
its own clock can time.

CALC vs SIM: scripts/stream_plan.py owns the geometry. A zone change needs
(HOLD_S + RETRACT_S) * BELT_SPEED_M_S metres of air while the blade holds and
retracts, so at 1 m/s the takt FLOOR for a zone-changing pair is that many
seconds; same-zone pairs ride nose to tail (min gap 0 m), so their floor is the
item's own length / belt speed, which the geometry does not fix. We compare the
observed zone-change takt to the computed floor and report the gap.

Usage:
    python3 scripts/measure_throughput.py runs/stream_A runs/stream_B ...
    python3 scripts/measure_throughput.py runs/            # every stream run under it
"""
from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from statistics import median

# Run either as `python3 scripts/measure_throughput.py` (its own dir is on the
# path, but the repo root is not) or under pytest (conftest adds both). Mirror
# stream_plan.py's own bootstrap so `src` and the sibling script both import.
_REPO = Path(__file__).resolve().parent.parent
for _p in (str(_REPO), str(_REPO / "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from src.constants import BELT_SPEED_M_S  # noqa: E402
import stream_plan  # noqa: E402  (HOLD_S/RETRACT_S/geometry live here, not duplicated)

# A ROS-logger line: "... [<epoch>.<ns>] [<node>]: item <id>: <rest>". The stamp is
# system wall-clock (~1.78e9 s), shared by every node on the machine, so stamps
# from different nodes subtract cleanly.
ROS_LINE = re.compile(
    r"\[(?P<stamp>\d+\.\d+)\]\s+\[(?P<node>perception|classifier|controller)\]:\s+"
    r"item\s+(?P<id>\d+):\s+(?P<rest>.*)"
)
# run_stream.sh's arrival line: "itemK slug -> zone: PASS at t=Xs (pose ...)".
# Only PASS lines carry a landing time; a FAILed item never reached its zone.
ARRIVAL_LINE = re.compile(
    r"^(?P<name>item\d+)\s+(?P<slug>\S+)\s+->\s+(?P<zone>[BCD]):\s+PASS at t=(?P<t>[\d.]+)s"
)


@dataclass
class Stages:
    """The stage stamps of ONE tracked item within a single skeleton.log."""

    detect: float | None = None       # first [perception] line — camera saw it
    decide_cls: float | None = None   # first [classifier] line — category published
    decide_ctrl: float | None = None  # first [controller] decision line — fallback
    fire: float | None = None         # [controller] pusher_x FIRED — command issued

    @property
    def decide(self) -> float | None:
        """The decision moment: the classifier's publish, or the controller's own
        decision line when the classifier line was truncated (killing the launch on
        verdict drops late node stdout — see triage_matrix.py CONTROLLER_RE)."""
        return self.decide_cls if self.decide_cls is not None else self.decide_ctrl


@dataclass
class Arrival:
    name: str
    slug: str
    zone: str
    t: float  # T0-relative seconds


def parse_skeleton(path: Path) -> dict[int, Stages]:
    """Per tracked item_id, the earliest stamp of each stage seen in skeleton.log."""
    items: dict[int, Stages] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        m = ROS_LINE.search(line)
        if not m:
            continue
        iid = int(m["id"])
        t = float(m["stamp"])
        rest = m["rest"]
        s = items.setdefault(iid, Stages())
        node = m["node"]
        if node == "perception":
            if s.detect is None or t < s.detect:
                s.detect = t
        elif node == "classifier":
            if s.decide_cls is None or t < s.decide_cls:
                s.decide_cls = t
        elif node == "controller":
            if "FIRED" in rest:
                if s.fire is None or t < s.fire:
                    s.fire = t
            elif rest[:1] in ("B", "C", "D"):  # "D — firing.." / "B — rides.."
                if s.decide_ctrl is None or t < s.decide_ctrl:
                    s.decide_ctrl = t
    return items


def parse_stream(path: Path) -> list[Arrival]:
    """The body-scored arrivals from run_stream.sh's saved summary, sorted by time."""
    out = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        m = ARRIVAL_LINE.match(line.strip())
        if m:
            out.append(Arrival(m["name"], m["slug"], m["zone"], float(m["t"])))
    return sorted(out, key=lambda a: a.t)


@dataclass
class TaktGap:
    gap_s: float       # observed time between two successive arrivals
    front_zone: str
    back_zone: str
    expected_s: float  # computed floor for this pair (stream_plan geometry)


def takt_gaps(arrivals: list[Arrival]) -> list[TaktGap]:
    """Successive-arrival gaps within one run, each tagged with its computed floor.

    The belt is FIFO, so arrival order is feed order: the front item of a pair is the
    one that landed first. min_gap_between_zones_m gives the metres the back item must
    trail; at the belt speed that is the takt floor in seconds.
    """
    gaps = []
    for front, back in zip(arrivals, arrivals[1:]):
        floor_m = stream_plan.min_gap_between_zones_m(front.zone, back.zone)
        gaps.append(TaktGap(back.t - front.t, front.zone, back.zone,
                            floor_m / BELT_SPEED_M_S))
    return gaps


def percentile(xs: list[float], p: float) -> float | None:
    """Linear-interpolation percentile (p in 0..100); None on empty input."""
    if not xs:
        return None
    s = sorted(xs)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * p / 100.0
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def _row(label: str, xs: list[float]) -> str:
    if not xs:
        return f"  {label:<26s} n=0    (no data)"
    return (f"  {label:<26s} n={len(xs):<3d} "
            f"median={median(xs):7.3f}s  p95={percentile(xs, 95):7.3f}s")


def find_runs(paths: list[str]) -> list[Path]:
    """Each path is a run dir (has skeleton.log or stream.log) or a parent of them."""
    runs = []
    for raw in paths:
        p = Path(raw)
        if (p / "skeleton.log").exists() or (p / "stream.log").exists():
            runs.append(p)
            continue
        for sub in sorted(p.glob("*")):
            if sub.is_dir() and ((sub / "skeleton.log").exists()
                                 or (sub / "stream.log").exists()):
                runs.append(sub)
    return runs


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="stream run dir(s), or a parent of them")
    args = ap.parse_args()

    runs = find_runs(args.paths)
    if not runs:
        print("no stream runs found (need a dir with skeleton.log or stream.log)")
        return 1

    cam_to_dec: list[float] = []
    dec_to_cmd: list[float] = []
    all_takts: list[float] = []
    change_takts: list[float] = []   # zone change: the blade must hold+retract (floor > 0)
    noair_takts: list[float] = []    # nose to tail, or a B item riding through (floor 0)
    n_runs_timed = 0

    print(f"throughput over {len(runs)} run(s):\n")
    for run in runs:
        skel = run / "skeleton.log"
        strm = run / "stream.log"
        if skel.exists():
            for s in parse_skeleton(skel).values():
                if s.detect is not None and s.decide is not None and s.decide >= s.detect:
                    cam_to_dec.append(s.decide - s.detect)
                if s.decide is not None and s.fire is not None and s.fire >= s.decide:
                    dec_to_cmd.append(s.fire - s.decide)
        if strm.exists():
            arrivals = parse_stream(strm)
            gaps = takt_gaps(arrivals)
            if gaps:
                n_runs_timed += 1
                run_takt = median([g.gap_s for g in gaps])
                print(f"  {run.name}: {len(arrivals)} arrivals, "
                      f"takt {run_takt:.2f}s ({60.0 / run_takt:.0f}/min)")
            for g in gaps:
                all_takts.append(g.gap_s)
                (change_takts if g.expected_s > 0 else noair_takts).append(g.gap_s)
        else:
            print(f"  {run.name}: no stream.log (skeleton latencies only)")

    print("\n=== per-stage latency (skeleton.log, per tracked item) ===")
    print(_row("camera -> decision", cam_to_dec))
    print(_row("decision -> command", dec_to_cmd))

    print("\n=== mechanism takt (stream.log, between arrivals) ===")
    print(_row("takt: all pairs", all_takts))
    print(_row("takt: nose-to-tail (floor 0)", noair_takts))
    print(_row("takt: zone-change (needs air)", change_takts))

    if all_takts:
        thr_med = 60.0 / median(all_takts)
        thr_p95 = 60.0 / percentile(all_takts, 95)  # slowest takt -> lowest rate
        print(f"\n  steady-state throughput: median {thr_med:.0f} items/min "
              f"(p95-slow takt: {thr_p95:.0f} items/min), "
              f"over {n_runs_timed} timed run(s)")

    # Calc vs sim: the geometry's floors against what the stream actually did.
    change_floor = (stream_plan.HOLD_S + stream_plan.RETRACT_S) / BELT_SPEED_M_S
    transit = (stream_plan.TARGET_X_M - stream_plan.FIRST_SPAWN_X_M) / BELT_SPEED_M_S
    print("\n=== computed floor (stream_plan.py geometry) vs observed ===")
    print(f"  zone-change takt floor: (HOLD {stream_plan.HOLD_S:.1f} + RETRACT "
          f"{stream_plan.RETRACT_S:.1f}) / {BELT_SPEED_M_S:.1f} m/s = {change_floor:.2f}s")
    if change_takts:
        obs = median(change_takts)
        print(f"    observed zone-change takt: median {obs:.2f}s "
              f"=> {obs - change_floor:+.2f}s over the floor")
    print("  same-zone takt floor: item length / belt speed (geometry-independent)")
    print(f"  per-item belt transit (spawn x={stream_plan.FIRST_SPAWN_X_M:.1f} -> "
          f"target x={stream_plan.TARGET_X_M:.1f}): {transit:.1f}s at {BELT_SPEED_M_S:.1f} m/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
