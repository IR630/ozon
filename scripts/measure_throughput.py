#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Steady-state throughput and per-stage latency of multi-item stream episodes.

Day 13, week 2 (P1 int): PLAN-WEEK2 criterion 4 asks for takt and items/min as a
MEDIAN and p95 over >=5 runs — not the single numbers day 10 quoted (1.3 s same
zone, 6.7 s zone change, 40 items/min, each from one episode).

WHAT THE LOGS CAN AND CANNOT TIME (found by reading a real run, not assumed):

  Three separate PROCESSES write skeleton.log — [python3-2] perception,
  [python3-3] classifier, [python3-4] controller — and the "[<stamp>]" is each
  node's own LOG-EMISSION time. Across processes those stamps carry ~0.2 s of
  scheduling/buffering jitter: in runs/stream_validate the controller logged its
  route commit 0.2 s BEFORE perception logged the frame that caused it. So a
  segment that is genuinely ~one camera frame (~0.05 s), like camera->decision,
  is BELOW that noise floor and even goes negative — it is not separately
  measurable from these logs, and pretending otherwise (the first cut did, and a
  >= guard then silently dropped every item whose stamps happened to invert)
  is the Karpathy-#6 trap.

  What survives:
    * MECHANISM TAKT / throughput — from stream.log, written by ONE process (the
      run_stream.sh poll loop), so its arrival times share a clock with no
      cross-process jitter. T0-relative (T0 = full belt speed), so Gazebo boot
      and the soft-start ramp are excluded by construction.
        itemK slug -> zone: PASS at t=Xs        (body-scored verdict, not origin)
    * decision -> command — controller route commit to FIRED, BOTH logged by the
      controller ([python3-4]): same process, same clock, no cross-node jitter.
        [controller]: item N: D — firing pusher_d in Xs   (first = commit)
        [controller]: item N: pusher_x FIRED at t=Ys
    * camera -> command (detect -> FIRED) — spans perception to controller, so it
      carries the jitter, but it is seconds-scale so 0.2 s is tolerable.
        [perception]: item N: WxHxD mm K=.. at (..)        (first = detect)

  The classifier publishes an ItemClassification EVERY frame (17x for the pen,
  confidence 0.20 -> 0.99), so "the ItemClassification publish" is not one moment;
  its first publish coincides with detection to within the jitter. The controller's
  route commit is the actionable decision, and that is what "decision" means here.

  skeleton.log's tracker item_id ("item 1") and stream.log's spawn index ("item0")
  are different namespaces, so the two logs are read INDEPENDENTLY — never joined.

CALC vs SIM: scripts/stream_plan.py owns the geometry. A zone change needs
(HOLD_S + RETRACT_S) * BELT_SPEED_M_S metres of air while the blade holds and
retracts, so at 1 m/s the takt FLOOR for a zone-changing pair is that many
seconds; same-zone pairs ride nose to tail (min gap 0 m). We compare the observed
zone-change takt to the computed floor and report the gap.

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
# each node's own log-emission wall-clock time (~1.78e9 s).
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
    """The stage stamps of ONE tracked item within a single skeleton.log.

    Every field is the EARLIEST line of its kind, so detect is first camera sight
    and commit is the moment the controller first committed a route (the "decision").
    """

    detect: float | None = None       # first [perception] line — camera saw it
    classify: float | None = None     # first [classifier] line — first ItemClassification
    commit: float | None = None       # first [controller] route line — the decision
    fire: float | None = None         # [controller] pusher_x FIRED — command issued


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
            if s.classify is None or t < s.classify:
                s.classify = t
        elif node == "controller":
            if "FIRED" in rest:
                if s.fire is None or t < s.fire:
                    s.fire = t
            elif rest[:1] in ("B", "C", "D"):  # "D — firing.." / "B — rides.." = route commit
                if s.commit is None or t < s.commit:
                    s.commit = t
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
        return f"  {label:<30s} n=0    (no data)"
    return (f"  {label:<30s} n={len(xs):<3d} "
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

    dec_to_cmd: list[float] = []     # controller-internal: reliable
    cam_to_cmd: list[float] = []     # perception->controller: seconds-scale, jitter-tolerant
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
                # Both endpoints logged by the controller -> reliable.
                if s.commit is not None and s.fire is not None:
                    dec_to_cmd.append(s.fire - s.commit)
                # Perception -> controller; seconds-scale, so the 0.2 s cross-node
                # jitter is tolerable (unlike camera->decision, which is one frame).
                if s.detect is not None and s.fire is not None:
                    cam_to_cmd.append(s.fire - s.detect)
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

    print("\n=== per-stage latency (skeleton.log) ===")
    print(_row("decision -> command", dec_to_cmd))
    print("      (controller route commit -> FIRED; one process, no cross-node jitter)")
    print(_row("camera -> command", cam_to_cmd))
    print("      (first detection -> FIRED; spans nodes, tolerates the ~0.2s jitter)")
    print("  camera -> decision (detect -> classification): ~1 camera frame (~0.05s),")
    print("      BELOW the ~0.2s cross-node log-emission jitter -> not separately")
    print("      measurable here (see docs/report/methodology_and_limitations.md).")

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
