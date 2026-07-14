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
retracts, so at 1 m/s the FEED floor for a zone-changing pair is that many
seconds; same-zone pairs ride nose to tail (min gap 0 m). The accepted feed plan
is saved in plan.log and checked against that floor. Arrival takt is deliberately
NOT compared with the feed floor: C, D and B have different transit/settling paths,
so two correctly spaced feeds may arrive less (or more) than 3.1 s apart.

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
# run_stream.sh's result line. PASS carries a landing time; FAIL deliberately
# does not. Throughput uses only PASS arrivals, while week-3 long-horizon
# reliability must retain BOTH outcomes instead of silently dropping failures.
RESULT_LINE = re.compile(
    r"^(?P<name>item\d+)\s+(?P<slug>\S+)\s+->\s+(?P<zone>[BCD]):\s+"
    r"(?P<outcome>PASS|FAIL)(?: at t=(?P<t>[\d.]+)s)?"
)
PLAN_LINE = re.compile(
    r"^(?P<index>\d+)\s+(?P<slug>\S+)\s+(?P<zone>[BCD])\s+"
    r"(?P<spawn_x>-?[\d.]+)\s+(?P<feed_s>[\d.]+)$"
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


@dataclass
class StreamResult:
    """Terminal routing verdict for one spawned item in one stream episode."""

    name: str
    slug: str
    zone: str
    passed: bool
    t: float | None  # T0-relative arrival; absent for FAIL


@dataclass
class ReliabilitySummary:
    episodes: int
    all_pass_episodes: int
    items: int
    passed_items: int
    # (slug, expected zone) -> (passed, total). Keeping the expected zone in the
    # key prevents two different routes of the same diagnostic model being mixed.
    by_route: dict[tuple[str, str], tuple[int, int]]


@dataclass
class PlannedItem:
    index: int
    slug: str
    zone: str
    feed_s: float


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


def parse_stream_results(path: Path) -> list[StreamResult]:
    """All body-scored PASS/FAIL verdicts from one saved stream summary."""
    out: list[StreamResult] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        m = RESULT_LINE.match(line.strip())
        if m:
            passed = m["outcome"] == "PASS"
            t = float(m["t"]) if m["t"] is not None else None
            # A malformed "PASS" without an arrival cannot support either a
            # throughput number or a trustworthy terminal result.
            if passed and t is None:
                continue
            out.append(StreamResult(m["name"], m["slug"], m["zone"], passed, t))
    return out


def parse_stream(path: Path) -> list[Arrival]:
    """Successful body-scored arrivals, sorted by time for throughput analysis."""
    out = [Arrival(r.name, r.slug, r.zone, r.t)
           for r in parse_stream_results(path) if r.passed and r.t is not None]
    return sorted(out, key=lambda a: a.t)


def parse_plan(path: Path) -> list[PlannedItem]:
    """Accepted feed schedule printed by stream_plan.py, in feed order.

    The same lines may live in the new plan.log or an older captured console.log.
    """
    items = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        m = PLAN_LINE.match(line.strip())
        if m:
            items.append(PlannedItem(
                int(m["index"]), m["slug"], m["zone"], float(m["feed_s"])))
    return sorted(items, key=lambda item: item.index)


def summarize_reliability(episodes: list[list[StreamResult]]) -> ReliabilitySummary:
    """Aggregate exact routing counts without inventing confidence from a small N."""
    nonempty = [results for results in episodes if results]
    route_counts: dict[tuple[str, str], list[int]] = {}
    items = passed_items = 0
    for results in nonempty:
        for result in results:
            items += 1
            passed_items += int(result.passed)
            counts = route_counts.setdefault((result.slug, result.zone), [0, 0])
            counts[0] += int(result.passed)
            counts[1] += 1
    return ReliabilitySummary(
        episodes=len(nonempty),
        all_pass_episodes=sum(all(r.passed for r in results) for results in nonempty),
        items=items,
        passed_items=passed_items,
        by_route={key: (counts[0], counts[1]) for key, counts in route_counts.items()},
    )


@dataclass
class TaktGap:
    gap_s: float       # observed time between two successive arrivals
    front_zone: str
    back_zone: str
    feed_floor_s: float  # tags a zone change; never compare with arrival gap_s


def takt_floor_s(front_zone: str, back_zone: str,
                 belt_speed_m_s: float = BELT_SPEED_M_S) -> float:
    """Convert the stream plan's minimum separation in metres to a takt in seconds."""
    floor_m = stream_plan.min_gap_between_zones_m(
        front_zone, back_zone, belt_speed_m_s=belt_speed_m_s)
    return floor_m / belt_speed_m_s


def takt_gaps(arrivals: list[Arrival]) -> list[TaktGap]:
    """Successive-arrival gaps within one run, each tagged with its computed floor.

    The belt is FIFO, so arrival order is feed order: the front item of a pair is the
    one that landed first. feed_floor_s is retained only to split same-route and
    zone-change arrival statistics. It belongs to the FEED schedule, not arrival time.
    """
    gaps = []
    for front, back in zip(arrivals, arrivals[1:]):
        gaps.append(TaktGap(back.t - front.t, front.zone, back.zone,
                            takt_floor_s(front.zone, back.zone)))
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


def _ratio(passed: int, total: int) -> str:
    if not total:
        return "0/0 (no data)"
    return f"{passed}/{total} ({100.0 * passed / total:.1f}%)"


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
    reliability_episodes: list[list[StreamResult]] = []
    planned_change_gaps: list[float] = []
    planned_change_margins: list[float] = []

    print(f"throughput over {len(runs)} run(s):\n")
    for run in runs:
        skel = run / "skeleton.log"
        strm = run / "stream.log"
        # plan.log is produced by current run_stream. console.log is a backward-
        # compatible source for older episodes whose stdout happened to be saved.
        plan_path = run / "plan.log"
        if not plan_path.exists() and (run / "console.log").exists():
            plan_path = run / "console.log"
        if plan_path.exists():
            plan = parse_plan(plan_path)
            for front, back in zip(plan, plan[1:]):
                floor = takt_floor_s(front.zone, back.zone)
                if floor > 0:
                    actual = back.feed_s - front.feed_s
                    planned_change_gaps.append(actual)
                    planned_change_margins.append(actual - floor)
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
            results = parse_stream_results(strm)
            reliability_episodes.append(results)
            arrivals = [Arrival(r.name, r.slug, r.zone, r.t)
                        for r in results if r.passed and r.t is not None]
            arrivals.sort(key=lambda a: a.t)
            gaps = takt_gaps(arrivals)
            if gaps:
                n_runs_timed += 1
                run_takt = median([g.gap_s for g in gaps])
                print(f"  {run.name}: {len(arrivals)} arrivals, "
                      f"takt {run_takt:.2f}s ({60.0 / run_takt:.0f}/min)")
            for g in gaps:
                all_takts.append(g.gap_s)
                (change_takts if g.feed_floor_s > 0 else noair_takts).append(g.gap_s)
        else:
            print(f"  {run.name}: no stream.log (skeleton latencies only)")

    reliability = summarize_reliability(reliability_episodes)
    print("\n=== routing reliability (stream.log; PASS and FAIL) ===")
    print(f"  all-pass episodes: {_ratio(reliability.all_pass_episodes, reliability.episodes)}")
    print(f"  routed items:      {_ratio(reliability.passed_items, reliability.items)}")
    for (slug, zone), (passed, total) in sorted(reliability.by_route.items()):
        print(f"  {slug:24s} -> {zone}: {_ratio(passed, total)}")

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

    # Calc vs accepted input schedule. Arrival takt remains a throughput observation:
    # destination-specific transit and settling make it incomparable with a feed floor.
    change_floor_m = stream_plan.min_gap_between_zones_m("C", "D")
    change_floor = takt_floor_s("C", "D")
    transit = (stream_plan.TARGET_X_M - stream_plan.FIRST_SPAWN_X_M) / BELT_SPEED_M_S
    print("\n=== computed feed floor (stream_plan.py geometry) vs accepted plan ===")
    print(f"  zone-change MIN FEED gap (anti-cross-fire): {change_floor_m:.2f} m "
          f"/ {BELT_SPEED_M_S:.1f} m/s = {change_floor:.2f}s "
          f"(HOLD {stream_plan.HOLD_S:.1f} + RETRACT {stream_plan.RETRACT_S:.1f})")
    if planned_change_gaps:
        min_gap = min(planned_change_gaps)
        min_margin = min(planned_change_margins)
        status = "PASS" if min_margin >= -1e-9 else "VIOLATION"
        print(f"    accepted zone-change FEED gaps: n={len(planned_change_gaps)}, "
              f"min={min_gap:.2f}s, median={median(planned_change_gaps):.2f}s; "
              f"minimum margin={min_margin:+.2f}s -> {status}")
    else:
        print("    accepted feed gaps: unavailable (no saved plan.log/console.log)")
    if change_takts:
        print(f"    observed zone-change ARRIVAL takt: median {median(change_takts):.2f}s "
              f"(descriptive only; destination transit/settling differs)")
    print("  same-zone takt floor: item length / belt speed (geometry-independent)")
    print(f"  per-item belt transit (spawn x={stream_plan.FIRST_SPAWN_X_M:.1f} -> "
          f"target x={stream_plan.TARGET_X_M:.1f}): {transit:.1f}s at {BELT_SPEED_M_S:.1f} m/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
