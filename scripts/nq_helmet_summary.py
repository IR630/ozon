#!/usr/bin/env python3
"""Aggregate the per-frame K distribution captured by nq_helmet_repro.sh.

The two multi-seed misses both printed an aggregated k=0.800 and routed D, but no
log we owned kept the per-FRAME K series behind that median (docs/decisions.md
23.07). This reads the preserved per-cell node logs and answers, per repeat and
per cell, the one question the aggregate hid: what does the K *distribution* look
like, and does its median actually sit above the 0.8 threshold the rule uses?

    python3 scripts/nq_helmet_summary.py runs/nq_helmet_repro

Pure parsing over the exact log format strings emitted by src/perception_node.py
(per-frame `... mm K=0.XXXX ...`) and src/classification via src/classifier_node.py
(aggregate `item N: <B|C|D> (k=0.XXXXXX, ...)`). No ROS, no Gazebo.
"""
from __future__ import annotations

import glob
import os
import re
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # src/ (cwd is not on path)

from src.constants import ROUND_K_THRESHOLD  # noqa: E402

# Per-frame perception line: "item 1: 324x298x284 mm K=0.8000 at (0.01, 0.02) heads=1"
FRAME_K = re.compile(r"item\s+\d+:\s+\d+x\d+x\d+\s+mm\s+K=([0-9.]+)")
# Aggregated classifier verdict: "item 1: D (k=0.800000, conf=0.83, n=16)"
VERDICT = re.compile(r"item\s+\d+:\s+([BCD])\s+\(k=([0-9.]+),")


@dataclass
class Repeat:
    path: str
    frame_ks: list[float] = field(default_factory=list)
    verdict_zone: str | None = None
    verdict_k: float | None = None


def parse_node_log(path: str) -> Repeat:
    rep = Repeat(path=path)
    with open(path, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            m = FRAME_K.search(line)
            if m:
                rep.frame_ks.append(float(m.group(1)))
                continue
            v = VERDICT.search(line)
            if v:
                rep.verdict_zone = v.group(1)
                rep.verdict_k = float(v.group(2))
    return rep


def _fmt_ks(ks: list[float]) -> str:
    if not ks:
        return "no frames"
    return (f"n={len(ks)} min={min(ks):.4f} med={statistics.median(ks):.4f} "
            f"max={max(ks):.4f}")


def summarize(logdir: str) -> int:
    paths = sorted(glob.glob(os.path.join(logdir, "*.node.log")))
    if not paths:
        print(f"no *.node.log under {logdir}", file=sys.stderr)
        return 1
    by_cell: dict[str, list[Repeat]] = {}
    for path in paths:
        rep = parse_node_log(path)
        # cell key = filename up to "_rep": "helmet_seed1_oi1"
        base = os.path.basename(path)
        cell = re.sub(r"_rep\d+\.node\.log$", "", base)
        by_cell.setdefault(cell, []).append(rep)

    thr = ROUND_K_THRESHOLD
    for cell, reps in by_cell.items():
        zones = [r.verdict_zone for r in reps]
        n_d = zones.count("D")
        print(f"\n=== {cell}: {len(reps)} repeats, verdict D in {n_d}/{len(reps)} "
              f"(threshold k>{thr}) ===")
        for r in reps:
            med = statistics.median(r.frame_ks) if r.frame_ks else float("nan")
            margin = med - thr
            print(f"  {os.path.basename(r.path):38s} "
                  f"verdict={r.verdict_zone} k_agg={r.verdict_k} "
                  f"frame_K[{_fmt_ks(r.frame_ks)}] median_margin={margin:+.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(summarize(sys.argv[1] if len(sys.argv) > 1 else "runs/nq_helmet_repro"))
