#!/usr/bin/env python3
"""Sample CPU and RSS for Linux process trees without third-party packages.

The stream runner starts this observer with Gazebo, ROS launch and feeder root
PIDs. Descendants are discovered on every lap because ROS nodes and short-lived
CLI publishers appear after launch. Raw samples go to CSV; a small JSON summary
is durable evidence for the Stage 24 soak gate.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import threading
import time
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ProcessUsage:
    ppid: int
    rss_kib: int
    cpu_s: float


@dataclass(frozen=True)
class TreeSample:
    elapsed_s: float
    processes: dict[int, ProcessUsage]


def _read_usage(pid_dir: Path, clock_ticks: int) -> ProcessUsage | None:
    """Read one consistent-enough /proc entry; vanished processes are normal."""
    try:
        stat = (pid_dir / "stat").read_text(encoding="utf-8")
        status = (pid_dir / "status").read_text(encoding="utf-8")
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return None

    closing_paren = stat.rfind(")")
    if closing_paren < 0:
        return None
    # After comm's closing parenthesis fields start at #3 (state). ppid is #4,
    # utime/stime are #14/#15. Splitting from the right handles spaces in comm.
    fields = stat[closing_paren + 2 :].split()
    try:
        ppid = int(fields[1])
        cpu_s = (int(fields[11]) + int(fields[12])) / clock_ticks
    except (IndexError, ValueError):
        return None

    rss_kib = 0
    for line in status.splitlines():
        if line.startswith("VmRSS:"):
            try:
                rss_kib = int(line.split()[1])
            except (IndexError, ValueError):
                return None
            break
    return ProcessUsage(ppid=ppid, rss_kib=rss_kib, cpu_s=cpu_s)


def collect_process_trees(
    root_pids: set[int],
    proc_root: Path = Path("/proc"),
    clock_ticks: int | None = None,
) -> dict[int, ProcessUsage]:
    """Return the live roots and all of their current descendants."""
    if clock_ticks is None:
        try:
            clock_ticks = int(os.sysconf("SC_CLK_TCK"))
        except AttributeError as exc:
            raise RuntimeError("Linux os.sysconf is unavailable") from exc
    all_processes = {}
    try:
        entries = list(proc_root.iterdir())
    except (FileNotFoundError, PermissionError) as exc:
        raise RuntimeError(f"Linux procfs is unavailable at {proc_root}") from exc

    for entry in entries:
        if not entry.name.isdigit():
            continue
        usage = _read_usage(entry, clock_ticks)
        if usage is not None:
            all_processes[int(entry.name)] = usage

    children: dict[int, list[int]] = {}
    for pid, usage in all_processes.items():
        children.setdefault(usage.ppid, []).append(pid)

    selected = set()
    pending = [pid for pid in root_pids if pid in all_processes]
    while pending:
        pid = pending.pop()
        if pid in selected:
            continue
        selected.add(pid)
        pending.extend(children.get(pid, ()))
    return {pid: all_processes[pid] for pid in selected}


def summarize(samples: list[TreeSample]) -> dict[str, float | int]:
    """Compute stable aggregate metrics from raw process-tree snapshots."""
    if not samples:
        return {
            "samples": 0,
            "elapsed_s": 0.0,
            "peak_processes": 0,
            "peak_rss_mib": 0.0,
            "average_cpu_cores": 0.0,
            "peak_cpu_cores": 0.0,
        }

    observed_cpu_s = 0.0
    peak_cpu_cores = 0.0
    previous = samples[0]
    for sample in samples[1:]:
        cpu_delta = 0.0
        for pid, usage in sample.processes.items():
            before = previous.processes.get(pid)
            cpu_delta += max(usage.cpu_s - (before.cpu_s if before else 0.0), 0.0)
        interval_s = sample.elapsed_s - previous.elapsed_s
        if interval_s > 0:
            peak_cpu_cores = max(peak_cpu_cores, cpu_delta / interval_s)
        observed_cpu_s += cpu_delta
        previous = sample

    elapsed_s = max(samples[-1].elapsed_s - samples[0].elapsed_s, 0.0)
    peak_rss_kib = max(sum(item.rss_kib for item in sample.processes.values()) for sample in samples)
    return {
        "samples": len(samples),
        "elapsed_s": round(elapsed_s, 3),
        "peak_processes": max(len(sample.processes) for sample in samples),
        "peak_rss_mib": round(peak_rss_kib / 1024.0, 3),
        "average_cpu_cores": round(observed_cpu_s / elapsed_s, 3) if elapsed_s else 0.0,
        "peak_cpu_cores": round(peak_cpu_cores, 3),
    }


def sample_until_stopped(
    root_pids: set[int],
    csv_path: Path,
    json_path: Path,
    interval_s: float,
    proc_root: Path = Path("/proc"),
) -> dict[str, float | int]:
    stop = threading.Event()

    def request_stop(_signum, _frame):
        stop.set()

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    samples = []
    started = time.monotonic()

    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("elapsed_s", "processes", "rss_kib", "total_cpu_s"))
        while True:
            processes = collect_process_trees(root_pids, proc_root)
            sample = TreeSample(time.monotonic() - started, processes)
            samples.append(sample)
            writer.writerow(
                (
                    f"{sample.elapsed_s:.3f}",
                    len(processes),
                    sum(item.rss_kib for item in processes.values()),
                    f"{sum(item.cpu_s for item in processes.values()):.3f}",
                )
            )
            stream.flush()
            if not processes or stop.wait(interval_s):
                break

    summary = summarize(samples)
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pid", type=int, action="append", required=True, dest="pids")
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--json", type=Path, required=True)
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()
    if any(pid <= 0 for pid in args.pids):
        parser.error("--pid values must be positive")
    if args.interval <= 0:
        parser.error("--interval must be positive")

    summary = sample_until_stopped(set(args.pids), args.csv, args.json, args.interval)
    print(json.dumps(summary, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
