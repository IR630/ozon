from pathlib import Path

import pytest

from sample_process_resources import ProcessUsage, TreeSample, collect_process_trees, summarize


def _write_process(proc_root: Path, pid: int, ppid: int, rss_kib: int, utime=10, stime=5):
    process = proc_root / str(pid)
    process.mkdir()
    # fields after comm start at #3; indices 11/12 are utime/stime (#14/#15).
    tail = ["S", str(ppid), "0", "0", "0", "0", "0", "0", "0", "0", "0", str(utime), str(stime)]
    (process / "stat").write_text(f"{pid} (worker with space) {' '.join(tail)}\n", encoding="utf-8")
    (process / "status").write_text(f"Name:\tworker\nVmRSS:\t{rss_kib} kB\n", encoding="utf-8")


def test_collect_process_trees_includes_descendants_but_not_unrelated_processes(tmp_path):
    _write_process(tmp_path, 10, 1, 100)
    _write_process(tmp_path, 11, 10, 200)
    _write_process(tmp_path, 12, 11, 300)
    _write_process(tmp_path, 20, 1, 900)

    found = collect_process_trees({10}, tmp_path, clock_ticks=100)

    assert set(found) == {10, 11, 12}
    assert sum(usage.rss_kib for usage in found.values()) == 600


def test_collect_process_trees_fails_loudly_without_procfs(tmp_path):
    with pytest.raises(RuntimeError, match="procfs is unavailable"):
        collect_process_trees({10}, tmp_path / "missing", clock_ticks=100)


def test_summary_reports_peak_rss_processes_and_cpu_cores():
    samples = [
        TreeSample(0.0, {1: ProcessUsage(0, 1024, 1.0)}),
        TreeSample(
            2.0,
            {
                1: ProcessUsage(0, 2048, 3.0),
                2: ProcessUsage(1, 1024, 0.5),
            },
        ),
        TreeSample(
            4.0,
            {
                1: ProcessUsage(0, 3072, 5.0),
                2: ProcessUsage(1, 2048, 1.5),
            },
        ),
    ]

    result = summarize(samples)

    assert result == {
        "samples": 3,
        "elapsed_s": 4.0,
        "peak_processes": 2,
        "peak_rss_mib": 5.0,
        "average_cpu_cores": 1.375,
        "peak_cpu_cores": 1.5,
    }
