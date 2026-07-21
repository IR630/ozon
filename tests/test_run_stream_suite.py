"""Dry-run and termination-ledger tests for the all-model stream suite."""
import os
import subprocess
from pathlib import Path

import pytest

from bash_host import find_bash

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "run_stream_suite.sh"
SLUGS = {
    "bottle",
    "box_300x200x200",
    "box_400x400x300",
    "lunchbox",
    "bag",
    "detergent",
    "pouf",
    "pen",
    "plate",
    "cylinder",
    "helmet",
}


def _bash_env():
    bash = find_bash()
    if bash is None:
        pytest.skip("bash is unavailable on this host")
    return bash, os.environ.copy()


def test_dry_run_covers_every_model_once_per_orientation():
    bash, env = _bash_env()
    env["STREAM_SUITE_DRY_RUN"] = "1"
    result = subprocess.run(
        [bash, str(SCRIPT), "7", "0", "2"],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    plans = [line for line in result.stdout.splitlines() if line.startswith("[plan ")]
    assert len(plans) == 6
    for oi in range(3):
        selected = [line for line in plans if f"oi={oi} " in line]
        found = {token.split(":", 1)[0] for line in selected for token in line.split() if ":" in token}
        assert found == SLUGS
    assert "6 episodes, 33 planned items" in result.stdout


def test_same_zone_gap_override_keeps_cross_zone_pair_and_slow_item_guards():
    bash, env = _bash_env()
    env["STREAM_SUITE_DRY_RUN"] = "1"
    env["SAME_ZONE_GAP_M"] = "0.8"
    result = subprocess.run(
        [bash, str(SCRIPT), "0", "0", "0"],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "plate:D:0.8" in result.stdout
    assert "pouf:C:1.5" in result.stdout
    assert result.stdout.count(":B:0.8") == 5
    assert "box_400x400x300:C:3.1" in result.stdout
    assert "pen:C:5.0" in result.stdout


def test_reversed_orientation_range_is_rejected():
    bash, env = _bash_env()
    env["STREAM_SUITE_DRY_RUN"] = "1"
    result = subprocess.run(
        [bash, str(SCRIPT), "0", "2", "1"],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=False,
    )

    assert result.returncode == 2
    assert "orientation range is reversed" in result.stderr


def test_each_runner_failure_gets_a_durable_status(tmp_path):
    bash, env = _bash_env()
    stub = tmp_path / "fail.sh"
    stub.write_text("#!/usr/bin/env bash\nexit 7\n", encoding="utf-8")
    env["STREAM_RUNNER"] = f"bash {stub.as_posix()}"
    env["LOGDIR"] = tmp_path.as_posix()
    result = subprocess.run(
        [bash, str(SCRIPT), "0", "0", "0"],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=False,
    )

    assert result.returncode != 0
    assert "episodes 0/2; failed 2" in result.stdout
    assert (tmp_path / "oi0_mixed_cd.status").read_text(encoding="utf-8") == "rc=7\n"
    assert (tmp_path / "oi0_pass_b.status").read_text(encoding="utf-8") == "rc=7\n"
