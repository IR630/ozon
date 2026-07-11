"""Static/dry-run checks for the resumable day-4 matrix harness."""
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "run_matrix.sh"


def _bash_env():
    bash = shutil.which("bash")
    if bash is None and os.name == "nt":
        git_bash = Path(os.environ.get("ProgramFiles", r"C:\Program Files")) / "Git" / "bin" / "bash.exe"
        if git_bash.exists():
            bash = str(git_bash)
    if bash is None:
        pytest.skip("bash is unavailable on this host")
    env = os.environ.copy()
    env["MATRIX_DRY_RUN"] = "1"
    env["PYTHON"] = Path(sys.executable).as_posix()
    return bash, env


def test_dry_run_selects_only_requested_tail():
    bash, env = _bash_env()
    result = subprocess.run(
        [bash, str(SCRIPT), "7", "2", "7", "10"],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    plans = [line for line in result.stdout.splitlines() if line.startswith("[plan ")]
    assert len(plans) == 8  # four requested items x two orientations
    assert plans[0].startswith("[plan pen oi=0 -> C]")
    assert plans[-1].startswith("[plan helmet oi=1 -> B]")
    assert "items=7..10: 8 cells" in result.stdout
    assert "bottle" not in result.stdout


def test_cell_timeout_bounds_a_wedged_episode(tmp_path):
    # A wedged Gazebo episode makes run_skeleton.sh hang forever; run_matrix must
    # cap each cell (timeout), record it TIMEOUT, and move on instead of hanging.
    bash, env = _bash_env()
    probe = subprocess.run(
        [bash, "-lc", "timeout --version"],
        capture_output=True, text=True, check=False,
    )
    if "coreutils" not in (probe.stdout + probe.stderr).lower():
        pytest.skip("GNU coreutils timeout unavailable")

    env.pop("MATRIX_DRY_RUN", None)  # actually execute the (stub) cell
    env["CELL_TIMEOUT"] = "1"
    stub = tmp_path / "hang.sh"  # runner that outlives the 1 s cap
    stub.write_text("#!/usr/bin/env bash\nsleep 8\n")
    env["SKELETON"] = f"bash {stub.as_posix()}"

    result = subprocess.run(
        [bash, str(SCRIPT), "0", "1", "0", "0"],  # one cell: bottle, oi=0
        cwd=ROOT, env=env, capture_output=True, text=True,
        check=False, timeout=25,
    )

    assert "TIMEOUT" in result.stdout, result.stdout + result.stderr
    assert "routing correctness 0/1" in result.stdout
    assert result.returncode != 0  # a timed-out cell is not a pass


def test_dry_run_rejects_invalid_item_range():
    bash, env = _bash_env()
    result = subprocess.run(
        [bash, str(SCRIPT), "0", "3", "10", "7"],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 2
    assert "item range must fit 0..10" in result.stderr
