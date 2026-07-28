"""Static/dry-run checks for the resumable day-4 matrix harness."""
import os
import subprocess
import sys
from pathlib import Path

import pytest
from bash_host import find_bash

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "run_matrix.sh"


def _bash_env():
    bash = find_bash()
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
        encoding="utf-8",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    plans = [line for line in result.stdout.splitlines() if line.startswith("[plan ")]
    assert len(plans) == 8  # four requested items x two orientations
    assert plans[0].startswith("[plan pen oi=0 -> C]")
    assert plans[-1].startswith("[plan helmet oi=1 -> B]")
    assert "items=7..10: 8 cells" in result.stdout
    assert "bottle" not in result.stdout


def test_the_census_measures_the_final_mechanism_and_says_which():
    """The census IS the milestone metric, so it must not be silent about what it measured.

    run_skeleton.sh defaults to the ballistic pusher (cell.sdf) — the day-3 baseline that
    CI drives. The matrix inherited that default, so three full censuses were run without
    WORLD set, silently measured the ABANDONED mechanism and scored 30-31/33 for it. The
    pusher's belt carries only 6.4 m of stroke against the diverter's 20 m, so it dies
    mid-episode and B items simply stop on the belt — which the old x>=3.5 band scored as
    "rode past the mechanisms". Pin both halves: the diverter is the default, and the
    world is named in the summary line a report would quote.
    """
    bash, env = _bash_env()
    env.pop("WORLD", None)
    result = subprocess.run(
        [bash, str(SCRIPT), "0", "1", "0", "0"],
        cwd=ROOT, env=env, capture_output=True, text=True, encoding="utf-8", check=False,
    )

    assert result.returncode == 0, result.stderr
    # The diverter stays pinned; what moved on 28.07 is WHICH RIG is production —
    # the three-head world, by the owner's decision. Both halves of the original
    # contract still hold: the default is the shipped mechanism, and the summary
    # line a report would quote names the world it measured.
    assert "world: sim/worlds/cell_diverter_3cam.sdf" in result.stdout
    assert "world=sim/worlds/cell_diverter_3cam.sdf" in result.stdout


def test_the_baseline_mechanism_can_still_be_measured_on_purpose():
    """compare_mechanisms.sh measures the pusher deliberately — an explicit WORLD wins."""
    bash, env = _bash_env()
    env["WORLD"] = "sim/worlds/cell.sdf"
    result = subprocess.run(
        [bash, str(SCRIPT), "0", "1", "0", "0"],
        cwd=ROOT, env=env, capture_output=True, text=True, encoding="utf-8", check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "world: sim/worlds/cell.sdf" in result.stdout


def test_dry_run_reports_the_selected_world():
    """An explicitly selected world is echoed too (IMaSKoT, 6692688)."""
    bash, env = _bash_env()
    env["WORLD"] = "sim/worlds/cell_diverter.sdf"
    result = subprocess.run(
        [bash, str(SCRIPT), "0", "1", "0", "0"],
        cwd=ROOT, env=env, capture_output=True, text=True, encoding="utf-8", check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "world: sim/worlds/cell_diverter.sdf" in result.stdout
    assert "matrix dry-run world=sim/worlds/cell_diverter.sdf" in result.stdout


def test_cell_timeout_bounds_a_wedged_episode(tmp_path):
    # A wedged Gazebo episode makes run_skeleton.sh hang forever; run_matrix must
    # cap each cell (timeout), record it TIMEOUT, and move on instead of hanging.
    bash, env = _bash_env()
    probe = subprocess.run(
        [bash, "-lc", "timeout --version"],
        capture_output=True, text=True, encoding="utf-8", check=False,
    )
    if "coreutils" not in (probe.stdout + probe.stderr).lower():
        pytest.skip("GNU coreutils timeout unavailable")

    env.pop("MATRIX_DRY_RUN", None)  # actually execute the (stub) cell
    env["CELL_TIMEOUT"] = "1"
    env["LOGDIR"] = tmp_path.as_posix()  # keep episode logs out of the repo's runs/
    stub = tmp_path / "hang.sh"  # runner that outlives the 1 s cap
    stub.write_text("#!/usr/bin/env bash\nsleep 8\n")
    env["SKELETON"] = f"bash {stub.as_posix()}"

    result = subprocess.run(
        [bash, str(SCRIPT), "0", "1", "0", "0"],  # one cell: bottle, oi=0
        cwd=ROOT, env=env, capture_output=True, text=True, encoding="utf-8",
        check=False, timeout=25,
    )

    assert "TIMEOUT" in result.stdout, result.stdout + result.stderr
    assert "routing correctness 0/1" in result.stdout
    assert result.returncode != 0  # a timed-out cell is not a pass
    assert (tmp_path / "matrix_bottle_0.status").read_text(encoding="utf-8") == "rc=124\n"


def test_runner_error_is_saved_in_a_durable_cell_status(tmp_path):
    bash, env = _bash_env()
    env.pop("MATRIX_DRY_RUN", None)
    env["LOGDIR"] = tmp_path.as_posix()
    stub = tmp_path / "runner_error.sh"
    stub.write_text("#!/usr/bin/env bash\nexit 7\n", encoding="utf-8")
    env["SKELETON"] = f"bash {stub.as_posix()}"

    result = subprocess.run(
        [bash, str(SCRIPT), "0", "1", "0", "0"],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=False,
    )

    assert result.returncode != 0
    assert "RUNNER_ERROR" in result.stdout
    assert (tmp_path / "matrix_bottle_0.status").read_text(encoding="utf-8") == "rc=7\n"


def test_targeted_replay_can_keep_a_pose_trace_per_cell(tmp_path):
    """A rare physics failure must leave its trajectory behind for triage."""
    bash, env = _bash_env()
    env.pop("MATRIX_DRY_RUN", None)
    env["LOGDIR"] = tmp_path.as_posix()
    env["SAVE_DYNAMICS"] = "1"
    stub = tmp_path / "trace.sh"
    stub.write_text(
        '#!/usr/bin/env bash\nprintf "capture=%s\\n" "$CAPTURE_DYNAMICS" '
        '> "$DYNAMICS_TRACE"\nprintf "%s -> %s: PASS (pose x=0 y=0 z=0, '
        'cycle 1.0s from launch)\\n" "$1" "$2"\n',
        encoding="utf-8",
    )
    env["SKELETON"] = f"bash {stub.as_posix()}"

    result = subprocess.run(
        [bash, str(SCRIPT), "0", "1", "10", "10"],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=False,
    )

    trace = tmp_path / "dynamic_pose_helmet_0.log"
    assert result.returncode == 0, result.stdout + result.stderr
    assert trace.read_text(encoding="utf-8") == "capture=1\n"
    assert not list(tmp_path.glob("matrix_*_dynamic_pose.log"))


def test_dry_run_rejects_invalid_item_range():
    bash, env = _bash_env()
    result = subprocess.run(
        [bash, str(SCRIPT), "0", "3", "10", "7"],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        encoding="utf-8",
        check=False,
    )

    assert result.returncode == 2
    assert "item range must fit 0..10" in result.stderr
