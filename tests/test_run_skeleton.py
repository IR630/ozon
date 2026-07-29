"""Static regression checks for the Gazebo episode lifecycle."""

import os
import subprocess
from pathlib import Path

import pytest
from bash_host import find_bash

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "run_skeleton.sh"


def test_errexit_is_armed_before_anything_that_can_fail():
    """`set -e` used to sit AFTER the two `source` lines, so the run continued unsourced."""
    body = [line for line in SCRIPT.read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.startswith("#")]

    assert body[0] == "set -e", f"first executable line must arm errexit, got {body[:3]}"


def test_a_clean_checkout_is_told_to_build_the_ros_workspace(tmp_path):
    """install/ is a gitignored build artifact, so a fresh checkout has no setup.bash.

    Sourcing it was attempted BEFORE `set -e` was armed, so the failure did not stop
    the run: it carried on unsourced and died 200 lines later claiming "the belt never
    reached full speed" — sending a new participant to debug conveyor physics instead
    of running one colcon command. Name the real cause, and stop.
    """
    bash = find_bash()
    if bash is None:
        pytest.skip("bash is unavailable on this host")
    env = os.environ.copy()
    env["ROS_INSTALL_ROOT"] = str(tmp_path / "not-built")

    result = subprocess.run(
        [bash, str(SCRIPT), "box_300x200x200", "B"],
        cwd=SCRIPT.parents[1], env=env, capture_output=True, text=True,
        check=False, timeout=60,
    )

    assert result.returncode != 0, result.stdout
    assert "colcon build --packages-select ros_msgs" in result.stderr
    assert "belt never reached full speed" not in result.stderr, "false diagnosis is back"


def test_episode_cleanup_runs_on_early_exit_and_reaps_stale_gazebo():
    text = SCRIPT.read_text(encoding="utf-8")

    assert "trap cleanup EXIT" in text
    assert "GAZEBO_PATTERN='^ign gazebo( |$)'" in text
    assert 'pkill -TERM -f "$GAZEBO_PATTERN"' in text
    assert 'pkill -KILL -f "$GAZEBO_PATTERN"' in text
    assert text.index("cleanup\n") < text.index('ign gazebo -s -r -v 0 "$WORLD"')


def test_dynamics_trace_path_is_configurable_for_matrix_triage():
    text = SCRIPT.read_text(encoding="utf-8")

    assert "DYNAMICS_TRACE=${DYNAMICS_TRACE:-/tmp/dyn_trace.log}" in text
    assert '> "$DYNAMICS_TRACE" 2>&1 &' in text
    assert 'capture_dynamics.py "$DYNAMICS_TRACE"' in text


def test_the_item_is_fed_onto_a_belt_already_at_speed():
    """Never start the belt UNDER the goods — an infeed places them on a running belt.

    The old order (spawn, then let the soft-start ramp drag the item up to 1 m/s) is what
    made the helmet look unstable: it stands on a 62x64 mm crown of its hull with its
    centre of mass 175 mm up, so it tips at 10 degrees. Started underneath, it TUMBLED the
    whole way down the belt (its axis swinging 0 -> 67 -> 125 degrees) and its lateral
    position random-walked off the 0.5 m belt in 4 runs out of 19 — the census's last
    failure. Fed onto a moving belt it rides dead centre: y = 0.0109 m, eight runs, five
    decimal places apart. run_stream.sh always fed this way, which is why the stream never
    saw it. Pin the ORDER: the spawn must come after the soft-start is confirmed done.
    """
    text = SCRIPT.read_text(encoding="utf-8")

    wait_done = text.index('grep -q "soft-start done"')
    spawn = text.index("ign service -s /world/cell/create")
    assert wait_done < spawn, "the item must be spawned onto a belt already at full speed"


def test_every_spawn_is_announced_to_the_feed_watchdog():
    """A jam BEFORE the camera is invisible to the in-window jam detector, so the
    runner must announce each feed on /infeed/fed right after the spawn — the
    controller's watchdog then latches the cell if the camera never confirms it."""
    text = SCRIPT.read_text(encoding="utf-8")

    spawn = text.index("ign service -s /world/cell/create")
    announce = text.index("ros2 topic pub -w 1 --once /infeed/fed std_msgs/msg/Empty")
    assert spawn < announce, "the feed must be announced AFTER the spawn, never before"


def test_soft_start_wait_is_overridable_and_defaults_to_the_shipped_value():
    """The bring-up wait is a HARNESS constant that has twice produced a false
    diagnosis, so it must be tunable without changing the shipped behaviour.

    First time it claimed "the belt never reached full speed" on an unsourced
    workspace (see the comment it carries). Second time it cost this branch a
    wrong verdict: the three-head rig runs a ~74 s median cycle against this
    world's ~23 s, overran the fixed 30 s in 3 of 33 census cells, and the
    failure was attributed to bridge load rather than to the timer. With the
    wait raised the same cells give 33/33.
    """
    text = SCRIPT.read_text(encoding="utf-8")

    assert 'seq 1 "${SOFT_START_TRIES:-60}"' in text, "wait must read SOFT_START_TRIES"
    # 60 tries x 0.5 s = the 30 s the shipped census has always used
    assert "sleep 0.5" in text
    assert "soft-start done" in text


def test_the_cycle_number_is_broken_down_into_attributable_stages():
    """One "cycle 94.3s" cannot tell a slow pipeline from a slow harness.

    It is measured from `ros2 launch`, so it carries the whole ROS stack coming
    up as well as the parcel's ride. A shipped line starts that stack once per
    shift and not once per parcel, so a rig comparison quoting the total alone
    charges the three-head rig for processes rather than for perception. The
    stamps must bracket bring-up, feed and transit separately.
    """
    text = SCRIPT.read_text(encoding="utf-8")

    for stamp in ("T0=$(date +%s.%N)", "T_UP=$(date +%s.%N)", "T_FED=$(date +%s.%N)",
                  "T1=$(date +%s.%N)"):
        assert stamp in text, f"missing stage stamp {stamp}"
    assert text.index("T0=$(date") < text.index("T_UP=$(date") < text.index("T_FED=$(date")
    assert "stages: bringup" in text


def test_the_episode_does_not_block_on_a_simulator_query_it_cannot_afford():
    """A stats probe was added here and removed the same evening.

    It returned nothing in six episodes and still burned its full 5 s timeout in
    each, and `ci_e2e.sh` gives the WHOLE contour smoke 120 s — so a diagnostic
    that yields no number may not sit in the episode path. Render load is
    separated from pipeline latency by `cam->decision` instead.
    """
    text = SCRIPT.read_text(encoding="utf-8")

    assert "ign topic -e -t /world/cell/stats" not in text
    assert "real_time_factor" not in text


def test_the_measurement_line_reaches_the_cell_log():
    """The dimensions + heads=N line must be printed EXPLICITLY, not via the tail.

    The episode ends with `grep "item N:" | tail -3`, and the classifier and
    controller lines match that pattern too. Being logged later they always take
    the last three slots, so the perception line — the only one carrying the
    measured dimensions and heads=N — never reached the cell log. Both failures
    are silent: census_tolerance.py scores nothing, and a rig that quietly lost a
    side head looks exactly like one that fused correctly.
    """
    text = SCRIPT.read_text(encoding="utf-8")
    assert r'grep -E "\[perception\]: item [0-9]+:" "$NODE_LOG" | tail -1' in text, (
        "the perception measurement line must be greped for on its own")


def test_the_renderer_is_a_seam_and_still_defaults_to_software():
    """Software GL everywhere is the portable default; a GPU box must be able to opt out.

    The default must NOT change: every stand this project runs on (WSL, CI, the
    organizers' headless check) has no usable GPU, and a bare `=0` would break them.
    """
    text = SCRIPT.read_text(encoding="utf-8")
    assert "export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE:-1}" in text, (
        "the renderer must stay overridable AND default to software")
