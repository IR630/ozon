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
