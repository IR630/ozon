"""Static regression checks for the multi-item stream runner."""

import subprocess
import sys
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "run_stream.sh"


def _run_inline_throughput(arrivals):
    """Execute the exact Python heredoc embedded in the shell runner."""
    text = SCRIPT.read_text(encoding="utf-8")
    marker = '"$PYTHON" - <<PY\n'
    start = text.index(marker) + len(marker)
    end = text.index("\nPY", start)
    code = text[start:end].replace("$ARRIVALS", "\n".join(map(str, arrivals)))
    return subprocess.run(
        [sys.executable, "-c", code],
        capture_output=True,
        text=True,
        check=False,
    )


def test_the_stream_announces_every_feed_to_the_watchdog():
    """Each spawned item is announced on /infeed/fed AFTER its spawn, inside the
    feeder loop, so a stream item wedged before the camera latches the cell as a
    FEED JAM instead of silently never arriving."""
    text = SCRIPT.read_text(encoding="utf-8")

    spawn = text.index("ign service -s /world/cell/create")
    announce = text.index("ros2 topic pub -w 1 --once /infeed/fed std_msgs/msg/Empty")
    fed_echo = text.index('echo "fed item$i')
    assert spawn < announce < fed_echo, (
        "the announce must sit in the feeder loop, after the spawn call")


def test_the_accepted_feed_plan_is_saved_beside_the_terminal_result():
    """Offline reliability must not infer feed spacing from zone arrival times."""
    text = SCRIPT.read_text(encoding="utf-8")

    plan = text.index('> "$LOGDIR/plan.log"')
    launch = text.index('ros2 launch launch/skeleton.launch.py')
    result = text.index('| tee "$LOGDIR/stream.log"')
    assert plan < launch < result


def test_hulls_are_precomputed_before_the_timed_feeder_starts():
    """Mesh loading must not inflate T0-relative latency or race the first feed."""
    text = SCRIPT.read_text(encoding="utf-8")

    hull_precompute = text.index("declare -A HULL")
    timer_start = text.index("T0=$(date +%s.%N)")
    feeder_started = text.index("FEEDER=$!")

    assert hull_precompute < timer_start < feeder_started


def test_early_exit_cleans_up_the_stream_processes():
    """Any failure after launch must reap the feeder, ROS nodes and Gazebo."""
    text = SCRIPT.read_text(encoding="utf-8")

    trap = text.index("trap cleanup EXIT")
    gazebo_start = text.index('ign gazebo -s -r -v 0 "$WORLD"')
    cleanup_start = text.index("cleanup() {")
    cleanup_end = text.index("\n}", cleanup_start)
    cleanup = text[cleanup_start:cleanup_end]

    assert trap < gazebo_start
    assert 'kill "$FEEDER"' in cleanup
    assert 'kill "$LAUNCH"' in cleanup
    assert 'pkill -f "ign gazebo"' in cleanup


def test_hull_precompute_failure_aborts_instead_of_degrading_the_verdict():
    """A missing body hull invalidates rotated body scoring; it is not a pose FAIL."""
    text = SCRIPT.read_text(encoding="utf-8")

    assert "|| HULL[$i]=/dev/null" not in text
    assert "ABORT: body hull precompute failed" in text


def test_verdict_tool_error_is_reported_as_invalid_not_physical_fail():
    """An exception or malformed verdict must poison the episode explicitly."""
    text = SCRIPT.read_text(encoding="utf-8")

    assert 'if ! VERDICT_OUTPUT=$("$PYTHON" scripts/zone_verdict.py' in text
    assert 'RUN_ERROR="$NAME verdict failed:' in text
    assert 'RUN_ERROR="$NAME verdict returned invalid output:' in text
    assert ': INVALID (' in text
    assert '[ -z "$RUN_ERROR" ] || exit 2' in text


def test_documented_lowercase_negative_verdict_keeps_polling():
    """A not-yet-arrived item is normal, not a malformed runner response."""
    verdict = SCRIPT.with_name("zone_verdict.py")
    result = subprocess.run(
        [sys.executable, str(verdict), "B", "-1.5", "0", "0.4"],
        capture_output=True,
        text=True,
        check=False,
    )
    text = SCRIPT.read_text(encoding="utf-8")

    assert result.returncode == 0
    assert result.stdout.strip() == "no"
    assert "NO|no)" in text


def test_aborted_feeder_reaps_background_watchdog_publishers():
    """An old /infeed/fed event must never leak into the next Gazebo world."""
    text = SCRIPT.read_text(encoding="utf-8")
    feeder = text[text.index("feeder_cleanup() {"):text.index("FEEDER=$!")]

    assert "jobs -pr" in feeder
    assert 'kill "$child"' in feeder
    assert "wait 2>/dev/null || true" in feeder
    assert "trap 'exit 143' TERM" in feeder


def test_physical_landing_cannot_hide_a_missing_pipeline_id():
    """An item swept by its predecessor's blade is not a valid system PASS."""
    text = SCRIPT.read_text(encoding="utf-8")
    roster = text[text.index("EXPECTED_IDS="):text.index("# The stream's own summary")]

    assert "PERCEPTION_COUNT" in roster
    assert "CLASSIFIER_COUNT" in roster
    assert "CONTROLLER_COUNT" in roster
    assert 'RUN_ERROR="roster mismatch:' in roster
    assert "roster: planned=$EXPECTED_IDS" in text


def test_feeder_failure_and_false_create_reply_reach_the_parent():
    """Failed creation must stop the run instead of becoming N terminal FAIL rows."""
    text = SCRIPT.read_text(encoding="utf-8")

    assert "CREATE_REPLY=$(ign service" in text
    assert 'grep -Eq "data:[[:space:]]*true"' in text
    assert "FEEDER_ERROR: create rejected" in text
    assert 'wait "$FEEDER" || FEEDER_RC=$?' in text
    assert "feeder exited with status" in text
    assert "wait $FEEDER 2>/dev/null || true" not in text


def test_equal_rounded_arrivals_are_reported_without_division_by_zero():
    result = _run_inline_throughput([1.0, 1.0])

    assert result.returncode == 0, result.stderr
    assert "unresolved nonpositive arrival intervals: 1" in result.stdout
    assert "items/min" not in result.stdout


def test_positive_arrival_gaps_remain_measurable_beside_unresolved_ones():
    result = _run_inline_throughput([1.0, 1.0, 2.5])

    assert result.returncode == 0, result.stderr
    assert "unresolved nonpositive arrival intervals: 1" in result.stdout
    assert "takt between arrivals: 1.5 s (1.5 s) => 40 items/min" in result.stdout


def test_summary_pipeline_propagates_python_failure_after_ros_setup():
    """A green tee must not mask a failed inline throughput calculation."""
    text = SCRIPT.read_text(encoding="utf-8")

    ros_setup = text.index('source "$ROS_INSTALL_ROOT/setup.bash"')
    pipefail = text.index("set -o pipefail")
    summary_pipeline = text.index('} | tee "$LOGDIR/stream.log"')

    assert ros_setup < pipefail < summary_pipeline
