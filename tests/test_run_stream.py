"""Static regression checks for the multi-item stream runner."""

from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "run_stream.sh"


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


def test_feeder_failure_and_false_create_reply_reach_the_parent():
    """Failed creation must stop the run instead of becoming N terminal FAIL rows."""
    text = SCRIPT.read_text(encoding="utf-8")

    assert "CREATE_REPLY=$(ign service" in text
    assert 'grep -Eq "data:[[:space:]]*true"' in text
    assert "FEEDER_ERROR: create rejected" in text
    assert 'wait "$FEEDER" || FEEDER_RC=$?' in text
    assert "feeder exited with status" in text
    assert "wait $FEEDER 2>/dev/null || true" not in text
