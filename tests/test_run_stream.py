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
