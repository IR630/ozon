"""Static regression checks for the Gazebo episode lifecycle."""

from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "run_skeleton.sh"


def test_episode_cleanup_runs_on_early_exit_and_reaps_stale_gazebo():
    text = SCRIPT.read_text(encoding="utf-8")

    assert "trap cleanup EXIT" in text
    assert "GAZEBO_PATTERN='^ign gazebo( |$)'" in text
    assert 'pkill -TERM -f "$GAZEBO_PATTERN"' in text
    assert 'pkill -KILL -f "$GAZEBO_PATTERN"' in text
    assert text.index("cleanup\n") < text.index('ign gazebo -s -r -v 0 "$WORLD"')


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
