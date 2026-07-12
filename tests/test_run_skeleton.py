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
