"""Static contracts for the reproducible real-frame capture runner."""
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "dump_item_frame.sh"


def test_capture_runner_exposes_position_and_frame_count_for_validation_slices():
    text = SCRIPT.read_text(encoding="utf-8")

    assert "SPAWN_X=${SPAWN_X:-1.5}" in text
    assert "position: {x: $SPAWN_X" in text
    assert "FRAMES=${FRAMES:-3}" in text
    assert '--frames "$FRAMES"' in text
