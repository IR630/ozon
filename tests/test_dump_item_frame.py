"""Static contracts for the reproducible real-frame capture runner."""
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "dump_item_frame.sh"


def test_capture_runner_exposes_position_and_frame_count_for_validation_slices():
    text = SCRIPT.read_text(encoding="utf-8")

    assert "SPAWN_X=${SPAWN_X:-1.5}" in text
    assert "position: {x: $SPAWN_X" in text
    assert "FRAMES=${FRAMES:-3}" in text
    assert '--frames "$FRAMES"' in text


def test_the_bridge_config_moves_with_the_world_seam():
    """A three-head world bridged by the one-head config is SILENT on the side
    topics, and the dump then reads as "the side heads see nothing" — the very
    conclusion such a dump is usually opened to test."""
    text = SCRIPT.read_text(encoding="utf-8")

    assert "BRIDGE_CONFIG=${BRIDGE_CONFIG:-sim/bridge.yaml}" in text
    assert 'config_file:="$PWD/$BRIDGE_CONFIG"' in text
    assert '"$PWD/sim/bridge.yaml"' not in text, "the bridge config is hard-coded again"


def test_an_inherited_out_dir_survives_the_positional_default():
    """`OUT=<dir> bash scripts/dump_item_frame.sh <slug>` is a written-down recipe
    (scripts/probe_noise_heads.py docstring), and a plain `:-` default silently won
    over it: the frames landed in /tmp while the operator believed they were in the
    repo, and the probe then scored against dumps nobody had refreshed."""
    text = SCRIPT.read_text(encoding="utf-8")

    assert "OUT=${2:-${OUT:-/tmp/frames_$SLUG}}" in text


def test_the_side_heads_can_be_dumped_and_are_off_by_default():
    """Off by default: in the one-camera world those topics never publish, so a
    dumper waiting on them would hang instead of finishing."""
    text = SCRIPT.read_text(encoding="utf-8")

    assert "${SIDE:+--side}" in text
    assert "SIDE=1 bash scripts/dump_item_frame.sh" in text, "write the invocation down"
