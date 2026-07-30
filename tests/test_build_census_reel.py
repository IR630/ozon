# -*- coding: utf-8 -*-
"""The census reel: what it films, how it trims, and what it may claim."""
from pathlib import Path

import cv2
import numpy as np
import pytest

from scripts.build_census_reel import (Cell, action_window, caption_lines,
                                       collect, parse_census_cells, read_verdict)

ROOT = Path(__file__).resolve().parents[1]
MATRIX = ROOT / "scripts" / "run_matrix.sh"
FPS = 10
SIZE = (320, 180)


def _write_clip(path: Path, painter) -> Path:
    """Render a synthetic clip; painter(index) returns one BGR frame."""
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), FPS, SIZE)
    index = 0
    while True:
        frame = painter(index)
        if frame is None:
            break
        writer.write(frame)
        index += 1
    writer.release()
    return path


def _still(value: int = 40) -> np.ndarray:
    return np.full((SIZE[1], SIZE[0], 3), value, dtype=np.uint8)


def _bringup_then_item(index: int):
    """The shape of a real cell: the world assembles, settles, then an item crosses.

    Bringup moves most of the frame; the item is a handful of pixels. Total 12 s.
    """
    if index >= 120:
        return None
    frame = _still()
    if index < 30:                      # world assembling — huge, global motion
        frame[:, : 4 * index] = 200
    elif 70 <= index < 86:              # the item: small, late, faint
        x = 20 + (index - 70) * 15
        frame[86:96, x:x + 10] = 255
    return frame


@pytest.fixture()
def bringup_clip(tmp_path):
    return _write_clip(tmp_path / "cell.mp4", _bringup_then_item)


def test_trim_window_covers_the_item_and_skips_bringup(bringup_clip):
    # THE BUG THIS PINS: the first cut kept the window with the MOST motion, and
    # that window is Gazebo spawning a 20 m belt while the camera already rolls.
    # On the real 31.07 bottle clip it trimmed to 3.4-14.4 s — bringup, no item.
    start, duration = action_window(bringup_clip, target=3.0, pad=0.5)
    end = start + duration
    # The item crosses between t=7.0 s and t=8.6 s.
    assert start <= 7.0 and end >= 8.6, f"item window missed: {start}..{end}"
    # Bringup runs 0-3 s and must NOT be what the reel shows.
    assert start >= 3.0, f"window starts inside bringup: {start}"


def test_trim_window_drops_the_empty_tail(bringup_clip):
    start, duration = action_window(bringup_clip, target=3.0, pad=0.5)
    # The clip is 12 s and the last 3.4 s are an empty belt.
    assert start + duration <= 10.0


def test_trim_survives_a_clip_where_nothing_moves(tmp_path):
    still = _write_clip(tmp_path / "still.mp4",
                        lambda i: None if i >= 40 else _still())
    start, duration = action_window(still, target=2.0, pad=0.5)
    assert duration > 0
    assert start >= 0


def test_cell_list_comes_from_the_census_script():
    # A second copy of the item/zone lists is the defect that shipped a deck
    # playing clips the repository did not contain. Parse, never restate.
    cells = parse_census_cells(MATRIX)
    assert len(cells) == 11, cells
    assert dict(cells)["pen"] == "C"
    assert dict(cells)["bottle"] == "D"
    assert dict(cells)["cylinder"] == "B"


def test_verdict_is_read_from_the_episode_log(tmp_path):
    log = tmp_path / "matrix_bottle_0.log"
    log.write_text(
        "some noise\n"
        "bottle -> D: PASS (pose x=3.88 y=-1.31 z=0.00, cycle 32.2s from launch)\n",
        encoding="utf-8")
    assert read_verdict(log, "bottle") == ("D", "PASS")


def test_a_log_without_a_verdict_aborts_instead_of_captioning_a_guess(tmp_path):
    log = tmp_path / "matrix_bottle_0.log"
    log.write_text("gazebo died here\n", encoding="utf-8")
    with pytest.raises(SystemExit, match="no terminal verdict"):
        read_verdict(log, "bottle")


def test_reel_refuses_a_zone_the_census_did_not_score(tmp_path):
    # If the episode log and run_matrix.sh disagree about the expected zone, the
    # caption would state a rule the census never applied. Refuse, do not render.
    clips, logs = tmp_path / "clips", tmp_path / "logs"
    clips.mkdir()
    logs.mkdir()
    (clips / "cell_bottle_0.mp4").write_bytes(b"")
    (logs / "matrix_bottle_0.log").write_text(
        "bottle -> B: PASS (pose x=3.88 y=0.0 z=0.0, cycle 30s from launch)\n",
        encoding="utf-8")
    with pytest.raises(SystemExit, match="run_matrix.sh lists D"):
        collect(clips, logs, MATRIX, poses=1)


def test_caption_names_the_item_the_pose_and_the_outcome():
    cell = Cell("bottle", 1, "D", "PASS", Path("x.mp4"))
    top, bottom = caption_lines(cell, index=12, total=33, poses=3)
    assert top == "Бутылка · поза 2/3"
    assert "D ожидалось → D" in bottom
    assert "[12/33]" in bottom
    assert "✓" in bottom


def test_a_failed_cell_is_not_captioned_as_a_pass():
    cell = Cell("pouf", 2, "C", "FAIL", Path("x.mp4"))
    _, bottom = caption_lines(cell, index=21, total=33, poses=3)
    assert "✗" in bottom
    assert "отказ" in bottom
    # It must NOT invent a landing zone: naming one needs a second ruler beside
    # zone_verdict.py, and two rulers is how a census disagrees with itself.
    assert "→ B" not in bottom and "→ D" not in bottom
