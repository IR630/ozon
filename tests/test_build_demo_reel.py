from pathlib import Path

import pytest

from scripts.build_demo_reel import SEGMENTS, Card, Clip, Still, plan


ROOT = Path(__file__).resolve().parents[1]
MAX_DEFENCE_SECONDS = 7 * 60


def _ffmpeg():
    from scripts.build_demo_reel import find_ffmpeg
    try:
        return find_ffmpeg()
    except SystemExit:
        pytest.skip("ffmpeg is not available in this environment")


def test_every_asset_the_reel_references_exists():
    # A missing clip is the failure this script must never hit silently: the
    # concat would simply produce a shorter video than the script describes.
    for segment in SEGMENTS:
        if isinstance(segment, Clip):
            assert (ROOT / "docs" / "report" / "video" / segment.name).is_file(), segment.name
        elif isinstance(segment, Still):
            assert (ROOT / segment.path).is_file(), segment.path


def test_reel_fits_the_seven_minute_defence_budget():
    total = sum(row[1] for row in plan(ROOT, _ffmpeg()))
    assert total <= MAX_DEFENCE_SECONDS, f"reel is {total:.0f}s, budget {MAX_DEFENCE_SECONDS}s"


def test_reel_covers_all_three_categories_with_footage():
    # The jury has to see B, C and D handled — captions carry the verdict, so
    # a reel that lost a category would still build and still look finished.
    captions = " ".join(s.caption for s in SEGMENTS if isinstance(s, Clip))
    for zone in ("→ B", "→ C", "→ D"):
        assert zone in captions, f"no clip demonstrates {zone}"


def test_cards_have_a_heading_line():
    # Line 0 is rendered larger as the heading; an empty first line would look
    # like a rendering bug rather than a deliberate blank.
    for segment in SEGMENTS:
        if isinstance(segment, Card):
            assert segment.lines and segment.lines[0].strip(), segment.lines
