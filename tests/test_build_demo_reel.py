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


def _clips_on_disk() -> bool:
    return all((ROOT / "docs" / "report" / "video" / s.name).is_file()
               for s in SEGMENTS if isinstance(s, Clip))


def test_every_still_the_reel_references_exists():
    # Stills are rendered from tracked SVGs, so unlike the footage they must be
    # present in every clone.
    for segment in SEGMENTS:
        if isinstance(segment, Still):
            assert (ROOT / segment.path).is_file(), segment.path


def test_a_clip_present_on_disk_is_also_in_the_repository():
    """Whoever records a clip must commit it, or the reel is unbuildable elsewhere.

    Presence on disk is checked the other way round on purpose. Asserting that
    every planned clip exists locally only ever passed on the machine that
    recorded the footage and failed everywhere else, which is why it sat red in CI
    from 28.07 while telling nobody anything new. The real defect — a clip the plan
    needs that the repository does not carry — is reported against git by
    ``check_submission.reel_clip_issues``. What is left for this test is the
    opposite drift: footage sitting beside the repo, never committed.
    """
    from scripts.check_submission import tracked_files

    tracked = set(tracked_files(ROOT))
    if not tracked:                      # no usable git index (colcon container)
        pytest.skip("no git inventory available here")
    for segment in SEGMENTS:
        if isinstance(segment, Clip):
            relative = f"docs/report/video/{segment.name}"
            if (ROOT / relative).is_file():
                assert relative in tracked, (
                    f"{segment.name} is on disk but not in the repository")


def test_reel_fits_the_seven_minute_defence_budget():
    if not _clips_on_disk():
        pytest.skip("reel footage is not all present locally; durations unmeasurable")
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
