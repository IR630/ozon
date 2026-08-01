"""The speech budget is a measurement, not an opinion.

The 7-minute limit is hard, and both drafts of this speech overshot it while
reading as if they fit: 7:02 for nine slides, 7:04 for twelve. So the number is
computed, and the shipped text is held to it by a test rather than by whoever
last remembered to re-count.
"""
from pathlib import Path

from scripts.measure_speech import (
    CLIP_SILENCE_SECONDS,
    LIMIT_SECONDS,
    SLIDE_STAMP,
    measure,
    slides,
    stamp,
)

ROOT = Path(__file__).resolve().parents[1]
SPEECH = (ROOT / "docs" / "report" / "defence_speech.md").read_text(encoding="utf-8")


def test_every_deck_slide_has_its_block_of_speech():
    # A speech that silently loses a slide leaves the presenter improvising in
    # front of the jury; the deck grew from 9 to 12 exactly once already.
    deck = (ROOT / "docs" / "report" / "slides" / "deck-c-ozon.html").read_text(encoding="utf-8")
    slide_count = deck.count('<section class="slide')

    assert len(slides(SPEECH)) == slide_count


def test_stage_directions_are_not_counted_as_speech():
    """The bracketed cue is an instruction to the presenter, not words to read.

    Counting it would inflate the estimate and, worse, hide real overrun behind
    padding nobody says out loud.
    """
    spoken = "## Слайд 1 — Проба\n\nОдин два три четыре пять.\n"
    with_cue = spoken + "\n**[ПРОБЕЛ — клипы запускаются вместе. Молчим двадцать секунд.]**\n"

    assert measure(with_cue)[1] == measure(spoken)[1] == 5


def test_the_shipped_speech_fits_the_seven_minute_limit_with_margin():
    _, words, speech_seconds = measure(SPEECH)
    total = speech_seconds + CLIP_SILENCE_SECONDS

    assert total <= LIMIT_SECONDS, f"speech is {total:.0f}s against a {LIMIT_SECONDS}s limit"
    # A limit hit exactly is a limit missed on the day: nerves cost seconds.
    assert LIMIT_SECONDS - total >= 20, (
        f"only {LIMIT_SECONDS - total:.0f}s of margin — too thin for a live defence")
    assert words > 400, "speech looks truncated"


def test_every_slide_heading_carries_a_timing_the_presenter_can_use():
    headings = [line for line in SPEECH.splitlines() if line.startswith("## Слайд ")]

    assert headings, "speech has no slide headings"
    unstamped = [h for h in headings if not SLIDE_STAMP.search(h)]
    assert not unstamped, f"slides without a timing stamp: {unstamped}"


def test_the_stamped_timings_match_the_current_text():
    """A hand-edited heading is how the budget and the text drift apart.

    Re-stamping is idempotent, so a file whose stamps are current does not change
    when stamped again. If this fails, the speech was edited without re-running
    `measure_speech.py --write` and its printed timings are lying.
    """
    assert stamp(SPEECH) == SPEECH, (
        "timings are stale — run: python scripts/measure_speech.py --write")


def test_the_running_clock_ends_where_the_total_does():
    # The last slide's cumulative mark IS the run time; if the two disagree, one
    # of them is wrong and the presenter is pacing against a fiction.
    _, _, speech_seconds = measure(SPEECH)
    total = speech_seconds + CLIP_SILENCE_SECONDS

    last = [line for line in SPEECH.splitlines() if line.startswith("## Слайд ")][-1]
    minutes, seconds = last.rsplit("к ", 1)[1].split(":")
    assert abs(int(minutes) * 60 + int(seconds) - total) <= 1
