"""The deck-layout checker must catch the defect class it was written for.

The three collisions found on a projector 01.08 were all the same shape: a slide
grows, `.foot` is absolutely positioned so it does not get pushed, and the two
texts render on top of each other while the HTML still looks fine. A checker that
returns "OK" for that is worse than no checker, so the guard here is a page built
to fail, not the real deck.

Playwright is a dev-only tool: it is not in requirements.txt and CI never
installs it, so every test skips cleanly where the browser is absent.
"""
from importlib.util import find_spec
from pathlib import Path

import pytest

from scripts.check_deck_layout import audit

# SKIP AT THE TEST, NOT AT IMPORT. `pytest.importorskip` at module level raises
# Skipped while the file is being imported, and pytest 6.2.5 — the apt version the
# colcon job runs — aborts the whole collection there: the CI run reported
# "1 skipped" and exited 5 (no tests collected), so every other test in the suite
# silently stopped running. A marker skips these four tests and leaves collection
# untouched on any pytest version.
_NO_PLAYWRIGHT = find_spec("playwright") is None
pytestmark = pytest.mark.skipif(
    _NO_PLAYWRIGHT, reason="playwright is a dev-only tool; not installed here")

ROOT = Path(__file__).resolve().parents[1]
SHIPPING_DECK = ROOT / "docs" / "report" / "slides" / "deck-c-ozon.html"


def _page(body: str) -> str:
    """A minimal stage with the same geometry contract as the real deck."""
    return f"""<!DOCTYPE html><html><head><meta charset="utf-8"><style>
      * {{ margin:0; padding:0; box-sizing:border-box; }}
      .deck-stage {{ position:absolute; left:0; top:0; width:1920px; height:1080px; }}
      .slide {{ position:absolute; inset:0; width:1920px; height:1080px; overflow:hidden;
               visibility:hidden; font:20px/1.4 monospace; color:#fff; background:#123; }}
      .slide.active, .slide.visible {{ visibility:visible; }}
      .foot {{ position:absolute; left:128px; right:128px; bottom:64px; }}
    </style></head><body>
      <main class="deck-stage">{body}</main>
    </body></html>"""


def _chromium_ok() -> bool:
    # Called at import time by the marker below, so it has to survive playwright
    # being absent entirely — not just the browser binary being missing.
    if _NO_PLAYWRIGHT:
        return False
    from playwright.sync_api import sync_playwright
    try:
        with sync_playwright() as p:
            p.chromium.launch().close()
        return True
    except Exception:
        return False


needs_browser = pytest.mark.skipif(
    not _chromium_ok(), reason="chromium binary not installed for playwright")


@pytest.fixture
def written(tmp_path):
    def _write(body):
        path = tmp_path / "deck.html"
        path.write_text(_page(body), encoding="utf-8")
        return path
    return _write


@needs_browser
def test_text_running_under_the_footer_is_reported(written):
    # 980px + three tall lines lands squarely on .foot — the exact failure the
    # cover slide had, where the team names drew over the footer strip.
    deck = written("""
      <section class="slide">
        <div style="position:absolute; left:128px; top:980px; width:1100px; line-height:2.4">
          Команда и участники, строка первая<br>строка вторая<br>строка третья
        </div>
        <div class="foot"><span>подвал</span></div>
      </section>""")

    kinds = [p["kind"] for row in audit(deck) for p in row["problems"]]

    assert "COLLIDES_WITH_CHROME" in kinds


@needs_browser
def test_content_past_the_bottom_of_the_stage_is_reported(written):
    deck = written("""
      <section class="slide">
        <div style="position:absolute; left:128px; top:1200px">Уехало за сцену</div>
      </section>""")

    kinds = [p["kind"] for row in audit(deck) for p in row["problems"]]

    assert "OVERFLOW_BOTTOM" in kinds


@needs_browser
def test_a_slide_that_fits_reports_nothing(written):
    # Guards the other direction: a checker that flags everything is also useless.
    deck = written("""
      <section class="slide">
        <div style="position:absolute; left:128px; top:300px">Обычный заголовок</div>
        <div class="foot"><span>подвал</span></div>
      </section>""")

    assert audit(deck) == [{"index": 1, "problems": []}]


@needs_browser
def test_the_shipping_deck_currently_fits():
    """The deck the jury sees is measured, not trusted.

    Rendered with entrance animations frozen, so this is the geometry at rest —
    the first version of the checker measured mid-transition and reported boxes
    up to 30px below where they actually land.
    """
    problems = {row["index"]: row["problems"] for row in audit(SHIPPING_DECK)}
    broken = {index: items for index, items in problems.items() if items}

    assert not broken, f"slides do not fit 1920x1080: {sorted(broken)}"


@needs_browser
def test_a_loud_block_above_the_heading_is_reported(written):
    """The rule the deck learned on 02.08: nothing shouts before the title does.

    A 58px accent panel sat above the heading on the limits slide and read as
    noise before the slide had said anything. Small quiet chrome above the title
    stays legal, so the guard keys on type size rather than position alone.
    """
    deck = written("""
      <section class="slide">
        <div style="position:absolute; left:128px; top:176px; font-size:58px">CV 4</div>
        <h2 style="position:absolute; left:128px; top:224px; font-size:82px">Заголовок</h2>
      </section>""")

    kinds = [p["kind"] for row in audit(deck) for p in row["problems"]]

    assert "ABOVE_HEADING" in kinds


@needs_browser
def test_small_quiet_text_above_the_heading_is_allowed(written):
    # The kicker and the chrome strip live above the title by design.
    deck = written("""
      <section class="slide">
        <div style="position:absolute; left:128px; top:120px; font-size:18px">служебная строка</div>
        <h2 style="position:absolute; left:128px; top:224px; font-size:82px">Заголовок</h2>
      </section>""")

    kinds = [p["kind"] for row in audit(deck) for p in row["problems"]]

    assert "ABOVE_HEADING" not in kinds


@needs_browser
def test_text_landing_on_other_text_is_reported(written):
    """The gap that let a broken cover pass: only chrome collisions were checked.

    Widening the cover heading made it wrap to a second line, which came down on
    the lede — two paragraphs drawn on top of each other, and the checker still
    reported OK because neither was `.foot` or `.chrome`.
    """
    deck = written("""
      <section class="slide">
        <div style="position:absolute; left:128px; top:300px; font-size:60px">Заголовок в две строки</div>
        <div style="position:absolute; left:128px; top:330px; font-size:30px">Лид, который лёг сверху</div>
      </section>""")

    kinds = [p["kind"] for row in audit(deck) for p in row["problems"]]

    assert "TEXT_OVERLAPS_TEXT" in kinds
