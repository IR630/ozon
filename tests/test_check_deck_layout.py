"""The deck-layout checker must catch the defect class it was written for.

The three collisions found on a projector 01.08 were all the same shape: a slide
grows, `.foot` is absolutely positioned so it does not get pushed, and the two
texts render on top of each other while the HTML still looks fine. A checker that
returns "OK" for that is worse than no checker, so the guard here is a page built
to fail, not the real deck.

Playwright is a dev-only tool: it is not in requirements.txt and CI never
installs it, so every test skips cleanly where the browser is absent.
"""
import pytest

pytest.importorskip("playwright.sync_api", reason="playwright is a dev-only tool")

from pathlib import Path  # noqa: E402

from scripts.check_deck_layout import audit  # noqa: E402

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
