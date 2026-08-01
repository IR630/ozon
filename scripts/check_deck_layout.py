#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render every slide of a deck and report text that does not fit.

WHY THIS EXISTS. The deck is authored at a fixed 1920x1080 and every slide sets
``overflow:hidden``, so content that grows past the stage is not visibly broken
in the editor — it is silently CLIPPED. Worse, ``.foot`` and ``.chrome`` are
absolutely positioned, so flowing content does not push them: it slides
UNDERNEATH them and the two texts render on top of each other. Both failures
look fine in the HTML and only appear on a projector, which is where they were
actually found (01.08, three slides at once).

Checking by eye does not scale to a deck that changes: the same three defects
have to be re-checked after every edit. So the check is a render, and the
verdict is geometry, not opinion.

    python scripts/check_deck_layout.py                     # ships-deck, exit 1 on defects
    python scripts/check_deck_layout.py --shots out/        # also write PNGs
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DECK = ROOT / "docs" / "report" / "slides" / "deck-c-ozon.html"

STAGE_W, STAGE_H = 1920, 1080

# A hairline of overlap is antialiasing, not a defect. Two lines of 15px mono
# sitting on each other is ~20px, so the floor sits well below that and well
# above rounding noise.
TOLERANCE_PX = 4

# Absolutely-positioned chrome does not participate in layout: content flows
# under it instead of pushing it. These are the elements to test collisions
# against, and they are the ones that actually collided.
OVERLAY_SELECTORS = (".foot", ".chrome")

# Measuring every node double-counts: a parent's box contains its children, so
# one long line is reported once per ancestor. Leaves are where text lives.
PROBE_JS = r"""
(args) => {
  const [tolerance, overlaySelectors] = args;
  const stage = document.querySelector('.deck-stage');
  const slides = Array.from(document.querySelectorAll('.slide'));
  const report = [];

  const label = (el) => {
    const cls = (el.className && el.className.baseVal !== undefined)
      ? el.className.baseVal : (el.className || '');
    const t = (el.textContent || '').trim().replace(/\s+/g, ' ').slice(0, 60);
    return el.tagName.toLowerCase() + (cls ? '.' + String(cls).trim().split(/\s+/).join('.') : '')
           + (t ? ' — "' + t + '"' : '');
  };

  slides.forEach((slide, index) => {
    const restore = slide.className;
    slide.className = restore + ' active visible';
    slide.getBoundingClientRect();

    const stageBox = stage.getBoundingClientRect();
    const toStage = (r) => ({
      left: r.left - stageBox.left, top: r.top - stageBox.top,
      right: r.right - stageBox.left, bottom: r.bottom - stageBox.top,
    });

    const problems = [];
    const all = Array.from(slide.querySelectorAll('*'));

    // Leaves that carry visible text; skip the decorative layers outright.
    const probes = all.filter((el) => {
      if (el.closest('.field') || el.tagName === 'svg' || el.closest('svg')) return false;
      const style = getComputedStyle(el);
      if (style.display === 'none' || style.visibility === 'hidden') return false;
      const hasElementChild = Array.from(el.children).some(
        (c) => c.tagName !== 'BR' && c.tagName !== 'B' && c.tagName !== 'SPAN');
      if (hasElementChild) return false;
      return (el.textContent || '').trim().length > 0;
    });

    const overlays = overlaySelectors
      .map((sel) => slide.querySelector(sel))
      .filter(Boolean)
      .map((el) => ({ el, box: toStage(el.getBoundingClientRect()) }));

    probes.forEach((el) => {
      const box = toStage(el.getBoundingClientRect());
      if (box.width === 0 && box.height === 0) return;

      if (box.bottom > 1080 + tolerance) {
        problems.push({ kind: 'OVERFLOW_BOTTOM',
                        detail: Math.round(box.bottom - 1080) + 'px past the stage',
                        node: label(el) });
      }
      if (box.right > 1920 + tolerance) {
        problems.push({ kind: 'OVERFLOW_RIGHT',
                        detail: Math.round(box.right - 1920) + 'px past the stage',
                        node: label(el) });
      }

      overlays.forEach(({ el: overlayEl, box: ov }) => {
        if (el === overlayEl || overlayEl.contains(el)) return;
        const dx = Math.min(box.right, ov.right) - Math.max(box.left, ov.left);
        const dy = Math.min(box.bottom, ov.bottom) - Math.max(box.top, ov.top);
        if (dx > tolerance && dy > tolerance) {
          problems.push({ kind: 'COLLIDES_WITH_CHROME',
                          detail: 'overlaps ' + label(overlayEl).split(' — ')[0]
                                  + ' by ' + Math.round(dy) + 'px',
                          node: label(el) });
        }
      });
    });

    slide.className = restore;
    report.push({ index: index + 1, problems });
  });

  return report;
}
"""


def audit(deck: Path, shots: Path | None = None) -> list[dict]:
    from playwright.sync_api import sync_playwright

    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": STAGE_W, "height": STAGE_H})
        page.goto(deck.resolve().as_uri())

        # MEASURE THE DECK AT REST. Every content block carries `.reveal`, which
        # holds it at translateY(30px) until its entrance transition finishes.
        # Measuring during that window reports boxes up to 30px lower than where
        # they actually land — the checker would invent overflow and a screenshot
        # would catch a half-faded slide. Freezing the animations is what makes
        # the geometry the FINAL geometry.
        page.add_style_tag(content="""
            *, *::before, *::after {
                transition:none !important; animation:none !important;
            }
            .reveal { opacity:1 !important; transform:none !important; }
        """)

        # Webfonts change metrics, and metrics are the whole measurement here.
        page.wait_for_load_state("networkidle")
        try:
            page.evaluate("document.fonts.ready")
        except Exception:                     # pragma: no cover - older engines
            pass
        page.wait_for_timeout(400)

        report = page.evaluate(PROBE_JS, [TOLERANCE_PX, list(OVERLAY_SELECTORS)])

        if shots:
            shots.mkdir(parents=True, exist_ok=True)
            count = page.evaluate("document.querySelectorAll('.slide').length")
            for i in range(count):
                page.evaluate(
                    """(i) => {
                        document.querySelectorAll('.slide').forEach((s, j) => {
                            s.classList.toggle('active', i === j);
                            s.classList.toggle('visible', i === j);
                        });
                    }""", i)
                page.wait_for_timeout(150)
                page.screenshot(path=str(shots / f"slide-{i + 1:02d}.png"))

        browser.close()
    return report


def main() -> int:
    # The report quotes slide text, which is Russian and contains arrows. A
    # Windows console defaults to cp1251 and dies on the first "→", taking the
    # findings with it — the checker must not fail at the moment it succeeds.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):   # pragma: no cover - non-tty
            pass

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("deck", nargs="?", type=Path, default=DEFAULT_DECK)
    ap.add_argument("--shots", type=Path, help="directory for per-slide PNGs")
    args = ap.parse_args()

    if not args.deck.is_file():
        print(f"deck not found: {args.deck}")
        return 2

    report = audit(args.deck, args.shots)
    bad = [row for row in report if row["problems"]]

    for row in report:
        if not row["problems"]:
            continue
        print(f"\nSLIDE {row['index']}")
        for problem in row["problems"]:
            print(f"  {problem['kind']}: {problem['detail']}")
            print(f"    {problem['node']}")

    print()
    if bad:
        total = sum(len(row["problems"]) for row in bad)
        print(f"DECK LAYOUT: {total} problem(s) on {len(bad)} of {len(report)} slides")
        return 1
    print(f"DECK LAYOUT: OK — {len(report)} slides fit 1920x1080")
    return 0


if __name__ == "__main__":
    sys.exit(main())
