#!/usr/bin/env python3
"""Rasterize the video inserts from their SVG sources.

The SVG is the source of truth and the PNG is what lands in the timeline, so the
two drift the moment a number is fixed in one of them — that is exactly how the
metrics card kept claiming a retired throughput figure while the documents around
it had moved on. One command re-renders every insert:

    python scripts/render_inserts.py

Uses the Chromium that ships with Playwright: the SVGs use gradients and
text-anchor, and a browser is the renderer they were authored against. The
previously documented rsvg-convert is not installed anywhere in this project's
environments, which is a large part of why the PNGs went stale.
"""
from __future__ import annotations

import sys
from pathlib import Path

INSERTS = Path("docs/report/video/inserts")
WIDTH, HEIGHT = 1920, 1080


def render(root: Path) -> int:
    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        print("ABORT: playwright is not installed (pip install playwright)", file=sys.stderr)
        return 1

    sources = sorted((root / INSERTS).glob("*.svg"))
    if not sources:
        print(f"ABORT: no SVG sources under {INSERTS}", file=sys.stderr)
        return 1

    with sync_playwright() as play:
        browser = play.chromium.launch()
        page = browser.new_page(viewport={"width": WIDTH, "height": HEIGHT})
        for source in sources:
            target = source.with_suffix(".png")
            page.goto(source.resolve().as_uri())
            page.screenshot(path=str(target))
            print(f"{target.relative_to(root).as_posix()}  "
                  f"{target.stat().st_size / 1024:.0f} KiB")
        browser.close()
    return 0


def main() -> int:
    return render(Path(__file__).resolve().parents[1])


if __name__ == "__main__":
    sys.exit(main())
