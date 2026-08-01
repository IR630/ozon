#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Measure the defence speech against the 7-minute limit.

WHY. The limit is hard and the deck is 12 slides. The first draft of this speech
"looked like six minutes" and measured 7:02 — over. Word count is the only honest
way to know, and it has to be re-run after every edit, so it is a command rather
than a paragraph someone remembers to redo.

Bracketed stage directions are not read aloud and are excluded; the pause under
the demo clip is added back as a fixed budget, because silence still spends the
limit.

    python scripts/measure_speech.py
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEECH = ROOT / "docs" / "report" / "defence_speech.md"

# Calm presenting pace for Russian prose. Deliberately not the faster rate a
# rehearsed speaker hits: the budget must survive the day, not the best take.
WORDS_PER_MINUTE = 120
LIMIT_SECONDS = 7 * 60
# Slide 8 runs five clips at once and the room needs to watch them.
CLIP_SILENCE_SECONDS = 20

SLIDE_HEADING = re.compile(r"^## Слайд (\d+) — (.+)$", re.M)
STAGE_DIRECTION = re.compile(r"\*\*\[.*?\]\*\*", re.S)
WORD = re.compile(r"[А-Яа-яЁёA-Za-z0-9]+")


def slides(text: str) -> list[tuple[str, str]]:
    """(heading, spoken body) per slide, in deck order."""
    marks = list(SLIDE_HEADING.finditer(text))
    out = []
    for i, m in enumerate(marks):
        end = marks[i + 1].start() if i + 1 < len(marks) else len(text)
        body = text[m.end():end]
        body = body.split("\n---", 1)[0]
        out.append((f"{m.group(1)} {m.group(2)}", STAGE_DIRECTION.sub(" ", body)))
    return out


def measure(text: str) -> tuple[list[tuple[str, int, float]], int, float]:
    rows, total_words = [], 0
    for heading, body in slides(text):
        words = len(WORD.findall(body))
        total_words += words
        rows.append((heading, words, words / WORDS_PER_MINUTE * 60))
    return rows, total_words, total_words / WORDS_PER_MINUTE * 60


def _mmss(seconds: float) -> str:
    sign = "-" if seconds < 0 else ""
    seconds = abs(int(seconds))
    return f"{sign}{seconds // 60}:{seconds % 60:02d}"


def main() -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):   # pragma: no cover - non-tty
            pass

    if not SPEECH.is_file():
        print(f"speech not found: {SPEECH}")
        return 2

    rows, words, speech_seconds = measure(SPEECH.read_text(encoding="utf-8"))
    total = speech_seconds + CLIP_SILENCE_SECONDS
    left = LIMIT_SECONDS - total

    print(f"{'СЛАЙД':<34}{'СЛОВ':>6}{'РЕЧЬ':>8}")
    for heading, slide_words, seconds in rows:
        print(f"{heading[:33]:<34}{slide_words:>6}{_mmss(seconds):>8}")

    print()
    print(f"{'Речь':<34}{words:>6}{_mmss(speech_seconds):>8}")
    print(f"{'+ пауза под клип':<34}{'':>6}{_mmss(CLIP_SILENCE_SECONDS):>8}")
    print(f"{'ВСЕГО':<34}{'':>6}{_mmss(total):>8}")
    print(f"{'ЗАПАС до 7:00':<34}{'':>6}{_mmss(left):>8}")

    if left < 0:
        print("\nНЕ ВЛЕЗАЕТ в лимит — резать текст.")
        return 1
    if left < 20:
        print("\nЗапас меньше 20 с — на защите этого мало.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
