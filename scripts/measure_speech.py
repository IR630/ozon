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

# The stamp `--write` puts on each heading, and the pattern that strips a stale
# one before writing a fresh stamp. Timings live in the headings because that is
# where the presenter reads them, but they are GENERATED — hand-editing a heading
# is how the text and its budget drift apart.
SLIDE_STAMP = re.compile(r"\s+·\s+\d+:\d{2}\s+·\s+к\s+\d+:\d{2}\s*$", re.M)

# Which slide holds the clip, 1-based: its pause lands inside the running clock.
CLIP_SLIDE = 8


def slides(text: str) -> list[tuple[str, str]]:
    """(heading, spoken body) per slide, in deck order."""
    marks = list(SLIDE_HEADING.finditer(text))
    out = []
    for i, m in enumerate(marks):
        end = marks[i + 1].start() if i + 1 < len(marks) else len(text)
        body = text[m.end():end]
        body = body.split("\n---", 1)[0]
        title = SLIDE_STAMP.sub("", m.group(2)).strip()
        out.append((f"{m.group(1)} {title}", STAGE_DIRECTION.sub(" ", body)))
    return out


def stamp(text: str) -> str:
    """Rewrite every slide heading with its measured length and running clock.

    The running clock is the number a presenter actually uses: "by the end of
    this slide the timer should read 2:15" catches drift three slides before the
    limit does. The clip pause is charged to the slide that plays it.
    """
    rows, _, _ = measure(text)
    lengths = [seconds for _, _, seconds in rows]
    running, marks = 0.0, []
    for index, seconds in enumerate(lengths, start=1):
        running += seconds
        if index == CLIP_SLIDE:
            running += CLIP_SILENCE_SECONDS
        marks.append(running)

    counter = [0]

    def rewrite(match: re.Match) -> str:
        i = counter[0]
        counter[0] += 1
        title = SLIDE_STAMP.sub("", match.group(2)).strip()
        return (f"## Слайд {match.group(1)} — {title}"
                f" · {_mmss(lengths[i])} · к {_mmss(marks[i])}")

    return SLIDE_HEADING.sub(rewrite, text)


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

    text = SPEECH.read_text(encoding="utf-8")

    if "--write" in sys.argv:
        stamped = stamp(text)
        if stamped != text:
            SPEECH.write_text(stamped, encoding="utf-8")
            print("тайминги в заголовках обновлены\n")
        else:
            print("тайминги в заголовках уже актуальны\n")
        text = stamped

    rows, words, speech_seconds = measure(text)
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
