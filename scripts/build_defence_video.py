#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Assemble the defence video: recorded narration plus the deck, slide by slide.

Each slide is held for exactly as long as its own audio track lasts, so the
montage cannot drift: there is no global frame rate to keep in sync with, only
twelve independent durations read from the files themselves.

Slide 8 is the exception and is RECORDED FROM THE BROWSER rather than rendered as
a still, because it is the only slide whose content moves — five clips playing at
once. Compositing those five rectangles by hand in ffmpeg would duplicate the
deck's own layout maths, and any later change to the slide would silently desync
the two copies.

    python scripts/build_defence_video.py                 # -> docs/report/video/defence.mp4
    python scripts/build_defence_video.py --plan          # timing only, nothing rendered
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DECK = ROOT / "docs" / "report" / "slides" / "deck-c-ozon.html"
OUT = ROOT / "docs" / "report" / "video" / "defence.mp4"

# The slide whose clips must actually run. 1-based, matching the audio names.
LIVE_SLIDE = 8
WIDTH, HEIGHT, FPS = 1920, 1080, 25


NARRATION = "docs/report/video/narration"


def audio_tracks(root: Path) -> list[Path]:
    """The twelve narration files, in slide order.

    ASCII names on purpose: the recordings arrived as `слайд1.aac`, and Cyrillic
    paths in git are a portability trap on machines whose locale is not UTF-8.
    """
    tracks = [root / NARRATION / f"s{i:02d}.aac" for i in range(1, 13)]
    missing = [t.name for t in tracks if not t.is_file()]
    if missing:
        raise SystemExit("нет записей: " + ", ".join(missing))
    return tracks


def _ffmpeg() -> tuple[str, str]:
    """(ffmpeg, ffprobe), from static_ffmpeg if present, else from PATH.

    static_ffmpeg is NOT in requirements.txt — it downloads a binary — so hard
    depending on it made the README's claim untrue: "собирается одной командой,
    поэтому ролик воспроизводим" met ModuleNotFoundError on any clean clone.
    `build_demo_reel.find_ffmpeg` already had the fallback pattern; this is the
    same one, extended to ffprobe, so the reel builds wherever ffmpeg exists.
    """
    try:
        import static_ffmpeg.run as runner
        return runner.get_or_fetch_platform_executables_else_raise()
    except ImportError:
        pass
    ffmpeg, ffprobe = shutil.which("ffmpeg"), shutil.which("ffprobe")
    if not ffmpeg or not ffprobe:
        raise SystemExit(
            "ABORT: neither static_ffmpeg nor a system ffmpeg/ffprobe was found.\n"
            "    pip install static-ffmpeg     # downloads its own binaries\n"
            "    # or install ffmpeg with your package manager")
    return ffmpeg, ffprobe


def duration(ffprobe: str, path: Path) -> float:
    out = subprocess.run(
        [ffprobe, "-v", "error", "-show_entries", "format=duration",
         "-of", "json", str(path)],
        capture_output=True, text=True, check=True).stdout
    return float(json.loads(out)["format"]["duration"])


def plan(root: Path = ROOT) -> list[tuple[int, float]]:
    _, ffprobe = _ffmpeg()
    return [(i, duration(ffprobe, t))
            for i, t in enumerate(audio_tracks(root), start=1)]


def _mmss(seconds: float) -> str:
    return f"{int(seconds) // 60}:{int(seconds) % 60:02d}"


def render_stills(work: Path, live_slide: int) -> dict[int, Path]:
    """One PNG per slide, entrance animations frozen so nothing is caught mid-fade."""
    from playwright.sync_api import sync_playwright

    shots: dict[int, Path] = {}
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": WIDTH, "height": HEIGHT})
        page.goto(DECK.resolve().as_uri())
        page.wait_for_load_state("networkidle")
        page.add_style_tag(content="""
            *, *::before, *::after { transition:none !important; animation:none !important; }
            .reveal { opacity:1 !important; transform:none !important; }
        """)
        page.wait_for_timeout(400)
        count = page.evaluate("document.querySelectorAll('.slide').length")

        # WALK THE DECK THE WAY A PRESENTER DOES. The first version toggled
        # `.active`/`.visible` by hand, which shows the right slide but skips
        # `showSlide()` — and that method is what advances the counter and the
        # progress bar. Every still therefore carried "1 / 12" in the corner
        # while the slide behind it changed. Pressing the real key runs the real
        # code path, so the chrome cannot disagree with the slide again.
        for index in range(count):
            if index + 1 != live_slide:
                page.wait_for_timeout(120)
                shot = work / f"slide-{index + 1:02d}.png"
                page.screenshot(path=str(shot))
                shots[index + 1] = shot
            if index + 1 < count:
                page.keyboard.press("ArrowRight")
        browser.close()
    return shots


def record_live_slide(work: Path, slide: int, seconds: float) -> tuple[Path, float]:
    """Film the moving slide straight from the browser, clips running.

    Returns the file AND the offset at which the slide is actually on screen.
    Recording starts when the CONTEXT is created, so the file always opens on
    the deck loading at slide 1 and walking to the target. Trimming from zero
    put that walk into the montage — a frame of slide 1 flashed before slide 8.
    The offset is measured rather than assumed, because page load and clip
    start-up are not constant.
    """
    import time

    from playwright.sync_api import sync_playwright

    videos = work / "live"
    videos.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as p:
        browser = p.chromium.launch(args=["--autoplay-policy=no-user-gesture-required"])
        started = time.monotonic()
        context = browser.new_context(
            viewport={"width": WIDTH, "height": HEIGHT},
            record_video_dir=str(videos),
            record_video_size={"width": WIDTH, "height": HEIGHT})
        page = context.new_page()
        page.goto(DECK.resolve().as_uri())
        page.wait_for_load_state("networkidle")
        for _ in range(slide - 1):
            page.keyboard.press("ArrowRight")
        # Let the five clips decode a frame and the entrance transition finish.
        page.wait_for_timeout(1600)
        offset = time.monotonic() - started
        page.wait_for_timeout(int(seconds * 1000) + 800)
        path = Path(page.video.path())
        context.close()
        browser.close()
    return path, offset


def build(root: Path, out: Path) -> int:
    ffmpeg, ffprobe = _ffmpeg()
    tracks = audio_tracks(root)
    lengths = [duration(ffprobe, t) for t in tracks]

    with tempfile.TemporaryDirectory() as tmp:
        work = Path(tmp)
        stills = render_stills(work, LIVE_SLIDE)
        live, live_offset = record_live_slide(work, LIVE_SLIDE, lengths[LIVE_SLIDE - 1])

        segments = []
        for index, seconds in enumerate(lengths, start=1):
            segment = work / f"seg-{index:02d}.mp4"
            if index == LIVE_SLIDE:
                # -ss BEFORE -i so the walk from slide 1 is skipped, not shown.
                cmd = [ffmpeg, "-y", "-v", "error",
                       "-ss", f"{live_offset:.3f}", "-i", str(live),
                       "-t", f"{seconds:.3f}"]
            else:
                cmd = [ffmpeg, "-y", "-v", "error", "-loop", "1",
                       "-i", str(stills[index]), "-t", f"{seconds:.3f}"]
            cmd += ["-r", str(FPS), "-c:v", "libx264", "-preset", "medium",
                    "-crf", "20", "-pix_fmt", "yuv420p",
                    "-vf", f"scale={WIDTH}:{HEIGHT}:force_original_aspect_ratio=decrease,"
                           f"pad={WIDTH}:{HEIGHT}:(ow-iw)/2:(oh-ih)/2",
                    "-an", str(segment)]
            subprocess.run(cmd, check=True)
            segments.append(segment)

        video_list = work / "video.txt"
        video_list.write_text(
            "".join(f"file '{s.as_posix()}'\n" for s in segments), encoding="utf-8")
        audio_list = work / "audio.txt"
        audio_list.write_text(
            "".join(f"file '{t.resolve().as_posix()}'\n" for t in tracks), encoding="utf-8")

        silent = work / "silent.mp4"
        subprocess.run([ffmpeg, "-y", "-v", "error", "-f", "concat", "-safe", "0",
                        "-i", str(video_list), "-c", "copy", str(silent)], check=True)
        voice = work / "voice.m4a"
        subprocess.run([ffmpeg, "-y", "-v", "error", "-f", "concat", "-safe", "0",
                        "-i", str(audio_list), "-c:a", "aac", "-b:a", "192k",
                        str(voice)], check=True)

        out.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run([ffmpeg, "-y", "-v", "error", "-i", str(silent), "-i", str(voice),
                        "-c:v", "copy", "-c:a", "aac", "-b:a", "192k",
                        "-shortest", str(out)], check=True)

    total = sum(lengths)
    print(f"собрано: {out}")
    print(f"длительность {_mmss(total)} ({total:.1f} с), {len(lengths)} слайдов")
    return 0


def main() -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):   # pragma: no cover - non-tty
            pass

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--plan", action="store_true", help="print timing, render nothing")
    ap.add_argument("--out", type=Path, default=OUT)
    args = ap.parse_args()

    rows = plan(ROOT)
    running = 0.0
    print(f"{'СЛАЙД':<8}{'ДОРОЖКА':>10}{'К КОНЦУ':>10}")
    for index, seconds in rows:
        running += seconds
        mark = "  <- живой слайд" if index == LIVE_SLIDE else ""
        print(f"{index:<8}{seconds:>9.1f}s{_mmss(running):>10}{mark}")
    total = sum(s for _, s in rows)
    print(f"\nвсего {_mmss(total)} ({total:.1f} с), лимит защиты 7:00")
    if total > 7 * 60:
        print("ПРЕВЫШЕН ЛИМИТ")

    if args.plan:
        return 0
    return build(ROOT, args.out)


if __name__ == "__main__":
    sys.exit(main())
