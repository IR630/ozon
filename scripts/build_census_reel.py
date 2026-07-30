#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Assemble the census demonstration reel: every cell of the 33-cell census.

Deliberately NOT build_demo_reel.py. That reel EXPLAINS the solution — title
cards, a narration track, an argument. The defence pitch covers that ground, and
repeating it wastes the one thing a demonstration is for. This reel only SHOWS
the thing working: 11 items x 3 seeded orientations, each trimmed to the moment
it is actually sorted, captioned with what it is and where it went.

    # 1. film the census (WSL, ~35 min for 33 cells)
    CENSUS_VIDEO_DIR=runs/census_video SKELETON="bash scripts/record_census_cell.sh" \
        SPECTATOR_POSE="0.2 -3.2 2.4 0 0.415 0.785" CELL_TIMEOUT=600 \
        LOGDIR=runs/census_video_logs bash scripts/run_matrix.sh 0 3
    # 2. assemble (Windows, where static_ffmpeg lives)
    python scripts/build_census_reel.py --clips runs/census_video \
        --logs runs/census_video_logs

The cell list is NOT restated here: it is parsed out of run_matrix.sh, which stays
the only definition of what a census is. The expected zone parsed from each
episode log is checked against it, so a reel can never caption a cell with a zone
the census did not actually score it against.
"""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from build_demo_reel import (BACKDROP, FPS, HEIGHT, WIDTH, _escaped_font,
                             _render, _write_text, find_ffmpeg, find_font)
from build_item_models import ITEMS

ROOT = Path(__file__).resolve().parents[1]
MATRIX = ROOT / "scripts" / "run_matrix.sh"

# Seconds of footage kept per cell, and the padding around the detected action.
# 11 s at 33 cells is a ~6 min reel — long enough to read each sort, short enough
# that a juror watches it to the end.
TARGET_SECONDS = 11.0
PAD_SECONDS = 1.5
# A frame counts as "something moved" at this many robust sigmas over the clip's
# own quiet level; ABSOLUTE_FLOOR keeps a perfectly still clip from promoting pure
# rounding noise to an event. Gaps shorter than GAP_SECONDS stay inside one burst.
QUIET_SIGMAS = 6.0
ABSOLUTE_FLOOR = 0.05
GAP_SECONDS = 1.5


@dataclass(frozen=True)
class Cell:
    """One census cell: what ran, and how it was scored."""

    slug: str
    oi: int
    expect: str      # zone the census scored this cell against
    verdict: str     # PASS | FAIL
    clip: Path


def parse_census_cells(matrix: Path) -> list[tuple[str, str]]:
    """(slug, expected zone) in census order, read from run_matrix.sh itself.

    A second copy of these two lists is precisely the defect that already cost
    this project a day: the deck played rig2_* while .gitignore re-included
    rig3_*, and nobody noticed until a clean clone showed broken video. So the
    reel derives its order from the census script instead of restating it.
    """
    text = matrix.read_text(encoding="utf-8")
    slugs = re.search(r"^SLUGS=\(([^)]*)\)", text, re.MULTILINE)
    zones = re.search(r"^ZONES=\(([^)]*)\)", text, re.MULTILINE)
    if not slugs or not zones:
        raise SystemExit(f"ABORT: cannot find SLUGS/ZONES arrays in {matrix}")
    names = slugs.group(1).split()
    wanted = zones.group(1).split()
    if len(names) != len(wanted):
        raise SystemExit(
            f"ABORT: {matrix} lists {len(names)} items but {len(wanted)} zones")
    return list(zip(names, wanted))


def read_verdict(log: Path, slug: str) -> tuple[str, str]:
    """(expected zone, PASS|FAIL) as the episode itself reported it."""
    pattern = re.compile(
        rf"^{re.escape(slug)} -> (?P<zone>[BCD]): (?P<verdict>PASS|FAIL) ")
    found = None
    for line in log.read_text(encoding="utf-8", errors="replace").splitlines():
        match = pattern.match(line)
        if match:
            found = (match.group("zone"), match.group("verdict"))
    if found is None:
        raise SystemExit(f"ABORT: no terminal verdict line in {log}")
    return found


def collect(clips: Path, logs: Path, matrix: Path, poses: int) -> list[Cell]:
    cells = []
    for slug, expect in parse_census_cells(matrix):
        for oi in range(poses):
            clip = clips / f"cell_{slug}_{oi}.mp4"
            log = logs / f"matrix_{slug}_{oi}.log"
            if not clip.is_file():
                raise SystemExit(f"ABORT: missing clip {clip}")
            if not log.is_file():
                raise SystemExit(f"ABORT: missing episode log {log}")
            scored, verdict = read_verdict(log, slug)
            # The two sources must agree about what this cell was scored against.
            # If they drift, the caption would state a zone the census never used.
            if scored != expect:
                raise SystemExit(
                    f"ABORT: {log.name} scored {slug} against {scored}, "
                    f"but run_matrix.sh lists {expect}")
            cells.append(Cell(slug, oi, expect, verdict, clip))
    return cells


def action_window(path: Path,
                  target: float = TARGET_SECONDS,
                  pad: float = PAD_SECONDS) -> tuple[float, float]:
    """(start, duration) of the stretch where the cell actually does something.

    Two obvious rules, both measured and both wrong (31.07, bottle probe):

    "Keep the last N seconds" — the 26.1 s cell shows the item from t=16.5 s to
    t=21 s and then plays 5 s of empty belt, so this ships clips of nothing.

    "Keep the window with the most motion" — that window is the WORLD BUILDING
    ITSELF. Gazebo spawns the belt, gantry and chutes while the spectator camera
    is already recording, and a 20 m belt appearing moves orders of magnitude more
    pixels than a 90 mm bottle crossing them. The first cut of this function
    trimmed the q1 bottle clip to 3.4-14.4 s: bringup, no item.

    So take the LAST burst of motion instead of the largest. Bringup is always at
    the head of the clip and the item transit is always the final event, so
    "last" separates them without a magic bringup duration to keep in sync.
    """
    capture = cv2.VideoCapture(str(path))
    fps = capture.get(cv2.CAP_PROP_FPS) or FPS
    motion, previous = [], None
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        small = cv2.cvtColor(cv2.resize(frame, (160, 90)), cv2.COLOR_BGR2GRAY)
        small = small.astype(np.float32)
        if previous is not None:
            motion.append(float(np.abs(small - previous).mean()))
        previous = small
    capture.release()
    duration = (len(motion) + 1) / fps
    if not motion:
        return 0.0, min(target, max(duration, 0.1))

    energy = np.asarray(motion)
    # Robust floor: renderer noise gives every frame a small non-zero diff, and a
    # mean-based threshold would be dragged up by the bringup spike it has to
    # ignore. Median + MAD is not moved by either.
    median = float(np.median(energy))
    deviation = float(np.median(np.abs(energy - median)))
    floor = median + max(QUIET_SIGMAS * deviation, ABSOLUTE_FLOOR)
    active = np.flatnonzero(energy > floor)
    if active.size == 0:
        return max(0.0, duration - target), min(target, duration)

    # Walk back from the last active frame through the burst it belongs to,
    # stepping over gaps shorter than GAP_SECONDS — an item crossing a shadow, or
    # the pause between the item settling and the blade parking, must not read as
    # two separate events.
    gap = max(1, int(round(GAP_SECONDS * fps)))
    end_index = int(active[-1])
    start_index = end_index
    for candidate in reversed(active[:-1].tolist()):
        if start_index - candidate > gap:
            break
        start_index = candidate

    start = max(0.0, start_index / fps - pad)
    end = min(duration, (end_index + 1) / fps + pad)
    # Pad a short window out to the target so every cell reads at the same pace.
    if end - start < target:
        slack = target - (end - start)
        start = max(0.0, start - slack / 2)
        end = min(duration, start + target)
        start = max(0.0, end - target)
    return start, min(end - start, target)


def caption_lines(cell: Cell, index: int, total: int, poses: int) -> tuple[str, str]:
    name = ITEMS[cell.slug][0] if cell.slug in ITEMS else cell.slug
    passed = cell.verdict == "PASS"
    # On FAIL the caption does NOT name where the item ended up: deciding that
    # would need a second ruler beside zone_verdict.py, and two rulers in one
    # project is how a census starts disagreeing with itself.
    landed = cell.expect if passed else "отказ"
    mark = "✓" if passed else "✗"
    return (f"{name} · поза {cell.oi + 1}/{poses}",
            f"{cell.expect} ожидалось → {landed}  {mark}  [{index}/{total}]")


def build(cells: list[Cell], out: Path, poses: int, dry_run: bool) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ffmpeg = find_ffmpeg()
    total = len(cells)
    plan_rows, elapsed = [], 0.0
    for index, cell in enumerate(cells, start=1):
        start, seconds = action_window(cell.clip)
        top, bottom = caption_lines(cell, index, total, poses)
        plan_rows.append((cell, start, seconds, top, bottom))
        print(f"{int(elapsed // 60):02d}:{int(elapsed % 60):02d}  {seconds:5.1f}s  "
              f"@{start:5.1f}s  {top} — {bottom}")
        elapsed += seconds
    passed = sum(1 for cell in cells if cell.verdict == "PASS")
    print(f"{int(elapsed // 60):02d}:{int(elapsed % 60):02d}  ИТОГО — "
          f"{passed}/{total} PASS")
    if dry_run:
        return 0

    font = _escaped_font(find_font())
    work = out.parent / "_census_parts"
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True)
    parts = []
    scale = (f"scale={WIDTH}:{HEIGHT}:force_original_aspect_ratio=decrease,"
             f"pad={WIDTH}:{HEIGHT}:(ow-iw)/2:(oh-ih)/2:color={BACKDROP}")

    for index, (cell, start, seconds, top, bottom) in enumerate(plan_rows):
        part = work / f"{index:02d}.mp4"
        top_file = _write_text(work / f"{index:02d}_t.txt", top)
        bottom_file = _write_text(work / f"{index:02d}_b.txt", bottom)
        draws = ",".join([
            (f"drawtext=textfile='{top_file}':fontfile='{font}':fontsize=30"
             f":fontcolor=white:box=1:boxcolor=0x0B1020@0.72:boxborderw=14"
             f":x=(w-text_w)/2:y=h-116"),
            (f"drawtext=textfile='{bottom_file}':fontfile='{font}':fontsize=30"
             f":fontcolor=0xE6E9F5:box=1:boxcolor=0x0B1020@0.72:boxborderw=14"
             f":x=(w-text_w)/2:y=h-64"),
        ])
        _render(ffmpeg, ["-ss", f"{start:.2f}", "-t", f"{seconds:.2f}",
                         "-i", str(cell.clip), "-vf", f"{scale},{draws}"], part)
        parts.append(part)

    listing = work / "parts.txt"
    listing.write_text(
        "\n".join(f"file '{part.name}'" for part in parts) + "\n", encoding="utf-8")
    subprocess.run([ffmpeg, "-y", "-hide_banner", "-loglevel", "error",
                    "-f", "concat", "-safe", "0", "-i", str(listing),
                    "-c", "copy", "-movflags", "+faststart", str(out)], check=True)
    shutil.rmtree(work)
    print(f"\n{out} — {out.stat().st_size / (1024 * 1024):.1f} MiB")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clips", type=Path, default=ROOT / "runs" / "census_video",
                        help="каталог с cell_<slug>_<oi>.mp4")
    parser.add_argument("--logs", type=Path,
                        default=ROOT / "runs" / "census_video_logs",
                        help="каталог с matrix_<slug>_<oi>.log")
    parser.add_argument("--out", type=Path,
                        default=ROOT / "docs" / "report" / "video" / "census_reel.mp4")
    parser.add_argument("--poses", type=int, default=3,
                        help="ориентаций на товар (N переписи)")
    parser.add_argument("--plan", action="store_true", help="печатать план, не собирая")
    args = parser.parse_args(argv)

    cells = collect(args.clips, args.logs, MATRIX, args.poses)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    return build(cells, args.out, args.poses, args.plan)


if __name__ == "__main__":
    raise SystemExit(main())
