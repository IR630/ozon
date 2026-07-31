#!/usr/bin/env python3
"""Assemble the submission video demonstration from the recorded clips.

The organizers require a video demonstration of the working solution
(`docs/md/task.md`), and list what the jury must be able to check in it:
how the category is determined, how all three categories are handled, how the
verdict reaches the actuator, how the item is physically redirected, behaviour
on the test set, the timing constraints, and edge cases. This script lays the
existing footage out against that checklist and burns in the captions, so the
reel reads on its own and a voice-over can be laid on top later.

    python scripts/build_demo_reel.py                 # -> docs/report/video/demo_reel.mp4
    python scripts/build_demo_reel.py --plan          # print the timeline only

Every segment is re-encoded to one format first, then concatenated by stream
copy: mixing 800x450 clips with 1920x1080 stills in one filter graph is where
this kind of script usually goes wrong silently.
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

WIDTH, HEIGHT, FPS = 1280, 720, 25
FONT_CANDIDATES = (
    "C:/Windows/Fonts/arial.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
)
BACKDROP = "0x0B1020"


@dataclass(frozen=True)
class Card:
    """A full-screen text card: section titles, rules, closing lines."""

    lines: tuple[str, ...]
    seconds: float
    size: int = 44


@dataclass(frozen=True)
class Clip:
    """Recorded footage. ``seconds`` trims from the start; None keeps it whole."""

    name: str
    caption: str
    seconds: float | None = None


@dataclass(frozen=True)
class Still:
    """A diagram or metrics card held on screen."""

    path: str
    caption: str
    seconds: float


# The reel follows the jury's checklist in order, so a reviewer can tick items
# off as they watch instead of hunting for them.
SEGMENTS: tuple[Card | Clip | Still, ...] = (
    # "О команде" is item 1 of the recommended structure, so it belongs on the
    # title card. Names are split across two lines on purpose: one line of five
    # would run past the frame at this size, and drawtext does not wrap.
    Card((
        "Роботизированная сортировка товаров",
        "Трек 3 · виртуальная ячейка пресортировки",
        "",
        "Команда «ИИ в массы»",
        "Иван Пушкин · Иван Резников · Максим Седов",
        "Михаил Анцев · Владимир Зайцев",
        "",
        "Видеодемонстрация работы решения",
    ), 9.0, size=40),

    Card((
        "Задача",
        "",
        "Лента 1 м/с идёт без остановки.",
        "Товар нужно распознать на ходу и физически развести",
        "по трём зонам: B, C и D.",
    ), 11.0),
    Still("docs/report/img/official_layout.png",
          "Участок 6×10 м: подача A, зоны B / C / D", 13.0),

    Card((
        "Правило классификации",
        "",
        "B — проходит основной сортировщик",
        "C — не проходит по габаритам",
        "D — круг в сечении, нужна доупаковка",
        "",
        "габариты 10×10×10 … 450×320×320 мм",
        "K = r_впис / R_опис > 0.8",
        "габарит важнее формы",
    ), 16.0, size=38),
    Still("docs/report/img/day9_overlay_ids_state.png",
          "Depth-кадр: маска, item_id и измеренные габариты — цвет не используется",
          12.0),

    Card((
        "Контур целиком",
        "",
        "depth-стойка → перцепция (item_id, габариты, K)",
        "→ классификатор (B/C/D) → контроллер",
        "→ поворотный шибер",
        "",
        "Вердикт напрямую становится управляющим сигналом:",
        "это один ПАК, а не два отдельных компонента.",
    ), 15.0, size=38),

    Card(("Сквозной поток", "", "Пять товаров подряд, лента не останавливается"), 6.0),
    Clip("hero_stream_mixed_cd.mp4",
         "Сквозной поток 5/5: бутылка→D, тарелка→D, короб→C, пуфик→C, ручка→C"),

    Card((
        "Правило целиком — по клипу на каждый исход",
        "",
        "Отгружаемая стойка: верхняя головка + боковой борт",
    ), 7.0),
    Clip("rig2_bottle_D.mp4", "Бутылка → D: круг в сечении, K = 0,996"),
    Clip("rig2_box_300x200x200_B.mp4", "Короб 300×200×200 → B: в габарите, идёт как есть"),
    Clip("rig2_box_400x400x300_C.mp4", "Короб 400×400×300 → C: 400 мм против ворот 320"),
    Clip("rig2_pen_C.mp4", "Ручка → C: габарит бьёт форму (9 мм показаны на depth-кадре выше)"),

    Card((
        "Ловушка, на которой ломается классификация по имени",
        "",
        "Цилиндр: K ≈ 0,75 < 0,8",
        "четыре продольные стяжки делают сечение НЕкруглым",
        "",
        "имя круглое — сечение нет → B",
    ), 11.0, size=38),
    Clip("hero_diverter_cylinder_B.mp4", "Цилиндр → B: проходит мимо лопастей нетронутым"),

    Card((
        "Исполнительная часть",
        "",
        "Шибер, а не толкатель: товар уходит СО СКОРОСТЬЮ ЛЕНТЫ,",
        "без бокового удара 2,5 м/с.",
        "",
        "На тяжёлом коробе шибер ≥1,3× мягче по пиковому ускорению,",
        "точечно до 3×.",
        "",
        "Полный цикл: сигнал → стенка за 0,5 с до входа → склиз → возврат.",
    ), 16.0, size=34),
    Clip("hero_diverter_box400_C.mp4",
         "Полный цикл механизма: формирование стенки, склиз, возврат"),

    Still("docs/report/video/inserts/sync_timeline.png",
          "Синхронизация: упреждение срабатывает с запасом", 13.0),

    Card((
        "Подтверждение числом",
        "",
        "классификация и маршрутизация — 33/33, под шумом 3 мм — 32/33",
        "пропускная способность отгружаемой стойки — 9 товаров/мин",
        "камера → решение — медиана 0,042 с",
        "расчёт ↔ симуляция — 5 величин сведены",
        "",
        "Каждая цифра воспроизводится одной командой и seed.",
    ), 17.0, size=34),
    Still("docs/report/video/inserts/metrics_card.png",
          "Доказано числом", 12.0),

    Card((
        "Нештатные ситуации",
        "",
        "E-stop замораживает лопасть на месте и разжимает её",
        "только по обратной связи, а не по таймеру.",
        "Затор и потеря подачи детектируются, зоны разделены.",
    ), 12.0, size=38),

    Card((
        "Честные границы",
        "",
        "9 товаров/мин — устойчивый режим, не пик; 18 — короткий пиковый поток.",
        "Плотный поток в одну зону: эпизод 2/6, товары сходят вбок —",
        "классификация верна, отказ исполнительный, причина не найдена.",
        "Массы, трение и rigid-body — инженерные допущения.",
        "Физический прототип — осознанный NO-GO: по условию достаточно ПАК в симуляции.",
        "Шум глубины, блики и дрейф калибровки стенд не моделирует —",
        "эти отказы обоснованы расчётом, а не замером.",
    ), 18.0, size=31),

    Card((
        "Воспроизводимость",
        "",
        "docker compose -f docker/docker-compose.yml run dev",
        "bash scripts/run_stream_suite.sh 0 0 2",
        "",
        "Вся случайность — через один seed.",
    ), 12.0, size=38),
)


def find_ffmpeg() -> str:
    for candidate in ("static_ffmpeg", "ffmpeg"):
        found = shutil.which(candidate)
        if found:
            return found
    local = Path(__file__).resolve().parent / "static_ffmpeg.exe"
    if local.is_file():
        return str(local)
    raise SystemExit("ABORT: ffmpeg not found (install it or put static_ffmpeg on PATH)")


def find_font() -> str:
    for candidate in FONT_CANDIDATES:
        if Path(candidate).is_file():
            return candidate
    raise SystemExit(f"ABORT: no usable font found, tried {FONT_CANDIDATES}")


def _write_text(path: Path, line: str) -> str:
    """Write one caption line for drawtext and return its escaped path.

    ``newline=""`` matters: Python would otherwise translate to CRLF on Windows
    and drawtext renders the stray CR as a missing-glyph box at every line end.
    """
    path.write_text(line, encoding="utf-8", newline="")
    return _escaped_font(str(path))


def _escaped_font(font: str) -> str:
    # drawtext parses ':' as its own separator, so a Windows drive letter has to
    # be escaped or the filter silently becomes unparseable.
    return font.replace("\\", "/").replace(":", r"\:")


def clip_duration(ffmpeg: str, path: Path) -> float:
    probe = Path(ffmpeg).with_name(Path(ffmpeg).name.replace("ffmpeg", "ffprobe"))
    result = subprocess.run(
        [str(probe), "-v", "error", "-show_entries", "format=duration",
         "-of", "default=noprint_wrappers=1:nokey=1", str(path)],
        capture_output=True, text=True, check=True,
    )
    return float(result.stdout.strip())


def plan(root: Path, ffmpeg: str) -> list[tuple[str, float, str]]:
    """Return (kind, seconds, label) per segment, resolving clip durations."""
    rows = []
    for segment in SEGMENTS:
        if isinstance(segment, Card):
            rows.append(("card", segment.seconds, segment.lines[0]))
        elif isinstance(segment, Still):
            rows.append(("still", segment.seconds, segment.caption))
        else:
            path = root / "docs" / "report" / "video" / segment.name
            if not path.is_file():
                raise SystemExit(f"ABORT: missing clip {path}")
            seconds = segment.seconds or clip_duration(ffmpeg, path)
            rows.append(("clip", seconds, segment.caption))
    return rows


def _render(ffmpeg: str, args: list[str], out: Path) -> None:
    subprocess.run([ffmpeg, "-y", "-hide_banner", "-loglevel", "error", *args,
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "21",
                    "-preset", "medium", "-r", str(FPS), "-an", str(out)],
                   check=True)


def build(root: Path, out: Path, dry_run: bool) -> int:
    # The plan lists Russian captions; a Windows console defaults to cp1251 and
    # dies on '×' alone. Reconfigure rather than transliterate the captions.
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ffmpeg = find_ffmpeg()
    rows = plan(root, ffmpeg)
    elapsed = 0.0
    for (kind, seconds, label) in rows:
        print(f"{int(elapsed // 60):02d}:{int(elapsed % 60):02d}  {seconds:5.1f}s  "
              f"{kind:<5}  {label}")
        elapsed += seconds
    print(f"{int(elapsed // 60):02d}:{int(elapsed % 60):02d}  ИТОГО")
    if dry_run:
        return 0

    font = _escaped_font(find_font())
    work = out.parent / "_reel_parts"
    if work.exists():
        shutil.rmtree(work)
    work.mkdir(parents=True)
    parts = []

    for index, segment in enumerate(SEGMENTS):
        part = work / f"{index:02d}.mp4"
        if isinstance(segment, Card):
            # One drawtext per line: a single multi-line drawtext centres the
            # BLOCK and leaves the lines ragged-left, which reads as a bug on a
            # title card. Per-line also lets the first line be the heading.
            line_height = segment.size * 1.5
            top = (HEIGHT - line_height * len(segment.lines)) / 2
            draws = []
            for number, line in enumerate(segment.lines):
                if not line:
                    continue
                text = _write_text(work / f"{index:02d}_{number}.txt", line)
                size = segment.size + 8 if number == 0 else segment.size
                colour = "white" if number == 0 else "0xE6E9F5"
                draws.append(
                    f"drawtext=textfile='{text}':fontfile='{font}':fontsize={size}"
                    f":fontcolor={colour}:x=(w-text_w)/2"
                    f":y={top + line_height * number:.0f}")
            _render(ffmpeg, ["-f", "lavfi", "-i",
                             f"color=c={BACKDROP}:s={WIDTH}x{HEIGHT}:d={segment.seconds}",
                             "-vf", ",".join(draws)], part)
        else:
            if isinstance(segment, Still):
                source = root / segment.path
                head = ["-loop", "1", "-t", str(segment.seconds), "-i", str(source)]
            else:
                source = root / "docs" / "report" / "video" / segment.name
                head = ["-i", str(source)]
                if segment.seconds:
                    head = ["-t", str(segment.seconds), *head]
            if not source.is_file():
                raise SystemExit(f"ABORT: missing asset {source}")
            caption = _write_text(work / f"{index:02d}.txt", segment.caption)
            scale = (f"scale={WIDTH}:{HEIGHT}:force_original_aspect_ratio=decrease,"
                     f"pad={WIDTH}:{HEIGHT}:(ow-iw)/2:(oh-ih)/2:color={BACKDROP}")
            draw = (f"drawtext=textfile='{caption}'"
                    f":fontfile='{font}':fontsize=28:fontcolor=white"
                    f":box=1:boxcolor=0x0B1020@0.72:boxborderw=16"
                    f":x=(w-text_w)/2:y=h-84")
            _render(ffmpeg, [*head, "-vf", f"{scale},{draw}"], part)
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
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path,
                        default=root / "docs" / "report" / "video" / "demo_reel.mp4")
    parser.add_argument("--plan", action="store_true", help="печатать план, не собирая")
    args = parser.parse_args(argv)
    return build(root, args.out, args.plan)


if __name__ == "__main__":
    sys.exit(main())
