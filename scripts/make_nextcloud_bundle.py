#!/usr/bin/env python3
"""Assemble the folder that gets uploaded to the organizers' Nextcloud.

The submission is split in two by the organizers' rules: the Git repository
carries code, documents and the small clips the deck plays, while bulky
material — the full video demonstration, source CAD, simulation assets and run
artifacts — goes to cloud storage with the links collected in README.md.

This script builds that upload folder from the working tree so the human step
is "drag one folder into Nextcloud, paste the link", not "remember what was
bulky". It copies, never moves, and writes a MANIFEST.md naming every file with
its size and what claim it backs.

    python scripts/make_nextcloud_bundle.py            # -> build/nextcloud/
    python scripts/make_nextcloud_bundle.py --dry-run  # just print the plan
"""
from __future__ import annotations

import argparse
import shutil
import sys
from dataclasses import dataclass, field
from pathlib import Path


@dataclass(frozen=True)
class Section:
    """One numbered folder of the upload, matching an organizer category."""

    folder: str
    title: str
    why: str
    # Glob patterns relative to the repository root. A pattern matching a
    # directory copies it whole.
    patterns: tuple[str, ...]
    # Missing sources are normally reported as gaps; optional ones are not,
    # because they are build artifacts a fresh checkout legitimately lacks.
    optional: frozenset[str] = field(default_factory=frozenset)


SECTIONS = (
    Section(
        folder="01-video-demo",
        title="Видеодемонстрация решения",
        why=(
            "Сквозной прогон и по клипу на каждый исход правила классификации. "
            "Эти же пять клипов лежат в репозитории — деке они нужны, чтобы "
            "играть из чистого клона; сюда они идут как полная демонстрация."
        ),
        patterns=(
            "docs/report/video/*.mp4",
            "docs/report/video/*_poster.png",
            "docs/report/video_script.md",
        ),
    ),
    Section(
        folder="02-cad-and-models",
        title="CAD-файлы и 3D-модели в исходных форматах",
        why=(
            "Наш эскиз ячейки в STEP/STL плюс размерный чертёж и изометрия, "
            "и исходные модели 11 товаров, выданные организаторами."
        ),
        patterns=(
            "docs/defense/cad/out",
            "docs/defense/cad/README.md",
            "docs/Step",
            "docs/Stl",
        ),
    ),
    Section(
        folder="03-simulation",
        title="Файлы симуляции",
        why=(
            "Миры Gazebo всех конфигураций стойки, конфиги моста ros_gz и "
            "сгенерированные SDF-модели товаров. Модели воспроизводятся "
            "командой build_item_models.py, но кладём готовые: проверяющему "
            "не придётся собирать их, чтобы открыть мир."
        ),
        patterns=(
            "sim/worlds",
            "sim/*.yaml",
            "sim/models",
        ),
        optional=frozenset({"sim/models"}),
    ),
    Section(
        folder="04-run-artifacts",
        title="Объёмные материалы: логи прогонов, из которых взяты цифры",
        why=(
            "Каждая заявленная цифра пересчитывается из этих логов одной "
            "командой measure_throughput.py — это и есть доказательство "
            "воспроизводимости, а не пересказ результата."
        ),
        patterns=(
            "runs/rig_throughput_20260728",
            "runs/stream_suite_20260717_123705_seed0",
            # Logs of the SHIPPED two-head rig (30.07). Without these the section
            # promised "the logs the numbers came from" while shipping only the
            # logs of superseded rigs: the census behind 33/33, the clean suite
            # behind 5/6 + 11 items/min + 0.040 s, and the gap diagnostic behind
            # the one acknowledged failure were all absent.
            "runs/census_prod2cam_seed0",
            "runs/throughput_prod2cam_clean",
            "runs/diag_gap18_oi1",
        ),
        optional=frozenset({
            "runs/rig_throughput_20260728",
            "runs/stream_suite_20260717_123705_seed0",
            "runs/census_prod2cam_seed0",
            "runs/throughput_prod2cam_clean",
            "runs/diag_gap18_oi1",
        }),
    ),
)

# Bulky files deliberately NOT bundled, with the reason. Printed by the script
# so the decision is visible instead of looking like an oversight.
EXCLUDED = (
    ("ozon_stream.mp4", "397 МБ сырой записи рабочей сессии — не артефакт сдачи; "
                        "если это нужный материал, добавить руками осознанно"),
    ("build/, install/, log/", "артефакты сборки colcon, воспроизводятся командой"),
)


# Дальше этого числа поимённый список перестаёт читаться и сворачивается
# в каталоги.
MANIFEST_FILE_ROWS = 30


def _human(size: int) -> str:
    if size >= 1024 * 1024:
        return f"{size / (1024 * 1024):.1f} МБ"
    return f"{size / 1024:.0f} КБ"


def _files_under(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    return sorted(p for p in path.rglob("*") if p.is_file())


def plan_bundle(root: Path) -> tuple[dict[str, list[Path]], list[str]]:
    """Return files per section folder plus the patterns that matched nothing."""
    plan: dict[str, list[Path]] = {}
    gaps: list[str] = []
    for section in SECTIONS:
        collected: list[Path] = []
        for pattern in section.patterns:
            # A pattern that resolves to an empty directory contributes nothing
            # and must be reported like an absent one, or the bundle ships a
            # named-but-empty folder.
            found = [file for match in sorted(root.glob(pattern))
                     for file in _files_under(match)]
            if not found and pattern not in section.optional:
                gaps.append(f"{section.folder}: не найдено — {pattern}")
            collected.extend(found)
        plan[section.folder] = collected
    return plan, gaps


def render_manifest(root: Path, plan: dict[str, list[Path]]) -> str:
    lines = [
        "# Что загружено в Nextcloud",
        "",
        "Собрано `scripts/make_nextcloud_bundle.py` из рабочего дерева репозитория.",
        "Репозиторий несёт код, документы и клипы деки; сюда уходит объёмное:",
        "видеодемонстрация, исходные CAD и 3D-модели, файлы симуляции и логи",
        "прогонов, из которых пересчитываются заявленные цифры.",
        "",
    ]
    total = 0
    for section in SECTIONS:
        files = plan.get(section.folder, [])
        size = sum(path.stat().st_size for path in files)
        total += size
        lines += [
            f"## {section.folder} — {section.title}",
            "",
            section.why,
            "",
            f"Файлов: {len(files)}, объём: {_human(size)}.",
            "",
        ]
        if files:
            # Логи прогонов — это сотни мелких файлов; поимённый список их не
            # описывает, поэтому длинные разделы сворачиваются в каталоги.
            lines += ["| файл | размер |", "|---|---|"]
            if len(files) <= MANIFEST_FILE_ROWS:
                rows = [(path.relative_to(root).as_posix(), path.stat().st_size)
                        for path in files]
            else:
                grouped: dict[str, int] = {}
                for path in files:
                    parent = path.parent.relative_to(root).as_posix()
                    grouped[parent] = grouped.get(parent, 0) + path.stat().st_size
                rows = [(f"{parent}/", size) for parent, size in sorted(grouped.items())]
            for name, size in rows:
                lines.append(f"| `{name}` | {_human(size)} |")
            lines.append("")
    lines += [
        f"**Итого: {_human(total)}.**",
        "",
        "## Чего здесь намеренно нет",
        "",
    ]
    lines += [f"- `{name}` — {reason}" for name, reason in EXCLUDED]
    lines.append("")
    return "\n".join(lines)


def build(root: Path, out: Path, dry_run: bool) -> int:
    # Console output stays English: Windows consoles mangle Cyrillic under the
    # default codepage, and this is tooling, not a document. MANIFEST.md, which
    # the jury actually reads, is Russian.
    plan, gaps = plan_bundle(root)
    total = 0
    for section in SECTIONS:
        files = plan[section.folder]
        size = sum(path.stat().st_size for path in files)
        total += size
        print(f"{section.folder:<18} {len(files):>4} files {size / (1024 * 1024):>8.1f} MiB")
    print(f"{'total':<18} {'':>4}       {total / (1024 * 1024):>8.1f} MiB")
    for gap in gaps:
        print(f"GAP: {gap}")

    if dry_run:
        return 1 if gaps else 0

    if out.exists():
        shutil.rmtree(out)
    for section in SECTIONS:
        for path in plan[section.folder]:
            target = out / section.folder / path.relative_to(root)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
    (out / "MANIFEST.md").write_text(render_manifest(root, plan), encoding="utf-8")

    print(f"\nbundle written to {out}")
    print("next, by hand: upload the folder to the organizers' Nextcloud, take the")
    print("share link, replace the placeholder in README.md, then")
    print("`python scripts/check_submission.py` -> PASS.")
    return 1 if gaps else 0


def main(argv: list[str] | None = None) -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=root / "build" / "nextcloud")
    parser.add_argument("--dry-run", action="store_true",
                        help="показать план и пробелы, ничего не копируя")
    args = parser.parse_args(argv)
    return build(root, args.out, args.dry_run)


if __name__ == "__main__":
    sys.exit(main())
