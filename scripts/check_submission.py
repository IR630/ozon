#!/usr/bin/env python3
"""Fast repository-only preflight for the final submission package.

This does not replace ``check_clean_deploy.sh`` or a trial upload. It catches
cheap packaging failures before those expensive human gates: missing artifacts,
known placeholders and accidentally tracked large binaries.
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path


REQUIRED_FILES = (
    "README.md",
    "src/constants.py",
    "src/perception.py",
    "src/classification.py",
    "src/controller_node.py",
    "ros_msgs/msg/ItemMeasurement.msg",
    "ros_msgs/msg/ItemClassification.msg",
    "launch/skeleton.launch.py",
    "sim/worlds/cell_diverter.sdf",
    "scripts/run_skeleton.sh",
    "docs/report/final_report.md",
    "docs/report/one_pager.md",
    "docs/report/classification.md",
    "docs/report/mechanism.md",
    "docs/report/architecture.md",
    "docs/report/methodology_and_limitations.md",
    "docs/report/criteria_coverage.md",
    "docs/report/presentation_outline.md",
    "docs/report/video/hero_stream_mixed_cd.mp4",
    "docs/defense/cad/out/cell_sideview.step",
    "docs/defense/cad/out/cell_sideview.stl",
    "docker/Dockerfile",
    "docker/docker-compose.yml",
    "requirements.txt",
    ".github/workflows/ci.yml",
    "scripts/check_clean_deploy.sh",
)
REQUIRED_GLOBS = ("docs/report/slides/deck-*.html",)
PLACEHOLDERS = {
    "README.md": ("<ссылка на облако организаторов>",),
}
MAX_TRACKED_BINARY_BYTES = 5 * 1024 * 1024
ALLOWED_LARGE_FILES = {
    # Original organizer materials are the explicit exception to the repository's
    # no-large-binaries rule (GIT.md); generated videos/weights are not.
    "docs/task.pdf",
    "docs/criteries.pdf",
    "docs/software.pdf",
}
ALLOWED_LARGE_PREFIXES = ("docs/Step/", "docs/Stl/")
VIDEO_LABEL = "**Видеодемонстрация:**"


def tracked_files(root: Path) -> list[str]:
    result = subprocess.run(
        ["git", "ls-files", "-z"],
        cwd=root,
        check=True,
        capture_output=True,
    )
    return [part.decode("utf-8") for part in result.stdout.split(b"\0") if part]


def has_https_video_link(readme: str) -> bool:
    """Return whether the video paragraph contains a usable cloud URL."""
    _, label, remainder = readme.partition(VIDEO_LABEL)
    if not label:
        return False
    paragraph = remainder.split("\n\n", maxsplit=1)[0]
    return "https://" in paragraph


def submission_issues(root: Path, tracked: list[str] | None = None) -> list[str]:
    issues = []
    for relative in REQUIRED_FILES:
        if not (root / relative).is_file():
            issues.append(f"MISSING: {relative}")
    for pattern in REQUIRED_GLOBS:
        if not any(root.glob(pattern)):
            issues.append(f"MISSING: {pattern}")

    placeholder_files = set()
    for relative, markers in PLACEHOLDERS.items():
        path = root / relative
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8")
        for marker in markers:
            if marker in text:
                issues.append(f"PLACEHOLDER: {relative}: {marker}")
                placeholder_files.add(relative)

    readme_path = root / "README.md"
    if readme_path.is_file() and "README.md" not in placeholder_files:
        readme = readme_path.read_text(encoding="utf-8")
        if not has_https_video_link(readme):
            issues.append("CLOUD_LINK: README.md: missing HTTPS video URL")

    for relative in tracked if tracked is not None else tracked_files(root):
        path = root / relative
        allowed = (relative in ALLOWED_LARGE_FILES
                   or relative.startswith(ALLOWED_LARGE_PREFIXES))
        if (not allowed and path.is_file()
                and path.stat().st_size > MAX_TRACKED_BINARY_BYTES):
            size_mib = path.stat().st_size / (1024 * 1024)
            issues.append(f"LARGE: {relative}: {size_mib:.1f} MiB")
    return issues


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    issues = submission_issues(root)
    if issues:
        print("SUBMISSION PREFLIGHT: BLOCKED")
        for issue in issues:
            print(f"- {issue}")
        return 1
    print("SUBMISSION PREFLIGHT: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
