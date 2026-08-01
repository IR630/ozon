#!/usr/bin/env python3
"""Fast repository-only preflight for the final submission package.

This does not replace ``check_clean_deploy.sh`` or a trial upload. It catches
cheap packaging failures before those expensive human gates: missing artifacts,
known placeholders and accidentally tracked large binaries.
"""
from __future__ import annotations

import importlib.util
import re
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
    # The shipped rig since 30.07 is the TWO-head one; the preflight has to guard
    # the configuration that actually runs, not only the single-head fallback.
    # The three-head world stays in the tree as a measured alternative the report
    # compares against, so it is guarded too.
    "sim/worlds/cell_diverter_2cam.sdf",
    "sim/bridge_2cam.yaml",
    # The shipped video demonstration itself (31.07). The task statement makes a
    # video demonstration mandatory, and this one lives in the repository rather
    # than only behind the cloud link — so a jury that never opens the link still
    # sees the solution work. If it ever vanishes from the tree, the submission is
    # incomplete, and the preflight has to say so rather than leave it to memory.
    "docs/report/video/census_reel.mp4",
    "sim/worlds/cell_diverter_3cam.sdf",
    "sim/bridge_3cam.yaml",
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
    "docs/report/slides/deck-c-ozon.html",
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
    # The census reel (13.6 MiB, 31.07) is the second deliberate exception and it
    # is named here rather than waived by raising the limit. It is the one artifact
    # that shows all 33 cells being sorted, and it is worth its size in the tree for
    # the same reason the deck's gallery clips are: a jury that never opens the
    # cloud link still sees the solution work. Raising MAX_TRACKED_BINARY_BYTES
    # instead would silently admit the next 14 MiB file nobody argued for.
    "docs/report/video/census_reel.mp4",
}
ALLOWED_LARGE_PREFIXES = ("docs/Step/", "docs/Stl/")
VIDEO_LABEL = "**Видеодемонстрация:**"

# Local markdown cross-references in the submission docs must resolve — a
# navigable kit (criteria section 7) cannot have dead links after a file is
# renamed or moved. External and pure-anchor links are out of scope here.
_MD_LINK = re.compile(r"\[[^\]]*\]\(([^)]+)\)")
_LINK_SCAN_DIRS = ("docs/report", "docs/defense")

# The deck is the presentation the jury opens; it has to work from a clean clone
# with no network. Two ways it silently degrades to placeholders, both seen in
# this repository: a clip that is gitignored (file absent) and a clip written by
# OpenCV in MPEG-4 part 2, which every browser refuses to decode. An mp4 whose
# sample entry is ``avc1`` is H.264; ``mp4v`` is the unplayable kind.
_HTML_MEDIA = re.compile(r'(?:src|poster)="([^"]+)"')
_DECK_GLOB = "docs/report/slides/deck-*.html"
_H264_SAMPLE_ENTRY = b"avc1"
# The four decks are one deck in four styles; only the one named here is
# presented, so only it must carry the demo gallery. The other three keep the
# single hero clip and are checked for broken media only.
SHIPPED_DECK = "docs/report/slides/deck-c-ozon.html"
MIN_DECK_VIDEOS = 3

# Where the reel's footage lives, and the plan that says which of it is needed.
_VIDEO_DIR = "docs/report/video"
_REEL_PLAN = "scripts/build_demo_reel.py"

# Media named in the defence plan, and the directories the report keeps media in.
_OUTLINE_MEDIA = re.compile(r"`([A-Za-z0-9_*.-]+\.(?:mp4|png))`")
_MEDIA_DIRS = (_VIDEO_DIR, "docs/report/img", f"{_VIDEO_DIR}/inserts")


def reel_clip_issues(clips, tracked: set[str] | None) -> list[str]:
    """Clips the reel plan needs that the repository does not carry.

    The deck already learned this lesson (see ``deck_media_issues``); the reel had
    the same hole and nothing watching it. ``build_demo_reel.py`` aborts on a
    missing clip, so on the author's disk — where every clip sits — the reel builds
    and looks finished, while a clean clone cannot rebuild the video demonstration
    the task statement requires at all.

    ``tracked`` follows the deck check's convention: an empty or absent inventory
    means "no usable git index here" (the colcon container), not "nothing is
    tracked", so the check stays quiet rather than condemning every clip.
    """
    if not tracked:
        return []
    return [f"UNTRACKED_REEL_CLIP: {name}" for name in clips
            if f"{_VIDEO_DIR}/{name}" not in tracked]


def outline_media_issues(root: Path, tracked: set[str] | None) -> list[str]:
    """Media the defence plan names on stage that the repository does not carry.

    Same failure as the deck's and the reel's, third venue. `presentation_outline.md`
    listed `hero_diverter_bottle_D.mp4` as the standby clip for slide 6 and that file
    was never committed — and nothing noticed, because the deck guard reads the HTML
    decks while the outline is markdown no player ever opens. A clip that is missing
    is found out on stage, which is the worst possible moment.

    Names are matched inside backticks and looked up in the directories the report
    keeps media in; a glob like `rig2_*` is a family, not a file, and is skipped.
    """
    if not tracked:
        return []
    outline = root / "docs" / "report" / "presentation_outline.md"
    if not outline.is_file():
        return []
    text = outline.read_text(encoding="utf-8", errors="replace")
    issues = []
    for name in sorted(set(_OUTLINE_MEDIA.findall(text))):
        if "*" in name:
            continue
        if not any(f"{directory}/{name}" in tracked for directory in _MEDIA_DIRS):
            issues.append(f"OUTLINE_MEDIA: {name}")
    return issues


def reel_clip_names(root: Path) -> list[str]:
    """Clip names the reel references, read from the reel's own plan.

    Loaded from the file rather than copied here on purpose: a hand-kept second
    list is precisely how the preflight's media inventory and the deck's gallery
    drifted apart on 30.07, and this guard exists to stop that repeating.
    Returns an empty list if the plan cannot be read — a preflight must report
    problems, not become one.
    """
    plan_path = root / _REEL_PLAN
    if not plan_path.is_file():
        return []
    spec = importlib.util.spec_from_file_location("_reel_plan", plan_path)
    if spec is None or spec.loader is None:
        return []
    module = importlib.util.module_from_spec(spec)
    # Register BEFORE executing: @dataclass looks its own module up in sys.modules
    # while building the class, and on a module that is absent there it raises
    # AttributeError on None.__dict__ — which this function would then swallow into
    # an empty list, quietly disarming the check.
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        return []
    finally:
        sys.modules.pop(spec.name, None)
    return [segment.name for segment in getattr(module, "SEGMENTS", ())
            if type(segment).__name__ == "Clip"]


def deck_media_issues(root: Path, tracked: set[str] | None = None) -> list[str]:
    """Local media referenced by the decks that would not render.

    Missing files, mp4s in a codec browsers refuse, media that is present locally
    but absent from the repository, and a shipped deck carrying fewer than
    ``MIN_DECK_VIDEOS`` playable clips.

    ``tracked`` is the set of git-tracked paths. Pass it: existence on the
    author's disk proves nothing about the jury's clone, and that gap already
    shipped once — `.gitignore` re-included the retired three-head gallery while
    the deck referenced the two-head one, so four of five gallery clips were
    missing from the repository while every disk check stayed green.
    """
    issues = []
    for deck in sorted(root.glob(_DECK_GLOB)):
        playable = 0
        for target in _HTML_MEDIA.findall(deck.read_text(encoding="utf-8", errors="replace")):
            if target.startswith(("http://", "https://", "data:", "#")):
                continue
            path = (deck.parent / target.split("#", 1)[0]).resolve()
            name = f"{deck.relative_to(root).as_posix()}: {target}"
            if not path.is_file():
                issues.append(f"MISSING_MEDIA: {name}")
                continue
            if tracked is not None:
                try:
                    relative_media = path.relative_to(root.resolve()).as_posix()
                except ValueError:
                    relative_media = None
                if relative_media is not None and relative_media not in tracked:
                    issues.append(f"UNTRACKED_MEDIA: {name}")
                    continue
            if path.suffix.lower() == ".mp4":
                if _H264_SAMPLE_ENTRY in path.read_bytes():
                    playable += 1
                else:
                    issues.append(f"UNPLAYABLE_MEDIA: {name}: not H.264")
        relative = deck.relative_to(root).as_posix()
        if relative == SHIPPED_DECK and playable < MIN_DECK_VIDEOS:
            issues.append(
                f"DECK_VIDEOS: {relative}: "
                f"{playable} playable clips, need {MIN_DECK_VIDEOS}"
            )
    return issues


def tracked_files(root: Path) -> list[str]:
    """Paths git tracks under ``root``; empty when git cannot answer.

    NOT ``check=True``. Every caller is built around "an empty inventory means no
    usable git index here, so do not condemn every file" — raising instead made
    that contract unreachable and cost real green builds: in the colcon container
    the workspace trips git's dubious-ownership guard, ``git ls-files`` exits 128,
    and three tests failed on every push from 28.07 on for a reason that had
    nothing to do with the submission. The same traceback would meet an organizer
    who unpacks the archive without ``.git`` and runs the preflight.
    """
    try:
        result = subprocess.run(
            ["git", "ls-files", "-z"],
            cwd=root,
            check=False,
            capture_output=True,
        )
    except OSError:                      # git not installed at all
        return []
    if result.returncode != 0:
        return []
    return [part.decode("utf-8") for part in result.stdout.split(b"\0") if part]


def has_https_video_link(readme: str) -> bool:
    """Return whether the video paragraph contains a usable cloud URL."""
    _, label, remainder = readme.partition(VIDEO_LABEL)
    if not label:
        return False
    paragraph = remainder.split("\n\n", maxsplit=1)[0]
    return "https://" in paragraph


def broken_local_links(root: Path) -> list[str]:
    """Local markdown links in the submission docs that do not resolve to a file.

    Catches a renamed or moved artifact leaving a dead cross-reference behind,
    before the trial upload does. README plus every doc under docs/report and
    docs/defense is scanned; http(s), mailto and pure-anchor targets are skipped.
    """
    docs = [root / "README.md"]
    for sub in _LINK_SCAN_DIRS:
        docs.extend(sorted((root / sub).glob("*.md")))
    issues = []
    for doc in docs:
        if not doc.is_file():
            continue
        for target in _MD_LINK.findall(doc.read_text(encoding="utf-8", errors="replace")):
            if target.startswith(("http://", "https://", "#", "mailto:")):
                continue
            path = target.split("#", 1)[0].strip()
            if not path:
                continue
            if not (doc.parent / path).resolve().is_file():
                relative = doc.relative_to(root).as_posix()
                issues.append(f"BROKEN_LINK: {relative}: {target}")
    return issues


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

    issues.extend(broken_local_links(root))
    # An EMPTY inventory means "no usable git index here" (the colcon container),
    # not "nothing is tracked" — treating it as the latter would report every
    # deck clip as missing from the repository. Only check tracking when we
    # actually have an inventory to check against.
    tracked_paths = set(tracked if tracked is not None else tracked_files(root))
    issues.extend(deck_media_issues(
        root, tracked=tracked_paths if tracked_paths else None))
    issues.extend(outline_media_issues(
        root, tracked=tracked_paths if tracked_paths else None))
    # NOT reel_clip_issues() here any more. That guard watches the narrative reel
    # (build_demo_reel.py), and since 31.07 the shipped video demonstration is
    # census_reel.mp4 — the narrative reel is not an artifact of the submission, so
    # its missing source clips must not block one. The guard itself stays alive and
    # tested: if the narrative reel is ever brought back, it is the thing that
    # notices its footage never reached the repository. What the preflight checks
    # instead is that the reel we DO ship is present, and that is REQUIRED_FILES.
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
