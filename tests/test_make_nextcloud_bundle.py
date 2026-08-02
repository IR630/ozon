from pathlib import Path

from scripts.make_nextcloud_bundle import (
    MANIFEST_FILE_ROWS,
    NOT_BUNDLED,
    SECTIONS,
    build,
    plan_bundle,
    render_manifest,
)


ROOT = Path(__file__).resolve().parents[1]


def _fake_tree(root: Path, *, with_logs: bool = True) -> None:
    """A miniature of the repository holding one file per required pattern.

    ``with_logs=False`` reproduces a checkout that never ran anything: the log
    directories are build artifacts, so every source of 04-run-artifacts is
    optional and the section legitimately collects nothing.
    """
    relatives = [
        "docs/report/video/hero.mp4",
        "docs/report/video/hero_poster.png",
        "docs/report/video_script.md",
        "docs/defense/cad/out/cell.step",
        "docs/defense/cad/README.md",
        "docs/Step/item.stp",
        "docs/Stl/item.stl",
        "sim/worlds/cell.sdf",
        "sim/bridge.yaml",
    ]
    if with_logs:
        relatives.append("runs/census_prod2cam_seed0/cell.log")
    for relative in relatives:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"x" * 1024)


def test_plan_covers_every_section_and_reports_nothing_missing(tmp_path):
    _fake_tree(tmp_path)

    plan, gaps = plan_bundle(tmp_path)

    assert gaps == []
    assert set(plan) == {section.folder for section in SECTIONS}
    assert any(p.name == "hero.mp4" for p in plan["01-video-demo"])
    assert any(p.name == "item.stp" for p in plan["02-cad-and-models"])
    assert any(p.name == "cell.sdf" for p in plan["03-simulation"])


def test_missing_required_source_is_a_reported_gap_not_a_silent_skip(tmp_path):
    _fake_tree(tmp_path)
    (tmp_path / "docs" / "Step" / "item.stp").unlink()

    _, gaps = plan_bundle(tmp_path)

    assert gaps == ["02-cad-and-models: не найдено — docs/Step"]


def test_a_section_that_ends_up_empty_is_reported_even_when_every_source_is_optional(tmp_path):
    """A named-but-empty folder is the worst outcome: it looks delivered.

    Every pattern of ``04-run-artifacts`` is optional, because logs are build
    artifacts a fresh checkout legitimately lacks. Combined, that meant a machine
    without ``runs/`` produced a bundle whose "logs every number comes from"
    folder was EMPTY, with no gap reported and exit status 0 — while the MANIFEST
    kept promising those logs. Optional means "this one source may be absent", not
    "this whole section may quietly vanish".
    """
    _fake_tree(tmp_path, with_logs=False)

    _, gaps = plan_bundle(tmp_path)

    assert any(gap.startswith("04-run-artifacts:") for gap in gaps), gaps


def test_explicit_without_runs_mode_builds_base_bundle_and_documents_omission(tmp_path):
    source = tmp_path / "repo"
    source.mkdir()
    _fake_tree(source, with_logs=False)
    out = tmp_path / "bundle"

    plan, gaps = plan_bundle(source, include_runs=False)

    assert gaps == []
    assert "04-run-artifacts" not in plan
    assert build(source, out, dry_run=False, include_runs=False) == 0
    manifest = (out / "MANIFEST.md").read_text(encoding="utf-8")
    assert "`04-run-artifacts` не включён" in manifest
    assert "`--without-runs`" in manifest


def test_build_copies_originals_and_writes_a_manifest(tmp_path):
    source = tmp_path / "repo"
    source.mkdir()
    _fake_tree(source)
    out = tmp_path / "bundle"

    assert build(source, out, dry_run=False) == 0

    assert (out / "01-video-demo" / "docs" / "report" / "video" / "hero.mp4").is_file()
    # Copied, never moved: the working tree must survive the packaging step.
    assert (source / "docs" / "report" / "video" / "hero.mp4").is_file()
    manifest = (out / "MANIFEST.md").read_text(encoding="utf-8")
    assert "Видеодемонстрация решения" in manifest
    assert "ozon_stream.mp4" in manifest  # the deliberate exclusion is named


def test_dry_run_writes_nothing(tmp_path):
    source = tmp_path / "repo"
    source.mkdir()
    _fake_tree(source)
    out = tmp_path / "bundle"

    assert build(source, out, dry_run=True) == 0
    assert not out.exists()


def test_long_sections_collapse_to_directories(tmp_path):
    _fake_tree(tmp_path)
    logs = tmp_path / "runs" / "rig_throughput_20260728" / "3cam"
    logs.mkdir(parents=True)
    for index in range(MANIFEST_FILE_ROWS + 5):
        (logs / f"episode_{index}.log").write_bytes(b"x" * 512)

    plan, _ = plan_bundle(tmp_path)
    manifest = render_manifest(tmp_path, plan)

    assert "| `runs/rig_throughput_20260728/3cam/` |" in manifest
    assert "episode_0.log" not in manifest


def test_real_repository_bundles_every_clip_the_deck_plays():
    """Regression guard: the demo the deck plays is also what the cloud gets.

    DERIVED FROM THE DECK, not a hardcoded list. The previous version pinned
    `rig3_*` by name, and that is exactly how this drifted twice: on 28.07 the
    shipped rig became three heads while the clips were single-head, and on 30.07
    it became two heads while the clips were three-head. A list written by hand
    goes stale silently; the deck's own <video src> cannot.
    """
    import re

    deck = (ROOT / "docs" / "report" / "slides" / "deck-c-ozon.html").read_text(encoding="utf-8")
    played = {Path(src).name for src in re.findall(r'<video[^>]+src="([^"]+\.mp4)"', deck)}
    assert played, "the deck plays no clips at all — the gallery is empty"

    plan, _ = plan_bundle(ROOT)
    bundled = {path.name for path in plan["01-video-demo"]}
    missing = played - bundled
    assert not missing, f"the deck plays clips the bundle does not carry: {sorted(missing)}"


def test_working_tree_leftovers_do_not_ride_along_into_the_bundle():
    """A file only the author has must not decide what the jury receives.

    `docs/report/video/*.mp4` is deliberately broad, so anything sitting in that
    folder ships. `demo_reel.mp4` is the case that made this concrete: it is not
    in git, it is documented as working material rather than a submission
    artifact, and the copy on the author's disk predates the 31.07 number fixes —
    so bundling it would have put a superseded figure in front of the jury on one
    machine and nothing at all on any other.
    """
    plan, _ = plan_bundle(ROOT)
    bundled = {path.relative_to(ROOT).as_posix() for path in plan["01-video-demo"]}
    assert not (bundled & NOT_BUNDLED), (
        f"excluded files reached the bundle: {sorted(bundled & NOT_BUNDLED)}")


def test_exclusion_is_enforced_and_not_merely_documented(tmp_path):
    """The rationale list is printed, not applied — the guard must be the set.

    Written against a tree where the excluded file is the ONLY thing the pattern
    could match, so a filter that silently does nothing cannot pass.
    """
    _fake_tree(tmp_path)
    video = tmp_path / "docs" / "report" / "video"
    for stray in video.glob("*.mp4"):
        stray.unlink()
    (video / "demo_reel.mp4").write_bytes(b"stale reel")

    plan, gaps = plan_bundle(tmp_path)
    names = {path.name for path in plan["01-video-demo"]}
    assert "demo_reel.mp4" not in names
    assert any("*.mp4" in gap for gap in gaps), (
        "a pattern left with nothing but excluded files must be reported as a gap")


def test_old_local_cycle_clip_and_poster_do_not_leak_into_bundle(tmp_path):
    _fake_tree(tmp_path)
    video = tmp_path / "docs" / "report" / "video"
    (video / "demo_sorting_cycle_C_short_20260713.mp4").write_bytes(b"old")
    (video / "demo_sorting_cycle_C_short_20260713_poster.png").write_bytes(b"old")

    plan, gaps = plan_bundle(tmp_path)
    bundled = {path.name for path in plan["01-video-demo"]}

    assert gaps == []
    assert "demo_sorting_cycle_C_short_20260713.mp4" not in bundled
    assert "demo_sorting_cycle_C_short_20260713_poster.png" not in bundled
