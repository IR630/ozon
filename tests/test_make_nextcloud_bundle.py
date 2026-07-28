from pathlib import Path

from scripts.make_nextcloud_bundle import (
    MANIFEST_FILE_ROWS,
    SECTIONS,
    build,
    plan_bundle,
    render_manifest,
)


ROOT = Path(__file__).resolve().parents[1]


def _fake_tree(root: Path) -> None:
    """A miniature of the repository holding one file per required pattern."""
    for relative in (
        "docs/report/video/hero.mp4",
        "docs/report/video/hero_poster.png",
        "docs/report/video_script.md",
        "docs/defense/cad/out/cell.step",
        "docs/defense/cad/README.md",
        "docs/Step/item.stp",
        "docs/Stl/item.stl",
        "sim/worlds/cell.sdf",
        "sim/bridge.yaml",
    ):
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


def test_real_repository_bundles_the_shipped_clips():
    # Regression guard: the demo the deck plays is also what the cloud gets.
    plan, _ = plan_bundle(ROOT)
    names = {path.name for path in plan["01-video-demo"]}
    assert {"hero_stream_mixed_cd.mp4", "rig3_bottle_D.mp4", "rig3_pen_C.mp4"} <= names
