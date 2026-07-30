from pathlib import Path

from scripts.check_submission import (
    MAX_TRACKED_BINARY_BYTES,
    MIN_DECK_VIDEOS,
    SHIPPED_DECK,
    broken_local_links,
    deck_media_issues,
    has_https_video_link,
    submission_issues,
)


ROOT = Path(__file__).resolve().parents[1]


def _deck(root: Path, name: str, sources: list[str]) -> Path:
    deck = root / "docs" / "report" / "slides" / name
    deck.parent.mkdir(parents=True, exist_ok=True)
    tags = "".join(f'<video src="{src}"></video>' for src in sources)
    deck.write_text(f"<section>{tags}</section>", encoding="utf-8")
    return deck


def _clip(root: Path, name: str, *, playable: bool) -> None:
    path = root / "docs" / "report" / "video" / name
    path.parent.mkdir(parents=True, exist_ok=True)
    # Only the sample entry fourcc matters: avc1 is H.264, mp4v is the MPEG-4
    # part 2 that browsers refuse to decode.
    path.write_bytes(b"\x00\x00\x00\x18ftypisom" + (b"avc1" if playable else b"mp4v"))


def test_current_package_has_only_the_known_human_placeholder():
    # The colcon container exercises the unpacked source/package contract but its
    # workspace does not guarantee a usable Git index. Large tracked binaries are
    # covered separately below with an explicit inventory; the real CLI remains
    # strict and obtains that inventory from ``git ls-files``.
    issues = submission_issues(ROOT, tracked=[])
    assert issues == [
        "PLACEHOLDER: README.md: <ссылка на облако организаторов>",
    ]


def test_preflight_reports_missing_artifacts_and_unapproved_large_files(tmp_path):
    large = tmp_path / "surprise.mp4"
    large.write_bytes(b"0" * (MAX_TRACKED_BINARY_BYTES + 1))

    issues = submission_issues(tmp_path, tracked=[large.name])

    assert "MISSING: README.md" in issues
    assert "MISSING: src/perception.py" in issues
    assert "MISSING: docs/report/slides/deck-*.html" in issues
    assert "LARGE: surprise.mp4: 5.0 MiB" in issues


def test_video_link_must_be_https_and_in_the_video_paragraph():
    assert has_https_video_link("**Видеодемонстрация:** https://cloud.example/video")
    assert not has_https_video_link("**Видеодемонстрация:** локальный файл")
    assert not has_https_video_link("https://cloud.example/video")
    assert not has_https_video_link(
        "**Видеодемонстрация:** заполнить позже\n\n"
        "Другая ссылка: https://example.com"
    )


def test_preflight_rejects_non_https_video_reference(tmp_path):
    readme = tmp_path / "README.md"
    readme.write_text("**Видеодемонстрация:** локальный файл", encoding="utf-8")

    issues = submission_issues(tmp_path, tracked=[])

    assert "CLOUD_LINK: README.md: missing HTTPS video URL" in issues


def test_broken_local_links_flags_dead_crossrefs(tmp_path):
    (tmp_path / "docs" / "report").mkdir(parents=True)
    (tmp_path / "docs" / "report" / "a.md").write_text(
        "ok [b](b.md), external [x](https://e), anchor [h](#h)", encoding="utf-8")
    (tmp_path / "docs" / "report" / "b.md").write_text("hi", encoding="utf-8")
    (tmp_path / "docs" / "report" / "dead.md").write_text(
        "see [gone](nope.md)", encoding="utf-8")

    issues = broken_local_links(tmp_path)

    assert "BROKEN_LINK: docs/report/dead.md: nope.md" in issues
    assert not any("a.md" in issue for issue in issues)


def test_current_docs_have_no_broken_links():
    # Regression guard: the shipped kit's cross-references all resolve.
    assert broken_local_links(ROOT) == []


def test_deck_media_flags_absent_and_unplayable_clips(tmp_path):
    name = Path(SHIPPED_DECK).name
    _clip(tmp_path, "good.mp4", playable=True)
    _clip(tmp_path, "mpeg4.mp4", playable=False)
    _deck(tmp_path, name, ["../video/good.mp4", "../video/mpeg4.mp4", "../video/gone.mp4"])

    issues = deck_media_issues(tmp_path)

    assert f"MISSING_MEDIA: {SHIPPED_DECK}: ../video/gone.mp4" in issues
    assert f"UNPLAYABLE_MEDIA: {SHIPPED_DECK}: ../video/mpeg4.mp4: not H.264" in issues


def test_shipped_deck_must_carry_the_demo_gallery(tmp_path):
    name = Path(SHIPPED_DECK).name
    _clip(tmp_path, "hero.mp4", playable=True)
    _deck(tmp_path, name, ["../video/hero.mp4"])
    # An alternate style deck with a single clip is fine; only the presented one
    # has to reach the video count.
    _deck(tmp_path, "deck-a-swiss-modern.html", ["../video/hero.mp4"])

    issues = deck_media_issues(tmp_path)

    assert issues == [
        f"DECK_VIDEOS: {SHIPPED_DECK}: 1 playable clips, need {MIN_DECK_VIDEOS}",
    ]


def test_current_decks_render_from_a_clean_clone():
    # Regression guard: every clip the decks reference is in the repository and
    # in a codec a browser will actually play.
    assert deck_media_issues(ROOT) == []


def test_deck_media_must_be_tracked_in_git_not_merely_present_on_disk(tmp_path):
    """A clip that exists locally but is gitignored still breaks the jury's clone.

    This is the gap that let a real defect ship: when the gallery moved from the
    three-head rig to the two-head one, `.gitignore` kept re-including `rig3_*`
    while the deck started referencing `rig2_*`. Every clip was on the author's
    disk, so the disk-only check stayed green — but four of the five clips on the
    shipped deck's gallery slide were absent from the repository, and a fresh
    clone rendered four broken videos on the slide that demonstrates the
    classification rule.
    """
    name = Path(SHIPPED_DECK).name
    for clip in ("kept.mp4", "ignored.mp4"):
        _clip(tmp_path, clip, playable=True)
    _deck(tmp_path, name, ["../video/kept.mp4", "../video/ignored.mp4"])

    issues = deck_media_issues(
        tmp_path, tracked={"docs/report/video/kept.mp4"})

    assert f"UNTRACKED_MEDIA: {SHIPPED_DECK}: ../video/ignored.mp4" in issues
    assert not any("kept.mp4" in issue for issue in issues), (
        "a tracked clip must not be reported")


def test_current_deck_media_is_all_tracked():
    """Regression guard on the real repository, with git as the source of truth."""
    from scripts.check_submission import tracked_files

    issues = deck_media_issues(ROOT, tracked=set(tracked_files(ROOT)))

    assert issues == [], f"deck media missing from the repository: {issues}"
