from pathlib import Path

import pytest

from scripts.check_submission import (
    MAX_TRACKED_BINARY_BYTES,
    MIN_DECK_VIDEOS,
    SHIPPED_DECK,
    broken_local_links,
    deck_media_issues,
    has_https_video_link,
    reel_clip_issues,
    outline_media_issues,
    reel_clip_names,
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


def test_no_tracked_binary_is_large_without_being_argued_for():
    """Run the size gate against the REAL git inventory, not an empty one.

    ``test_current_package_has_only_the_known_human_placeholder`` passes
    ``tracked=[]``, so the large-file rule never fires there — the suite stayed
    green while the actual preflight went BLOCKED. That is how a 13.6 MiB reel
    landed in the tree with every test passing. Anything big must be listed in
    ALLOWED_LARGE_FILES with a reason, which is a decision, not an oversight.
    """
    from scripts.check_submission import tracked_files

    if not tracked_files(ROOT):
        pytest.skip("no git inventory here (colcon container); nothing to size-check")

    large = [issue for issue in submission_issues(ROOT) if issue.startswith("LARGE:")]

    assert large == [], f"large tracked files nobody approved: {large}"


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

    tracked = set(tracked_files(ROOT))
    if not tracked:
        pytest.skip("no git inventory here (colcon container); nothing to check against")

    issues = deck_media_issues(ROOT, tracked=tracked)

    assert issues == [], f"deck media missing from the repository: {issues}"


def test_outline_media_issues_flags_a_clip_the_defence_plan_names_but_git_lacks(tmp_path):
    """Third repeat of one failure: a document promises footage the repo lacks.

    The deck had it (rig3_* re-included while the deck played rig2_*), the reel had
    it (two hero clips never committed), and the defence plan had it too —
    ``presentation_outline.md`` listed ``hero_diverter_bottle_D.mp4`` as the standby
    clip, and that file was never in git. Nothing watched this one: the deck guard
    reads the HTML decks, and the outline is markdown that no player opens, so the
    gap only surfaces when someone reaches for the clip on stage.
    """
    outline = tmp_path / "docs" / "report" / "presentation_outline.md"
    outline.parent.mkdir(parents=True, exist_ok=True)
    outline.write_text(
        "- **Клип 2:** `kept.mp4` — есть\n- Резерв: `missing.mp4`, `shot.png`\n",
        encoding="utf-8")

    issues = outline_media_issues(
        tmp_path, tracked={"docs/report/video/kept.mp4", "docs/report/img/shot.png"})

    assert issues == ["OUTLINE_MEDIA: missing.mp4"]


def test_outline_media_issues_stays_quiet_without_a_git_inventory(tmp_path):
    assert outline_media_issues(tmp_path, tracked=None) == []


def test_current_defence_plan_names_only_media_the_repository_carries():
    """Regression guard on the real outline — what is promised on stage must exist."""
    from scripts.check_submission import tracked_files

    tracked = set(tracked_files(ROOT))
    if not tracked:
        pytest.skip("no git inventory here (colcon container); nothing to check against")

    issues = outline_media_issues(ROOT, tracked=tracked)

    assert issues == [], f"defence plan names media git does not carry: {issues}"


def test_reel_clip_issues_flags_a_clip_the_plan_needs_but_the_repository_lacks():
    """The reel has the deck's failure mode, and no guard was watching it.

    ``build_demo_reel.py`` aborts on a missing clip, so the author — who has every
    clip on disk — never sees a problem, while a clean clone cannot rebuild the
    video demonstration at all. The preflight listed only ``hero_stream_mixed_cd``
    by hand, so its media list and the reel's own plan could drift apart silently,
    which is exactly what happened to the deck on 30.07.
    """
    issues = reel_clip_issues(
        ["kept.mp4", "local_only.mp4"],
        tracked={"docs/report/video/kept.mp4"},
    )

    assert issues == ["UNTRACKED_REEL_CLIP: local_only.mp4"]


def test_reel_clip_issues_stays_quiet_without_a_git_inventory():
    # Same rule the deck check follows: an empty inventory means "no usable git
    # index here" (the colcon container), not "nothing is tracked".
    assert reel_clip_issues(["anything.mp4"], tracked=None) == []


def test_tracked_files_returns_nothing_where_git_cannot_answer(tmp_path):
    """A preflight must report problems, not become one.

    Every caller here is written around "an empty inventory means no usable git
    index" — but ``tracked_files`` ran git with ``check=True`` and raised instead,
    so the contract was never actually available. Two real consequences: the
    colcon container (where the workspace trips git's dubious-ownership guard and
    ``git ls-files`` exits 128) failed three tests on every push since 28.07, and
    an organizer who unpacks the archive without ``.git`` would get a traceback
    instead of the submission report.
    """
    from scripts.check_submission import tracked_files

    assert tracked_files(tmp_path) == []


def test_reel_clip_names_actually_reads_the_plan():
    """The wiring, not just the rule — a silent empty list disarms the guard.

    ``reel_clip_names`` loads the plan by path and returns [] on any failure, so a
    broken import looks exactly like "the reel needs no clips" and the preflight
    goes quiet. It did: loading a module that defines dataclasses without first
    registering it in ``sys.modules`` raises AttributeError inside @dataclass, and
    the unit tests above passed anyway because they never went through this path.
    """
    from scripts.build_demo_reel import SEGMENTS, Clip

    assert reel_clip_names(ROOT) == [s.name for s in SEGMENTS if isinstance(s, Clip)]


def test_the_preflight_names_every_reel_clip_missing_from_the_repository():
    """Whatever the reel cannot rebuild from a clean clone must be SAID, not hidden.

    This is a consistency guard, not a demand that the list be empty: it asserts
    the preflight reports exactly the clips git does not carry, so the gap can
    never be silently larger than what the submission report claims.
    """
    from scripts.build_demo_reel import SEGMENTS, Clip
    from scripts.check_submission import tracked_files

    tracked = set(tracked_files(ROOT))
    if not tracked:
        pytest.skip("no git inventory here (colcon container); nothing to check against")
    planned = [s.name for s in SEGMENTS if isinstance(s, Clip)]
    absent = [n for n in planned if f"docs/report/video/{n}" not in tracked]

    reported = reel_clip_issues(planned, tracked=tracked)

    assert reported == [f"UNTRACKED_REEL_CLIP: {name}" for name in absent]
