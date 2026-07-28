from pathlib import Path

from scripts.check_submission import (
    MAX_TRACKED_BINARY_BYTES,
    broken_local_links,
    has_https_video_link,
    submission_issues,
)


ROOT = Path(__file__).resolve().parents[1]


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
