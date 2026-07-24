from pathlib import Path

from scripts.check_submission import (
    MAX_TRACKED_BINARY_BYTES,
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
