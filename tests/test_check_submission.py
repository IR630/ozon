from pathlib import Path

from scripts.check_submission import MAX_TRACKED_BINARY_BYTES, submission_issues


ROOT = Path(__file__).resolve().parents[1]


def test_current_package_has_only_the_known_human_placeholder():
    issues = submission_issues(ROOT)
    assert issues == [
        "PLACEHOLDER: README.md: <ссылка на облако организаторов>",
    ]


def test_preflight_reports_missing_artifacts_and_unapproved_large_files(tmp_path):
    large = tmp_path / "surprise.mp4"
    large.write_bytes(b"0" * (MAX_TRACKED_BINARY_BYTES + 1))

    issues = submission_issues(tmp_path, tracked=[large.name])

    assert "MISSING: README.md" in issues
    assert "MISSING: docs/report/slides/deck-*.html" in issues
    assert "LARGE: surprise.mp4: 5.0 MiB" in issues
