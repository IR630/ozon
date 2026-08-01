"""Deployment pinning contract (Stage 25).

The organizers rebuild the image from sources: an unpinned base tag or a bare
requirement name silently drifts, and the deployed contour is no longer the
validated one. In the first build of docker/Dockerfile this actually happened
inside a single environment: bare names in requirements.txt were satisfied by
the apt-built numpy 1.21.5/scipy 1.8.0 while dev hosts ran numpy 1.26 — the
lock file records exactly what the image validated.

Static checks on purpose: rebuilding the image is scripts/check_clean_deploy.sh's
job, and neither the plain pytest job nor CI's e2e needs docker for these locks.
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

DOCKERFILE = (ROOT / "docker" / "Dockerfile").read_text(encoding="utf-8")
CONSTRAINTS = (ROOT / "docker" / "pip-constraints.txt").read_text(encoding="utf-8")
REQUIREMENTS = (ROOT / "requirements.txt").read_text(encoding="utf-8")
DEPLOY_CHECK = (ROOT / "scripts" / "check_clean_deploy.sh").read_text(encoding="utf-8")


def _names(text, pattern):
    names = []
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        match = re.fullmatch(pattern, line)
        assert match, f"unparseable dependency line: '{line}'"
        names.append(match.group(1).lower().replace("_", "-"))
    return names


def test_base_image_is_pinned_by_digest():
    assert re.search(
        r"^FROM osrf/ros:humble-desktop@sha256:[0-9a-f]{64}$", DOCKERFILE, re.M)


def test_image_installs_through_the_constraints_lock():
    assert "COPY docker/pip-constraints.txt /tmp/pip-constraints.txt" in DOCKERFILE
    assert "-r /tmp/requirements.txt -c /tmp/pip-constraints.txt" in DOCKERFILE


def test_dockerignore_lets_every_copied_file_into_the_build_context():
    """.dockerignore excludes ** — each COPY source needs its own whitelist line."""
    ignore = (ROOT / ".dockerignore").read_text(encoding="utf-8").splitlines()

    for src in re.findall(r"^COPY (\S+) ", DOCKERFILE, re.M):
        assert f"!{src}" in ignore, f"COPY {src} is excluded from the build context"


def test_every_requirement_is_locked_to_an_exact_version():
    required = _names(REQUIREMENTS, r"([A-Za-z0-9._-]+)")
    locked = _names(CONSTRAINTS, r"([A-Za-z0-9._-]+)==[A-Za-z0-9.]+")

    missing = set(required) - set(locked)
    assert not missing, f"requirements without a deployment pin: {sorted(missing)}"


def test_deploy_check_runs_the_e2e_on_a_clean_checkout_not_the_worktree():
    assert "git archive HEAD" in DEPLOY_CHECK
    assert "bash scripts/ci_e2e.sh" in DEPLOY_CHECK
    # the image must be built from the archived sources, not the live repo root
    assert re.search(r'docker build .*-f "\$CHECKOUT/docker/Dockerfile" "\$CHECKOUT"',
                     DEPLOY_CHECK)


WORKFLOW = (ROOT / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")


def test_ci_never_installs_a_named_package_without_a_version_source():
    """Twice now, a release published upstream turned CI red on an unrelated commit.

    28.07 it was ruff 0.16 (wider default rule set, 370 findings); 01.08 it was
    trimesh 5.0.0, published 00:05 UTC and installed by the 01:38 run, which drops
    numpy 1.x — and the colcon container keeps numpy apt-built at 1.x on purpose,
    so every test died at collection. Neither commit touched the dependency.

    `pip install -r requirements.txt` is deliberately exempt: the lint job runs on
    a dev-grade Python where unpinned is the point, and that is stated in
    docker/pip-constraints.txt. What must never reappear is a BARE PACKAGE NAME on
    a pip line — that is the form with no version source at all.
    """
    offenders = []
    for line in WORKFLOW.splitlines():
        stripped = line.strip()
        if stripped.startswith("#") or not re.search(r"\bpip3?\s+install\b", stripped):
            continue
        if "-r requirements.txt" in stripped:
            continue
        constrained = "-c docker/pip-constraints.txt" in stripped
        pinned = "==" in stripped
        if not (constrained or pinned):
            offenders.append(stripped)
    assert not offenders, (
        "pip line with no version source (add -c docker/pip-constraints.txt "
        f"or an == pin): {offenders}")


def test_ci_and_the_deployment_image_agree_on_trimesh():
    """One version, one file. The CI step applies the lock instead of copying it.

    A second hand-written "4.12.2" in the workflow is exactly how the deployed
    image and the tested environment drift apart while both look pinned.
    """
    assert "pip3 install --no-deps -c docker/pip-constraints.txt trimesh" in WORKFLOW
    assert re.search(r"^trimesh==\d+\.\d+\.\d+$", CONSTRAINTS, re.M), (
        "the lock must carry the trimesh version the CI step now defers to")
