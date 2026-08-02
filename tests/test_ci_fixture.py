"""The text-only CI item must match the production box's domain dimensions."""
import re
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_ci_box_fixture_is_300x200x200_mm_and_1_5_kg():
    path = (ROOT / "sim" / "models" / "fixtures"
            / "box_300x200x200" / "model.sdf")
    root = ET.parse(path).getroot()
    assert [float(v) for v in root.findtext(".//collision/geometry/box/size").split()] == [
        0.3, 0.2, 0.2]
    assert float(root.findtext(".//mass")) == 1.5


def test_ci_occupied_estop_fixture_is_400x400x300_mm_and_3_kg():
    path = (ROOT / "sim" / "models" / "fixtures"
            / "box_400x400x300" / "model.sdf")
    root = ET.parse(path).getroot()
    assert [float(v) for v in root.findtext(".//collision/geometry/box/size").split()] == [
        0.4, 0.4, 0.3]
    assert float(root.findtext(".//mass")) == 3.0


def test_ci_e2e_runs_occupied_estop_with_text_only_fixtures():
    e2e = (ROOT / "scripts" / "ci_e2e.sh").read_text(encoding="utf-8")
    smoke = (ROOT / "scripts" / "smoke_estop_stream.sh").read_text(encoding="utf-8")

    assert "ITEM_MODEL_ROOT=${ITEM_MODEL_ROOT:-sim/models/items}" in smoke
    assert "LOGDIR=/tmp/estop_stream_e2e" in e2e
    assert "python3 scripts/parse_ign_joint_angle.py" in smoke

    # THE BUDGET IS PINNED AS AN INVARIANT, NOT AS A NUMBER. This line used to
    # assert `timeout 180` literally, so raising the cap broke a test that has
    # nothing to do with the cap's value. What actually matters is the ordering:
    # the E-stop smoke boots the SAME multi-head world as the skeleton step and
    # then runs a stream on top of it, so giving it less time than the simpler
    # step is how a green contour reports red on a loaded machine — which is
    # exactly what happened on 02.08 (run 30730053772, exit 124 one line after
    # the recovery gate passed).
    caps = {name: int(seconds) for seconds, name in re.findall(
        r"^timeout (\d+) bash scripts/(\S+)", e2e, re.M)}
    assert "smoke_estop_stream.sh" in caps, "the E-stop smoke must run under a timeout"
    assert caps["smoke_estop_stream.sh"] >= caps["run_skeleton.sh"], (
        f"E-stop smoke has less budget than the simpler skeleton step: {caps}")


def test_host_requirements_include_opencv_for_validation_paths():
    """README's host install must exercise PNG/overlay tests, not skip them."""
    requirements = (
        ROOT / "requirements.txt"
    ).read_text(encoding="utf-8").splitlines()

    assert "opencv-python-headless" in requirements
