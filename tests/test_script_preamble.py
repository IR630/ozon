"""Preamble contract for every script that sources the colcon workspace.

Three invariants, each learned from real breakage rather than taste:

1. errexit is armed BEFORE anything that can fail. It used to sit AFTER the two
   `source` lines, so on a clean checkout the missing workspace did not stop the
   run: the script carried on unsourced and died much later with a false diagnosis
   ("the belt never reached full speed").

2. nounset is NEVER armed before the sources. ROS's own setup.bash reads unbound
   variables (`/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES:
   unbound variable`) and dies under `set -u`, taking the script with it on a
   perfectly good workspace. Moving a `set -eu` wholesale to the top broke
   smoke_multi_item/smoke_estop/smoke_estop_stream exactly this way; -u belongs
   after the sources, which is where it always sat.

3. The clean-checkout guard names the fix (`colcon build --packages-select
   ros_msgs`) before the workspace source, since install/ is gitignored.

These are static checks on purpose: the nounset break only reproduces in a real
ROS environment, which neither the Windows/plain-pytest job nor CI's e2e (it drives
run_skeleton.sh only) exercises for these scripts.
"""
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]

# Every script that sources the colcon workspace. ci_e2e.sh is deliberately absent:
# it BUILDS the workspace itself, so it has no clean-checkout guard to assert.
ROS_SCRIPTS = [
    "run_skeleton.sh",
    "run_stream.sh",
    "smoke_multi_item.sh",
    "smoke_estop.sh",
    "smoke_estop_stream.sh",
    "smoke_estop_recover.sh",
    "dump_item_frame.sh",
    "record_skeleton_video.sh",
]

NOUNSET = re.compile(r"^set -[a-z]*u", re.M)


def _text(name):
    return (ROOT / "scripts" / name).read_text(encoding="utf-8")


@pytest.mark.parametrize("name", ROS_SCRIPTS)
def test_errexit_is_armed_before_anything_that_can_fail(name):
    body = [line for line in _text(name).splitlines()
            if line.strip() and not line.startswith("#")]

    assert body[0] == "set -e", f"{name}: first executable line must arm errexit, got {body[:3]}"


@pytest.mark.parametrize("name", ROS_SCRIPTS)
def test_nounset_is_never_armed_before_the_ros_source(name):
    text = _text(name)
    preamble = text[:text.index("source /opt/ros/humble/setup.bash")]

    found = NOUNSET.search(preamble)
    assert found is None, (
        f"{name}: `{found.group() if found else ''}` before the ROS source — setup.bash reads "
        "AMENT_TRACE_SETUP_FILES and dies under nounset, on a good workspace too"
    )


@pytest.mark.parametrize("name", ROS_SCRIPTS)
def test_a_clean_checkout_is_told_to_build_the_workspace(name):
    text = _text(name)

    assert text.index("colcon build --packages-select ros_msgs") \
        < text.index('source "$ROS_INSTALL_ROOT/setup.bash"'), \
        f"{name}: the guard must name the fix BEFORE sourcing the workspace"
