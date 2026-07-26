# -*- coding: utf-8 -*-
"""The miscalibrated world must differ from the real one ONLY by the stated error.

The whole value of the run this world feeds is that the difference is known and
small. A generator that also nudged the belt, the diverters or the top head would
produce a census nobody could attribute, so the diff is asserted line by line.
"""
import math
import re

import pytest

from scripts.make_miscal_world import (
    SHIFT_MM, SRC_WORLD, TILT_DEG, heads_present, miscalibrate)
from src.constants import CAMERA_SIDE_NEG_Y_POSE_M, CAMERA_SIDE_POS_Y_POSE_M


def _pose(sdf_text, model_name):
    match = re.search(
        r'<model name="%s">\s*\n\s*<static>true</static>\s*\n\s*<pose>([^<]+)</pose>'
        % model_name, sdf_text)
    assert match, "no pose for %s" % model_name
    return [float(v) for v in match.group(1).split()]


@pytest.fixture(scope="module")
def worlds():
    clean = SRC_WORLD.read_text(encoding="utf-8")
    return clean, miscalibrate(clean)


def test_only_the_two_side_head_poses_change(worlds):
    clean, dirty = worlds
    differing = [(a, b) for a, b in zip(clean.splitlines(), dirty.splitlines()) if a != b]
    assert len(clean.splitlines()) == len(dirty.splitlines())
    assert len(differing) == 2, differing
    assert all("<pose>" in a for a, _ in differing)


def test_side_heads_move_outward_by_the_budget(worlds):
    clean, dirty = worlds
    for name, sign in (("camera_side_neg_y", -1.0), ("camera_side_pos_y", +1.0)):
        before, after = _pose(clean, name), _pose(dirty, name)
        assert after[1] - before[1] == pytest.approx(sign * SHIFT_MM / 1000.0)
        assert after[4] - before[4] == pytest.approx(math.radians(TILT_DEG))
        # x, z, roll and yaw are untouched: one error, not four.
        assert [after[i] for i in (0, 2, 3, 5)] == [before[i] for i in (0, 2, 3, 5)]


def test_top_head_is_not_touched(worlds):
    clean, dirty = worlds
    assert _pose(clean, "camera") == _pose(dirty, "camera")


def test_the_node_is_left_believing_the_nominal_poses(worlds):
    """THIS is what makes the run a calibration test rather than a re-aim.

    If the constants moved with the world the node would be perfectly registered
    again and the run would measure nothing.
    """
    _clean, dirty = worlds
    for name, pose in (("camera_side_neg_y", CAMERA_SIDE_NEG_Y_POSE_M),
                       ("camera_side_pos_y", CAMERA_SIDE_POS_Y_POSE_M)):
        believed_y = pose[0][1]
        assert abs(_pose(dirty, name)[1] - believed_y) == pytest.approx(SHIFT_MM / 1000.0)


# --- two-head rig -----------------------------------------------------------
# The same error, applied to a world that carries only the -y head. The failure
# this guards is silence: a generator that found no head to move and wrote the
# clean world out under a "_miscal" name would produce a census that proves the
# rig survives an error it was never subjected to.

TWO_CAM_WORLD = SRC_WORLD.parent / "cell_diverter_2cam.sdf"


def test_heads_present_reads_the_rig_off_the_world():
    three = SRC_WORLD.read_text(encoding="utf-8")
    two = TWO_CAM_WORLD.read_text(encoding="utf-8")
    assert [n for n, _s in heads_present(three)] == ["camera_side_neg_y",
                                                     "camera_side_pos_y"]
    assert [n for n, _s in heads_present(two)] == ["camera_side_neg_y"]


def test_the_two_head_world_is_miscalibrated_on_its_one_head():
    clean = TWO_CAM_WORLD.read_text(encoding="utf-8")
    dirty = miscalibrate(clean, heads=heads_present(clean))
    before, after = _pose(clean, "camera_side_neg_y"), _pose(dirty, "camera_side_neg_y")
    assert after[1] - before[1] == pytest.approx(-SHIFT_MM / 1000.0)
    assert after[4] - before[4] == pytest.approx(math.radians(TILT_DEG))
    assert _pose(clean, "camera") == _pose(dirty, "camera")
    differing = [(a, b) for a, b in zip(clean.splitlines(), dirty.splitlines()) if a != b]
    assert len(differing) == 1, differing


def test_asking_for_an_absent_head_is_an_error_not_a_no_op():
    """The whole point of naming heads explicitly: a typo must not pass silently."""
    clean = TWO_CAM_WORLD.read_text(encoding="utf-8")
    with pytest.raises(ValueError):
        miscalibrate(clean, heads=(("camera_side_pos_y", +1.0),))


@pytest.mark.parametrize("axis,index", [("roll", 3), ("pitch", 4), ("yaw", 5)])
def test_each_calibration_axis_moves_its_own_angle_and_no_other(axis, index):
    """One axis per world, or the census cannot be attributed to an angle.

    Until 26.07 only pitch was ever perturbed, so `path_to_line.md` #2 honestly
    read "one angle checked of three". Roll matters most of the three and is the
    one an offline probe cannot reach at all: `src.multiview.camera_axes` builds
    the basis as `right = forward x world_up`, so the reconstruction carries five
    degrees of freedom and a rolled head is unrepresentable rather than merely
    mis-modelled. It has to be injected into the WORLD, which is what this does.
    """
    clean = SRC_WORLD.read_text(encoding="utf-8")
    dirty = miscalibrate(clean, axes=(axis,))
    for name, _sign in heads_present(clean):
        before, after = _pose(clean, name), _pose(dirty, name)
        for i in (3, 4, 5):
            moved = after[i] - before[i]
            if i == index:
                assert moved == pytest.approx(math.radians(TILT_DEG)), axis
            else:
                assert moved == pytest.approx(0.0, abs=1e-12), f"{axis} moved angle {i}"


def test_the_default_axis_is_still_pitch_alone():
    """Every world and census taken before 26.07 used pitch — defaults must not move."""
    clean = SRC_WORLD.read_text(encoding="utf-8")
    assert miscalibrate(clean) == miscalibrate(clean, axes=("pitch",))


def test_an_unknown_axis_is_loud_instead_of_a_silent_no_op():
    """A typo'd axis must not produce a world that is quietly NOT miscalibrated."""
    clean = SRC_WORLD.read_text(encoding="utf-8")
    with pytest.raises(ValueError, match="unknown calibration axis"):
        miscalibrate(clean, axes=("tilt",))
