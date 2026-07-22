# -*- coding: utf-8 -*-
"""The two-head rig must bridge exactly one side stream — no more, no fewer.

The two failure modes are opposite and both silent. Bridge too FEW topics and the
node waits for a head the world publishes, degrading to the top view with nothing
in the log to say so. Bridge too MANY and the rig under test is not the two-head
rig at all: the node holds a subscription that never fires, and the census would
be attributed to a configuration that was never run.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _topics(path):
    return {line.split("topic_name:")[1].strip()
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip().startswith("- topic_name:")}


def test_two_head_bridge_keeps_every_shipped_topic():
    base = _topics(ROOT / "sim" / "bridge.yaml")
    rig = _topics(ROOT / "sim" / "bridge_2cam.yaml")
    assert base <= rig, f"two-head bridge dropped {base - rig}"


def test_two_head_bridge_adds_exactly_the_minus_y_side_stream():
    base = _topics(ROOT / "sim" / "bridge.yaml")
    rig = _topics(ROOT / "sim" / "bridge_2cam.yaml")
    assert rig - base == {"/camera_side_neg_y/depth_image"}


def test_the_world_carries_the_minus_y_head_and_only_it():
    """Which head it is, is the whole experiment.

    The offline probe scored "2: top+бок" as TOP + SIDE_NEG_Y; a world built with
    the +y head instead would produce numbers compared against a probe row that
    does not describe it.
    """
    world = (ROOT / "sim" / "worlds" / "cell_diverter_2cam.sdf").read_text(encoding="utf-8")
    assert "<topic>camera_side_neg_y</topic>" in world
    assert "camera_side_pos_y" not in world


def test_the_two_head_world_is_the_three_head_world_minus_one_head():
    """The cells must be IDENTICAL apart from the deleted head.

    A belt, blade or zone that drifted between the two worlds would turn a camera
    comparison into a comparison of two different cells — and the misroute counts
    would be attributed to head count regardless. Asserted by reconstructing the
    two-head body from the three-head one: same physics, same belt, same blades,
    same zones, or this fails.
    """
    import re

    two = (ROOT / "sim" / "worlds" / "cell_diverter_2cam.sdf").read_text(encoding="utf-8")
    three = (ROOT / "sim" / "worlds" / "cell_diverter_3cam.sdf").read_text(encoding="utf-8")
    # Header comments describe the rig and differ on purpose; the world does not.
    body = lambda t: t.split('<sdf version="1.8">', 1)[1]   # noqa: E731
    expected = re.sub(r'\n    <model name="camera_side_pos_y">.*?\n    </model>\n',
                      "\n", body(three), flags=re.DOTALL)
    # The -y head's own comment block loses its plural; exempt it, nothing else.
    strip_comments = lambda t: re.sub(r"<!--.*?-->", "", t, flags=re.DOTALL)  # noqa: E731
    assert strip_comments(body(two)) == strip_comments(expected)
