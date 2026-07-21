# -*- coding: utf-8 -*-
"""The three-head bridge must be a superset of the shipped one.

A rig that silently bridges FEWER topics than the world publishes fails as a
missing head, which the node then degrades over — quietly, with no error.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _topics(path):
    return {line.split("topic_name:")[1].strip()
            for line in path.read_text(encoding="utf-8").splitlines()
            if line.strip().startswith("- topic_name:")}


def test_three_head_bridge_keeps_every_shipped_topic():
    base = _topics(ROOT / "sim" / "bridge.yaml")
    rig = _topics(ROOT / "sim" / "bridge_3cam.yaml")
    assert base <= rig, f"three-head bridge dropped {base - rig}"


def test_three_head_bridge_adds_exactly_the_two_side_streams():
    base = _topics(ROOT / "sim" / "bridge.yaml")
    rig = _topics(ROOT / "sim" / "bridge_3cam.yaml")
    assert rig - base == {"/camera_side_neg_y/depth_image",
                          "/camera_side_pos_y/depth_image"}


def test_the_frame_dumper_looks_at_the_same_side_topics_as_the_node():
    """A dumper aimed at a topic nobody publishes produces an EMPTY diagnosis.

    The side clouds are only ever inspected offline, off these dumps, so a typo
    here reads as "the head sees nothing" — the exact conclusion the dump was
    opened to test. Asserted by text: importing the dumper needs rclpy/cv_bridge,
    which the test host does not have to carry.
    """
    dumper = (ROOT / "scripts" / "dump_camera.py").read_text(encoding="utf-8")
    for topic in ("/camera_side_neg_y/depth_image", "/camera_side_pos_y/depth_image"):
        assert topic in dumper, f"dumper does not dump {topic}"


def test_side_topics_match_what_the_node_subscribes_to():
    """The world, the bridge and the node all name these topics; a typo in any
    one of them shows up as a head that never arrives."""
    node = (ROOT / "src" / "perception_node.py").read_text(encoding="utf-8")
    world = (ROOT / "sim" / "worlds" / "cell_diverter_3cam.sdf").read_text(encoding="utf-8")
    for topic in ("/camera_side_neg_y/depth_image", "/camera_side_pos_y/depth_image"):
        assert topic in node, f"node does not subscribe to {topic}"
        assert f"<topic>{topic.split('/')[1]}</topic>" in world, f"world lacks {topic}"
