# -*- coding: utf-8 -*-
"""Message contracts lock their field names (perception/classifier depend on them).

Parses the .msg files directly, so it runs without a ROS/colcon build (that
syntactic check lives in CI). This guards against silent contract drift —
a renamed field breaks P3<->P4 long before anyone rebuilds the workspace.
"""
from pathlib import Path

MSG_DIR = Path(__file__).resolve().parent.parent / "ros_msgs" / "msg"


def fields(msg_name):
    """{field_name: type} parsed from a .msg file, comments/blanks skipped."""
    out = {}
    for line in (MSG_DIR / msg_name).read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        type_, name = line.split()
        out[name] = type_
    return out


def test_item_measurement_fields():
    f = fields("ItemMeasurement.msg")
    assert f == {
        "header": "std_msgs/Header",
        "item_id": "uint32",
        "dims_mm": "float32[3]",
        "k": "float32",
        "confidence": "float32",
        "position": "geometry_msgs/Point",
    }
    # category is the classifier's output, never the measurement's
    assert "category" not in f


def test_item_classification_carries_category():
    f = fields("ItemClassification.msg")
    assert f["category"] == "string"
    assert f["dims_mm"] == "float32[3]"


def test_both_messages_registered_for_build():
    cmake = (MSG_DIR.parent / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "msg/ItemClassification.msg" in cmake
    assert "msg/ItemMeasurement.msg" in cmake
