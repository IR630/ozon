# -*- coding: utf-8 -*-
"""World locks for the second head (feat/two-cameras, sim/worlds/cell_diverter.sdf).

The side camera's whole viability rests on one geometric fact that is invisible in a
diff: its housing must sit OUTSIDE the top camera's view frustum, or its blob touches
the top mask border and _find_item returns None on every frame (the day-5 blocker).
These tests pin that, plus the SDF<->perception<->bridge consistency the node needs.
Pure XML/text math — no simulator.
"""
import xml.etree.ElementTree as ET
from pathlib import Path

from src.perception import (
    CAMERA_Z_M,
    FX,
    IMG_W,
    SIDE_CAMERA_POS_M,
)

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "sim" / "worlds" / "cell_diverter.sdf"
BRIDGE = ROOT / "sim" / "bridge.yaml"


def _side_model():
    model = ET.parse(WORLD).getroot().find(".//model[@name='camera_side']")
    assert model is not None, "camera_side model missing from the world"
    return model


def _top_window_half_width_at(z_m):
    """Half-width (m) of the top camera's Y view window at height z — widens downward."""
    return (IMG_W / 2.0) * (CAMERA_Z_M - z_m) / FX


def test_side_camera_pose_mirrors_perception_constants():
    """The world file is the source; perception's SIDE_CAMERA_POS_M must match it."""
    pose = [float(v) for v in _side_model().find("pose").text.split()]
    assert tuple(pose[:3]) == SIDE_CAMERA_POS_M, pose[:3]


def test_side_camera_publishes_its_own_topic():
    sensor = _side_model().find(".//sensor[@type='rgbd_camera']")
    assert sensor is not None, "camera_side needs an rgbd_camera sensor"
    assert sensor.find("topic").text == "camera_side"


def test_side_housing_clears_the_top_camera_frustum():
    """The housing's inner (belt-facing) edge must stay outside the top Y window.

    Checked at the housing's LOWEST point, where the frustum is widest — the failure
    mode is the low structure re-entering the widening cone (docs/decisions.md).
    """
    _, y, z, *_ = [float(v) for v in _side_model().find("pose").text.split()]
    size = [float(v) for v in _side_model().find(".//visual/geometry/box/size").text.split()]
    inner_edge_y = abs(y) - size[1] / 2.0          # nearest the belt centre
    lowest_z = z - size[2] / 2.0                    # widest frustum here
    assert inner_edge_y > _top_window_half_width_at(lowest_z), (
        f"housing inner edge |y|={inner_edge_y:.3f} enters the top window "
        f"(half-width {_top_window_half_width_at(lowest_z):.3f} at z={lowest_z:.3f})")


def test_bridge_exposes_side_camera_topics():
    text = BRIDGE.read_text(encoding="utf-8")
    for topic in ("/camera_side/depth_image", "/camera_side/camera_info", "/camera_side/image"):
        assert topic in text, f"bridge.yaml missing {topic}"
