# -*- coding: utf-8 -*-
"""The presentation drawing must stay dimensionally tied to the final SDF."""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "sim" / "worlds" / "cell_diverter.sdf"
SVG = ROOT / "docs" / "report" / "img" / "cell_layout.svg"
LAYOUT = ROOT / "docs" / "report" / "layout.md"
SVG_NS = {"svg": "http://www.w3.org/2000/svg"}

# Diagram mapping documented next to the SVG geometry.
PX_PER_M = 150.0
WORLD_X0_PX = 355.0
WORLD_Y0_PX = 440.0


def _guard_collision(name):
    root = ET.parse(WORLD).getroot()
    model = root.find(".//model[@name='belt_guides_camera']")
    assert model is not None
    collision = model.find(f".//collision[@name='{name}']")
    assert collision is not None
    pose = [float(v) for v in collision.find("pose").text.split()]
    size = [float(v) for v in collision.find("geometry/box/size").text.split()]
    return pose, size


@pytest.mark.parametrize(("side", "line_id"), [
    ("left", "camera-guard-left"),
    ("right", "camera-guard-right"),
])
def test_svg_camera_guards_are_projected_from_the_sdf(side, line_id):
    """A future SDF move must fail until the presentation drawing follows it."""
    (x, y, _, _, _, _), (length, width, _) = _guard_collision(side)
    svg = ET.parse(SVG).getroot()
    line = svg.find(f".//svg:line[@id='{line_id}']", SVG_NS)
    assert line is not None, f"{line_id} missing from the engineering drawing"

    assert float(line.get("x1")) == pytest.approx(WORLD_X0_PX + PX_PER_M * (x - length / 2))
    assert float(line.get("x2")) == pytest.approx(WORLD_X0_PX + PX_PER_M * (x + length / 2))
    expected_y = WORLD_Y0_PX - PX_PER_M * y
    assert float(line.get("y1")) == pytest.approx(expected_y)
    assert float(line.get("y2")) == pytest.approx(expected_y)
    assert float(line.get("stroke-width")) == pytest.approx(PX_PER_M * width)


def test_layout_table_names_both_asymmetric_guard_extents():
    text = LAYOUT.read_text(encoding="utf-8")
    assert "+Y: `x=0.70..2.43`" in text
    assert "−Y: `x=0.70..2.93`" in text
    assert "collision без visual" in text
