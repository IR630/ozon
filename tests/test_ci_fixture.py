"""The text-only CI item must match the production box's domain dimensions."""
import xml.etree.ElementTree as ET
from pathlib import Path


def test_ci_box_fixture_is_300x200x200_mm_and_1_5_kg():
    path = (Path(__file__).resolve().parents[1] / "sim" / "models" / "fixtures"
            / "box_300x200x200" / "model.sdf")
    root = ET.parse(path).getroot()
    assert [float(v) for v in root.findtext(".//collision/geometry/box/size").split()] == [
        0.3, 0.2, 0.2]
    assert float(root.findtext(".//mass")) == 1.5

