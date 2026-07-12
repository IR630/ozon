"""Safety floor markings must stay visible and physics-neutral in both worlds."""
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

WORLDS = Path(__file__).resolve().parents[1] / "sim" / "worlds"


@pytest.mark.parametrize("filename", ["cell.sdf", "cell_diverter.sdf"])
def test_safety_marking_is_visual_only_and_complete(filename):
    root = ET.parse(WORLDS / filename).getroot()
    model = root.find(".//model[@name='safety_marking']")
    assert model is not None
    assert model.findtext("static") == "true"
    assert model.findall(".//collision") == []

    names = {visual.attrib["name"] for visual in model.findall(".//visual")}
    assert names == {
        "hazard_north",
        "hazard_south",
        "hazard_infeed",
        "hazard_outfeed",
        "operator_safe_walkway",
    }


@pytest.mark.parametrize("filename", ["cell.sdf", "cell_diverter.sdf"])
def test_operator_walkway_is_outside_hazard_perimeter(filename):
    root = ET.parse(WORLDS / filename).getroot()
    model = root.find(".//model[@name='safety_marking']")
    north = model.find(".//visual[@name='hazard_north']")
    walkway = model.find(".//visual[@name='operator_safe_walkway']")
    north_y = float(north.findtext("pose").split()[1])
    walkway_pose = [float(v) for v in walkway.findtext("pose").split()]
    walkway_width = float(walkway.findtext("geometry/box/size").split()[1])
    assert walkway_pose[1] - walkway_width / 2 > north_y

