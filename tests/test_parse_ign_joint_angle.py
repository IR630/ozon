import pytest

from scripts.parse_ign_joint_angle import first_axis1_angle


def test_reads_explicit_nonzero_joint_position():
    text = """
joint {
  name: "diverter_joint"
  axis1 {
    xyz { x: 0 y: 0 z: 1 }
    position: 0.70858954909028959
    velocity: -0.01
  }
}
"""
    assert first_axis1_angle(text.splitlines()) == pytest.approx(0.70858954909028959)


def test_omitted_default_zero_does_not_steal_a_later_position():
    """Regression for mirror CI #310: the old grep returned x=4.829 as angle."""
    text = """
joint {
  name: "diverter_joint"
  axis1 {
    xyz { x: 0 y: 0 z: 1 }
    velocity: 0
  }
}
pose {
  position: 4.829
}
"""
    assert first_axis1_angle(text.splitlines()) == 0.0


def test_refuses_input_without_a_complete_axis_block():
    with pytest.raises(ValueError, match="no complete axis1 block"):
        first_axis1_angle(["pose {", "  position: 4.829", "}"])
