# -*- coding: utf-8 -*-
"""The noisy-world generator: the ways a "noisy" census would quietly be a clean one.

The output of this script is the input of a multi-hour census, so every failure
here is expensive and silent — a world noised on one head of three, a world with
two stacked noise models, a GUI viewpoint noised instead of a sensor. Each gets a
test by value.
"""
import pytest

from scripts.make_noisy_world import add_noise, cameras_present

_ONE_HEAD = """<sdf version="1.8"><world name="cell">
  <model name="cam"><link name="l">
    <sensor name="rgbd" type="rgbd_camera">
      <topic>camera</topic>
      <camera>
        <horizontal_fov>1.05</horizontal_fov>
      </camera>
    </sensor>
  </link></model>
</world></sdf>"""

_THREE_HEADS = _ONE_HEAD.replace(
    "</world>",
    """<model name="cam2"><link name="l">
    <sensor name="rgbd_side_neg_y" type="rgbd_camera"><camera></camera></sensor>
  </link></model>
  <model name="cam3"><link name="l">
    <sensor name="rgbd_side_pos_y" type="rgbd_camera"><camera></camera></sensor>
  </link></model></world>""")


def test_every_head_gets_the_noise_not_just_the_first():
    """A rig where only the top head is noisy would flatter the extra heads —
    the exact comparison these worlds exist to make."""
    out = add_noise(_THREE_HEADS, 0.015)
    assert out.count("<noise>") == 3
    assert out.count("<stddev>0.015</stddev>") == 3


def test_the_sigma_survives_the_file():
    """Sub-millimetre sigmas are the interesting ones (the probe says we hold
    0.75-1.5 mm); a %g round-trip that dropped digits would silently change the
    experiment."""
    out = add_noise(_ONE_HEAD, 0.00075)
    assert "<stddev>0.00075</stddev>" in out


def test_a_world_that_already_has_noise_is_refused():
    """Two <noise> blocks in one <camera> is undefined behaviour Ignition resolves
    silently — the census would then measure a model nobody chose."""
    once = add_noise(_ONE_HEAD, 0.015)
    with pytest.raises(ValueError, match="already declares"):
        add_noise(once, 0.002)


def test_a_gui_viewpoint_is_not_mistaken_for_a_sensor():
    """SDF allows <camera> under <gui>. Noising it would produce a world that
    reads as noisy, renders differently, and measures exactly like the clean one."""
    with_gui = _ONE_HEAD.replace(
        "<world name=\"cell\">",
        "<world name=\"cell\"><gui><camera><pose>0 0 5 0 0 0</pose></camera></gui>")
    with pytest.raises(ValueError, match="refusing an unattributable world"):
        add_noise(with_gui, 0.015)


def test_the_production_world_is_the_clean_one():
    """The census baseline must stay noise-free: if the shipped world ever gained
    a noise model, every historical 33/33 would silently change meaning."""
    from pathlib import Path

    world = Path(__file__).resolve().parents[1] / "sim" / "worlds" / "cell_diverter.sdf"
    text = world.read_text(encoding="utf-8")
    assert "<noise>" not in text
    assert cameras_present(text) == ("rgbd",)
