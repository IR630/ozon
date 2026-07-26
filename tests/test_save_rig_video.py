# -*- coding: utf-8 -*-
"""The rig video must not draw a head as working when it is not.

A composed frame is persuasive, which is exactly why it is dangerous: a panel
that renders a plausible ramp for a head that never published would put a
four-camera claim on screen with three cameras running. The two things asserted
here are the two that a viewer cannot check — that a silent head is drawn as
silent, and that "no depth returned" never borrows a colour from the range ramp.

Skipped outside the ROS environment (rclpy/cv_bridge), same guard as
tests/test_perception_node.py.
"""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

pytest.importorskip("rclpy")
pytest.importorskip("cv_bridge")

from save_rig_video import HEADS, PANEL_H, PANEL_W, _panel  # noqa: E402

from src.perception import BELT_DEPTH_M  # noqa: E402


def test_a_head_that_never_published_is_drawn_as_silent():
    """No frame must read as 'no frame', never as an empty but valid view."""
    panel = _panel(None, "SIDE  y=-0.90", False)
    assert panel.shape == (PANEL_H, PANEL_W, 3)
    # Text on black and nothing else: no ramp colour anywhere.
    assert panel.max() <= 200
    assert (panel[:, :, 0] == panel[:, :, 1]).all(), "a silent head got a colour ramp"


def test_pixels_with_no_return_stay_black_inside_a_live_panel():
    """Black is not a value on the scale — the figure's convention, kept here."""
    depth = np.full((60, 80), BELT_DEPTH_M, dtype=float)
    depth[10:30, 10:30] = BELT_DEPTH_M - 0.2   # an item
    depth[40:50, 40:60] = 0.0                  # a dropout: no return
    panel = _panel(depth, "TOP  z=1.90", True)
    assert panel.shape == (PANEL_H, PANEL_W, 3)
    # The dropout patch maps to the same fraction of the resized panel.
    y0, y1 = int(40 / 60 * PANEL_H) + 4, int(50 / 60 * PANEL_H) - 4
    x0, x1 = int(40 / 80 * PANEL_W) + 4, int(60 / 80 * PANEL_W) - 4
    assert panel[y0:y1, x0:x1].max() == 0, "a no-return region was given a colour"


def test_every_head_the_video_draws_is_a_head_the_rig_actually_has():
    """The panel list and the bridged topics must not drift apart."""
    bridged = (ROOT / "sim" / "bridge_3cam.yaml").read_text(encoding="utf-8")
    for topic, _label in HEADS:
        assert topic in bridged, f"{topic} is drawn but not bridged in bridge_3cam.yaml"


def test_the_poster_is_the_frame_with_the_most_goods_not_the_first(tmp_path):
    """The cover of a three-head clip must not be an empty belt.

    Two ways this went wrong before it was gated: the first composed frame shows
    three "no frame yet" panels, and the first frame after every head publishes
    still shows bare belt because the goods have not reached the camera. Both are
    correct inside the video and both advertise a rig that does nothing. Driven
    for real rather than inspected — the failure is invisible until someone opens
    the slide deck.
    """
    import rclpy
    from sensor_msgs.msg import Image

    from save_rig_video import RigVideoSaver

    def _scene_msg(h=45, w=80):
        msg = Image()
        msg.height, msg.width = h, w
        msg.encoding = "bgr8"
        msg.step = w * 3
        msg.data = np.full((h, w, 3), 90, np.uint8).tobytes()
        return msg

    def _depth_msg(near_px):
        """A top frame with `near_px` pixels standing over the belt."""
        depth = np.full((8, 8), BELT_DEPTH_M + 0.1, np.float32)
        depth.ravel()[:near_px] = BELT_DEPTH_M - 0.2
        msg = Image()
        msg.height, msg.width = depth.shape
        msg.encoding = "32FC1"
        msg.step = depth.shape[1] * 4
        msg.data = depth.tobytes()
        return msg

    poster = tmp_path / "poster.png"
    rclpy.init()
    try:
        saver = RigVideoSaver(str(tmp_path / "out.mp4"), 10.0, str(poster))
        saver.on_scene(_scene_msg())
        assert saver.frames == 1
        assert saver.best_frame is None, "a frame was chosen before any head published"

        for topic, _label in HEADS:
            saver.on_depth(_depth_msg(0), topic)
        saver.on_scene(_scene_msg())          # empty belt
        saver.on_depth(_depth_msg(20), HEADS[0][0])
        saver.on_scene(_scene_msg())          # goods in view
        saver.on_depth(_depth_msg(3), HEADS[0][0])
        saver.on_scene(_scene_msg())          # goods leaving
        assert saver.best_goods_px == 20, "the fullest frame was not the one kept"

        assert not poster.exists(), "poster written before the run ended"
        saver.write_poster()
        assert poster.exists()

        saver.writer.release()
        saver.destroy_node()
    finally:
        rclpy.shutdown()
