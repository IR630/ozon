# -*- coding: utf-8 -*-
"""Node-level test for perception (day 2, P3): depth Image -> ItemMeasurement.

Runs only where rclpy and the built ros_msgs overlay exist (WSL / ROS CI);
importorskip skips it elsewhere. A probe node publishes a synthetic 32FC1
depth frame (no cv_bridge needed) and asserts the measurement fields.
"""
import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
msgs = pytest.importorskip("ros_msgs.msg")

from sensor_msgs.msg import Image  # noqa: E402

from src.perception import BELT_DEPTH_M, FX  # noqa: E402
from src.perception_node import PerceptionNode  # noqa: E402

ItemMeasurement = msgs.ItemMeasurement


def _depth_image_msg():
    """Synthetic frame: belt at BELT_DEPTH_M, 100x100 px box top at 1.3 m."""
    depth = np.full((480, 640), BELT_DEPTH_M, dtype=np.float32)
    depth[100:200, 150:250] = 1.3
    msg = Image()
    msg.height, msg.width = depth.shape
    msg.encoding = "32FC1"
    msg.step = depth.shape[1] * 4
    msg.data = depth.tobytes()
    return msg


def _two_item_depth_image_msg():
    depth = np.full((480, 640), BELT_DEPTH_M, dtype=np.float32)
    depth[80:160, 100:180] = 1.30
    depth[280:370, 430:530] = 1.25
    msg = Image()
    msg.height, msg.width = depth.shape
    msg.encoding = "32FC1"
    msg.step = depth.shape[1] * 4
    msg.data = depth.tobytes()
    return msg


def test_depth_frame_yields_measurement():
    rclpy.init()
    try:
        node = PerceptionNode()
        probe = rclpy.create_node("probe")
        received = []
        probe.create_subscription(ItemMeasurement, "/item/measurement", received.append, 10)
        pub = probe.create_publisher(Image, "/camera/depth_image", 10)

        frame = _depth_image_msg()
        for _ in range(100):  # ~5 s budget; breaks as soon as the reply lands
            pub.publish(frame)
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(probe, timeout_sec=0.05)
            if received:
                break

        assert received, "perception published no ItemMeasurement"
        m = received[0]
        # laterals = 100 px * 1.3 / FX * 1000; height = (BELT_DEPTH_M - 1.3) * 1000
        lat = 100 * 1.3 / FX * 1000.0
        assert list(m.dims_mm) == pytest.approx([lat, lat, 200.0], abs=0.1)
        assert m.k == pytest.approx(1 / np.sqrt(2), abs=0.01)  # square top
        assert m.confidence == 1.0
        assert m.item_id == 1
        # centroid (199.5, 149.5) px -> world via the verified mapping
        assert m.position.x == pytest.approx(1.5 + (240 - 149.5) * 1.3 / FX, abs=1e-3)
        assert m.position.y == pytest.approx((320 - 199.5) * 1.3 / FX, abs=1e-3)

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_empty_belt_publishes_nothing():
    rclpy.init()
    try:
        node = PerceptionNode()
        probe = rclpy.create_node("probe")
        received = []
        probe.create_subscription(ItemMeasurement, "/item/measurement", received.append, 10)
        pub = probe.create_publisher(Image, "/camera/depth_image", 10)

        empty = Image()
        empty.height, empty.width = 480, 640
        empty.encoding = "32FC1"
        empty.step = 640 * 4
        empty.data = np.full((480, 640), BELT_DEPTH_M, dtype=np.float32).tobytes()
        for _ in range(10):
            pub.publish(empty)
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(probe, timeout_sec=0.05)

        assert received == []

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_a_side_frame_without_a_top_frame_publishes_nothing():
    """T3 as a test instead of as a reading of three places in the node.

    `docs/report/cameras.md` §8 names this exact gap: the claim "no redundancy at
    any head count" rests on reading `on_side_depth` (:80-94), `_side_clouds`
    (:96-115) and the single publish inside the top-frame handler (:163). Nothing
    machine-checked it, so an edit that gave a side head its own publish path
    would silently change the meaning of every number in that section.

    Both halves matter. The frame must be PARKED — the rig is wired and working —
    and the cell must still say nothing, because parking is not availability.
    """
    rclpy.init()
    try:
        node = PerceptionNode()
        probe = rclpy.create_node("probe_side_only")
        received = []
        probe.create_subscription(ItemMeasurement, "/item/measurement", received.append, 10)
        pub = probe.create_publisher(Image, "/camera_side_neg_y/depth_image", 10)

        # A side view carrying goods: belt at ~1 m with a body standing closer.
        depth = np.full((480, 640), 1.0, dtype=np.float32)
        depth[200:400, 250:450] = 0.85
        frame = Image()
        frame.height, frame.width = depth.shape
        frame.encoding = "32FC1"
        frame.step = depth.shape[1] * 4
        frame.data = depth.tobytes()

        for _ in range(20):
            pub.publish(frame)
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(probe, timeout_sec=0.05)

        assert node._side_frames, "side frame was not even parked — rig not wired"
        assert received == [], "a side head published a measurement on its own"

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_dump_dir_writes_only_measured_frames(tmp_path, monkeypatch):
    # Opt-in PERCEPTION_DUMP_DIR freezes the exact frames the node measures (day 11
    # validation set). A frame with an item writes one PNG; an empty belt writes
    # none (the `and measurements` guard), so the set never fills with belt frames.
    pytest.importorskip("cv2")
    monkeypatch.setenv("PERCEPTION_DUMP_DIR", str(tmp_path))
    rclpy.init()
    try:
        node = PerceptionNode()
        node.on_depth(_depth_image_msg())
        assert len(list(tmp_path.glob("depth_*.png"))) == 1

        empty = Image()
        empty.height, empty.width = 480, 640
        empty.encoding = "32FC1"
        empty.step = 640 * 4
        empty.data = np.full((480, 640), BELT_DEPTH_M, dtype=np.float32).tobytes()
        node.on_depth(empty)
        assert len(list(tmp_path.glob("depth_*.png"))) == 1  # empty belt adds nothing

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_dump_overlay_carries_ids_and_classifier_state(tmp_path, monkeypatch):
    # Day 9 debt: the dumped overlay names each item and shows the classifier's
    # last aggregation state for it; ids without a verdict yet read "unclassified".
    pytest.importorskip("cv2")
    monkeypatch.setenv("PERCEPTION_DUMP_DIR", str(tmp_path))
    rclpy.init()
    try:
        node = PerceptionNode()
        verdict = msgs.ItemClassification()
        verdict.item_id = 1
        verdict.category = "B"
        verdict.confidence = 0.9
        node.on_classification(verdict)
        assert node._agg_state[1] == "B conf=0.90"

        node.on_depth(_depth_image_msg())  # tracker assigns item_id 1
        assert len(list(tmp_path.glob("overlay_*.png"))) == 1
        assert len(list(tmp_path.glob("depth_*.png"))) == 1

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_two_disconnected_items_get_distinct_ids():
    rclpy.init()
    try:
        node = PerceptionNode()
        probe = rclpy.create_node("probe_multi")
        received = []
        probe.create_subscription(ItemMeasurement, "/item/measurement", received.append, 10)
        pub = probe.create_publisher(Image, "/camera/depth_image", 10)

        frame = _two_item_depth_image_msg()
        for _ in range(100):
            pub.publish(frame)
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(probe, timeout_sec=0.05)
            if len(received) >= 2:
                break

        assert len(received) >= 2
        first_frame = received[:2]
        assert {measurement.item_id for measurement in first_frame} == {1, 2}
        assert len({measurement.position.x for measurement in first_frame}) == 2

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()
