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


def test_a_side_frame_is_backprojected_once_per_frame_not_once_per_body(monkeypatch):
    """The rig's cost must scale with HEADS, not with how many bodies noise invents.

    Backprojection depends on the head and the frame, never on the body, but it used
    to sit inside the per-item loop. Sensor noise is exactly what multiplies bodies
    (249 at sigma 10 mm on a dump), so that loop turned a 42 ms backprojection into
    10.5 s per frame — on the multi-head rig only, which is how a two-head census
    lost 15 cells to the cell cap while measuring correctly.
    """
    import src.perception_node as node_mod
    from src.constants import CAMERA_SIDE_NEG_Y_POSE_M

    calls = []
    real = node_mod.world_cloud_from_depth

    def counting(depth_m, pose, *args, **kwargs):
        calls.append(pose)
        return real(depth_m, pose, *args, **kwargs)

    monkeypatch.setattr(node_mod, "world_cloud_from_depth", counting)

    rclpy.init()
    try:
        node = PerceptionNode()
        # Park one side frame the way on_side_depth would, stamped with the top
        # frame's time so the staleness gate keeps it.
        side = np.full((480, 640), 1.2, dtype=np.float64)
        node._side_frames[CAMERA_SIDE_NEG_Y_POSE_M] = (0.0, side)

        msg = _two_item_depth_image_msg()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        node.on_depth(msg)

        assert len(calls) == 1, (
            f"one side head and one top frame must cost ONE backprojection, "
            f"got {len(calls)} — the call is back inside the per-item loop")

        # ...and an EMPTY belt must cost none at all. Hoisting the call out of the
        # per-item loop made it unconditional, so idle frames — which is most frames
        # between items — paid a full-frame backprojection per side head forever.
        calls.clear()
        empty = _depth_image_msg()
        empty_depth = np.full((480, 640), BELT_DEPTH_M, dtype=np.float32)
        empty.data = empty_depth.tobytes()
        empty.header.stamp.sec = 0
        empty.header.stamp.nanosec = 0
        node.on_depth(empty)
        assert calls == [], (
            f"an empty belt must not backproject any side frame, got {len(calls)}")
        node.destroy_node()
    finally:
        rclpy.shutdown()
