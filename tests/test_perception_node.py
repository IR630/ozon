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
