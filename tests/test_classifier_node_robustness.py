# -*- coding: utf-8 -*-
"""The classifier node must reject bad input before it reaches aggregation."""
import pytest

rclpy = pytest.importorskip("rclpy")
msgs = pytest.importorskip("ros_msgs.msg")

from src.classifier_node import ClassifierNode  # noqa: E402  (after importorskip)

ItemClassification = msgs.ItemClassification
ItemMeasurement = msgs.ItemMeasurement


def _spin_pair(node, probe, pub, message, received, attempts=20):
    for _ in range(attempts):
        pub.publish(message)
        rclpy.spin_once(node, timeout_sec=0.05)
        rclpy.spin_once(probe, timeout_sec=0.05)
        if received:
            break


def test_bad_measurement_is_dropped_without_poisoning_item_history():
    rclpy.init()
    node = probe = None
    try:
        node = ClassifierNode()
        probe = rclpy.create_node("classifier_robustness_probe")
        received = []
        probe.create_subscription(
            ItemClassification, "/item/classification", received.append, 10)
        pub = probe.create_publisher(ItemMeasurement, "/item/measurement", 10)

        bad = ItemMeasurement()
        bad.item_id = 7
        bad.dims_mm = [2000.0, 200.0, 200.0]
        bad.k = 0.5
        node.on_measurement(bad)
        assert received == []
        assert bad.item_id not in node.agg._dims

        good = ItemMeasurement()
        good.item_id = bad.item_id
        good.dims_mm = [300.0, 200.0, 200.0]
        good.k = 0.5
        _spin_pair(node, probe, pub, good, received)

        assert received, "a valid frame after a bad one was not classified"
        assert list(received[0].dims_mm) == [300.0, 200.0, 200.0]
        assert received[0].category == "B"
    finally:
        if node is not None:
            node.destroy_node()
        if probe is not None:
            probe.destroy_node()
        rclpy.shutdown()
