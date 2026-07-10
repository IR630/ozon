# -*- coding: utf-8 -*-
"""Node-level integration test for the classifier (day 2, P4).

Runs only where rclpy and the built ros_msgs overlay exist (WSL / ROS CI);
importorskip skips it on the plain pytest CI job and on the Windows dev host.
A probe node acts as the mock perception publisher and the mock controller
subscriber, so the classifier is exercised over the real ROS graph.
"""
import pytest

rclpy = pytest.importorskip("rclpy")
msgs = pytest.importorskip("ros_msgs.msg")

from src.classifier_node import ClassifierNode  # noqa: E402  (after importorskip)

ItemClassification = msgs.ItemClassification
ItemMeasurement = msgs.ItemMeasurement


def test_box_300_measurement_routes_to_B():
    rclpy.init()
    try:
        node = ClassifierNode()
        probe = rclpy.create_node("probe")
        received = []
        probe.create_subscription(ItemClassification, "/item/classification", received.append, 10)
        pub = probe.create_publisher(ItemMeasurement, "/item/measurement", 10)

        m = ItemMeasurement()
        m.item_id = 1
        m.dims_mm = [300.0, 200.0, 200.0]  # Короб 300: fits sorter, square section
        m.k = 0.71  # < 0.8 -> not round
        m.confidence = 0.9

        for _ in range(100):  # ~5 s budget; breaks as soon as the reply lands
            pub.publish(m)
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(probe, timeout_sec=0.05)
            if received:
                break

        assert received, "classifier published no ItemClassification"
        out = received[0]
        assert out.category == "B"
        assert out.item_id == 1
        assert list(out.dims_mm) == [300.0, 200.0, 200.0]
        assert out.confidence == pytest.approx(0.9, abs=1e-6)

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()
