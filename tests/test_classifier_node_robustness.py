# -*- coding: utf-8 -*-
"""The classifier node must survive an out-of-contract measurement (audit 24.07).

classify() raises ValueError on dims outside [1, 1000] mm or K outside [0, 1]. In
the integrated pipeline perception guarantees sane values, but a stray/replayed
publisher on /item/measurement would otherwise throw straight out of the
subscription callback and take the node down. The node now catches it, logs, and
drops the frame. Runs only where rclpy exists (WSL / ROS CI).
"""
import pytest

rclpy = pytest.importorskip("rclpy")
msgs = pytest.importorskip("ros_msgs.msg")

from src.classifier_node import ClassifierNode  # noqa: E402  (after importorskip)

ItemClassification = msgs.ItemClassification
ItemMeasurement = msgs.ItemMeasurement


def test_out_of_contract_measurement_is_dropped_not_crashed():
    rclpy.init()
    node = probe = None
    try:
        node = ClassifierNode()
        probe = rclpy.create_node("probe")
        received = []
        probe.create_subscription(ItemClassification, "/item/classification",
                                  received.append, 10)
        pub = probe.create_publisher(ItemMeasurement, "/item/measurement", 10)

        bad = ItemMeasurement()
        bad.item_id = 7
        bad.dims_mm = [2000.0, 200.0, 200.0]  # > SANE_DIM_MM_MAX -> classify() raises
        bad.k = 0.5
        bad.confidence = 0.9

        # Before the fix the uncaught ValueError surfaced through spin_once and
        # crashed the run; now the frame is dropped and nothing is published.
        for _ in range(20):
            pub.publish(bad)
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(probe, timeout_sec=0.05)
        assert received == []
    finally:
        if node is not None:
            node.destroy_node()
        if probe is not None:
            probe.destroy_node()
        rclpy.shutdown()
