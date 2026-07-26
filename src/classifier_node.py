# -*- coding: utf-8 -*-
"""Classifier ROS 2 node: ItemMeasurement -> ItemClassification (day 2 / day 4, P4).

The rules live once in src.classification (single source of truth); this node
moves the decision onto the wire. Day 4: it no longer reacts to each frame.
It aggregates the measurements of an item by item_id (src.aggregation) and
classifies the running MEDIAN dims/K with the conservative size policy
(classify_conservative). The published dims/K/confidence are the aggregate, so
the category is stable and the flip patches in the controller become a safety
net rather than the fix.

Runs inside the ROS 2 environment (needs rclpy and the built ros_msgs overlay):
    python3 src/classifier_node.py
"""
import rclpy
from rclpy.node import Node

from ros_msgs.msg import ItemClassification, ItemMeasurement

from src.aggregation import ItemAggregator
from src.classification import classify_conservative


class ClassifierNode(Node):
    def __init__(self):
        super().__init__("classifier")
        self.agg = ItemAggregator()
        self.pub = self.create_publisher(ItemClassification, "/item/classification", 10)
        self.create_subscription(ItemMeasurement, "/item/measurement", self.on_measurement, 10)

    def on_measurement(self, msg):
        try:
            # Validate before aggregation: a bad frame must not poison the running
            # median for later valid frames carrying the same item_id.
            classify_conservative(msg.dims_mm, msg.k)
        except ValueError as error:
            self.get_logger().error(
                f"item {msg.item_id}: skipped bad measurement — {error}")
            return

        est = self.agg.update(msg.item_id, msg.dims_mm, msg.k)
        out = ItemClassification()
        out.header = msg.header
        out.item_id = msg.item_id
        out.dims_mm = [float(d) for d in est.dims_mm]
        out.k = est.k
        out.confidence = est.confidence
        out.position = msg.position
        out.category = classify_conservative(est.dims_mm, est.k)
        self.pub.publish(out)
        self.get_logger().info(
            f"item {msg.item_id}: {out.category} "
            f"(k={est.k:.3f}, conf={est.confidence:.2f}, n={est.n_frames})")


def main():
    rclpy.init()
    node = ClassifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
