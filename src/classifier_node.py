# -*- coding: utf-8 -*-
"""Classifier ROS 2 node: ItemMeasurement -> ItemClassification (day 2, P4).

Thin wrapper. The rules live once in src.classification.classify() (single
source of truth); this node only moves that decision onto the wire. The
uncertainty / conservative policy is day 4 — here confidence passes through
from the measurement unchanged.

Runs inside the ROS 2 environment (needs rclpy and the built ros_msgs overlay):
    python3 src/classifier_node.py
"""
import rclpy
from rclpy.node import Node

from ros_msgs.msg import ItemClassification, ItemMeasurement

from src.classification import classify


class ClassifierNode(Node):
    def __init__(self):
        super().__init__("classifier")
        self.pub = self.create_publisher(ItemClassification, "/item/classification", 10)
        self.create_subscription(ItemMeasurement, "/item/measurement", self.on_measurement, 10)

    def on_measurement(self, msg):
        out = ItemClassification()
        out.header = msg.header
        out.item_id = msg.item_id
        out.dims_mm = msg.dims_mm
        out.k = msg.k
        out.confidence = msg.confidence  # day 4: uncertainty policy will adjust routing
        out.position = msg.position
        out.category = classify(msg.dims_mm, msg.k)
        self.pub.publish(out)
        self.get_logger().info(f"item {msg.item_id}: {out.category} (k={msg.k:.3f})")


def main():
    rclpy.init()
    node = ClassifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
