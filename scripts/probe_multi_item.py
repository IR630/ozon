#!/usr/bin/env python3
"""Exit successfully after one camera stamp contains at least two item IDs."""
import time

import rclpy
from rclpy.node import Node

from ros_msgs.msg import ItemMeasurement


def main():
    rclpy.init()
    node = Node("multi_item_probe")
    by_stamp = {}
    success = []

    def on_measurement(msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        item_ids = by_stamp.setdefault(stamp, set())
        item_ids.add(int(msg.item_id))
        if len(item_ids) >= 2:
            success.append((stamp, sorted(item_ids)))

    node.create_subscription(ItemMeasurement, "/item/measurement", on_measurement, 20)
    deadline = time.monotonic() + 90.0
    while time.monotonic() < deadline and not success:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
    if not success:
        raise SystemExit("no frame contained two distinct item IDs")
    stamp, item_ids = success[0]
    print(f"simultaneous frame stamp={stamp[0]}.{stamp[1]:09d}, ids={item_ids}")


if __name__ == "__main__":
    main()
