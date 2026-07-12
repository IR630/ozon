# -*- coding: utf-8 -*-
"""Perception ROS 2 node: depth frames -> ItemMeasurement (day 2, P3).

Thin wrapper. The geometry lives once in src.perception.measure_item(); this
node only decodes sensor_msgs/Image (32FC1, meters — what ros_gz_bridge
publishes for the gz depth camera) and moves the measurement onto the wire.

Intrinsics: prefers fx/fy from /camera/camera_info at runtime; until the first
CameraInfo arrives, falls back to the offline calibration in src.perception.

Disconnected items are measured independently. A lightweight world-position
tracker keeps item_id stable across frames and short detection gaps.

Runs inside the ROS 2 environment (needs rclpy and the built ros_msgs overlay):
    python3 -m src.perception_node
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from ros_msgs.msg import ItemMeasurement

from src.item_tracking import ItemTracker
from src.perception import measure_items


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception")
        self.fx = None  # from CameraInfo; None -> perception.py defaults
        self.fy = None
        self.cx = None  # principal point from CameraInfo; None -> image center
        self.cy = None
        self.tracker = ItemTracker()
        self.pub = self.create_publisher(ItemMeasurement, "/item/measurement", 10)
        self.create_subscription(Image, "/camera/depth_image", self.on_depth, 10)
        self.create_subscription(CameraInfo, "/camera/camera_info", self.on_info, 10)

    def on_info(self, msg):
        self.fx = float(msg.k[0])  # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def on_depth(self, msg):
        if msg.encoding != "32FC1":
            self.get_logger().error(f"expected 32FC1 depth, got {msg.encoding}", once=True)
            return
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)

        kwargs = {}
        if self.fx is not None:
            kwargs = {"fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy}
        measurements = measure_items(depth.astype(np.float64), **kwargs)
        item_ids = self.tracker.update([measurement.position_m for measurement in measurements])
        for item_id, measurement in zip(item_ids, measurements):
            out = ItemMeasurement()
            out.header.stamp = msg.header.stamp  # measurement time = frame time
            out.header.frame_id = "world"
            out.item_id = item_id
            out.dims_mm = [float(d) for d in measurement.dims_mm]
            out.k = measurement.k
            out.confidence = 1.0  # classifier aggregation computes decision confidence
            out.position.x, out.position.y, out.position.z = measurement.position_m
            self.pub.publish(out)
            self.get_logger().info(
                f"item {out.item_id}: {out.dims_mm[0]:.0f}x{out.dims_mm[1]:.0f}x"
                f"{out.dims_mm[2]:.0f} mm K={out.k:.2f} at "
                f"({out.position.x:.2f}, {out.position.y:.2f})"
            )


def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
