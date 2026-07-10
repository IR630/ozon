# -*- coding: utf-8 -*-
"""Perception ROS 2 node: depth frames -> ItemMeasurement (day 2, P3).

Thin wrapper. The geometry lives once in src.perception.measure_item(); this
node only decodes sensor_msgs/Image (32FC1, meters — what ros_gz_bridge
publishes for the gz depth camera) and moves the measurement onto the wire.

Intrinsics: prefers fx/fy from /camera/camera_info at runtime; until the first
CameraInfo arrives, falls back to the offline calibration in src.perception.

item_id v0: one item is visible at a time (task flow), so a new id is assigned
whenever an item appears after empty frames. Real multi-item tracking is out
of scope for the skeleton.

Runs inside the ROS 2 environment (needs rclpy and the built ros_msgs overlay):
    python3 -m src.perception_node
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from ros_msgs.msg import ItemMeasurement

from src.perception import measure_item

_GAP_FRAMES = 2  # empty frames before the next detection counts as a new item


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception")
        self.fx = None  # from CameraInfo; None -> perception.py defaults
        self.fy = None
        self.item_id = 0
        self.gap = _GAP_FRAMES  # start "empty long enough": first item gets a fresh id
        self.pub = self.create_publisher(ItemMeasurement, "/item/measurement", 10)
        self.create_subscription(Image, "/camera/depth_image", self.on_depth, 10)
        self.create_subscription(CameraInfo, "/camera/camera_info", self.on_info, 10)

    def on_info(self, msg):
        self.fx = float(msg.k[0])  # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self.fy = float(msg.k[4])

    def on_depth(self, msg):
        if msg.encoding != "32FC1":
            self.get_logger().error(f"expected 32FC1 depth, got {msg.encoding}", once=True)
            return
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)

        kwargs = {}
        if self.fx is not None:
            kwargs = {"fx": self.fx, "fy": self.fy}
        m = measure_item(depth.astype(np.float64), **kwargs)
        if m is None:
            self.gap += 1
            return
        if self.gap >= _GAP_FRAMES:
            self.item_id += 1
        self.gap = 0

        out = ItemMeasurement()
        out.header.stamp = msg.header.stamp  # measurement time = frame time
        out.header.frame_id = "world"
        out.item_id = self.item_id
        out.dims_mm = [float(d) for d in m.dims_mm]
        out.k = m.k
        out.confidence = 1.0  # day 4: uncertainty policy will lower this
        out.position.x, out.position.y, out.position.z = m.position_m
        self.pub.publish(out)
        self.get_logger().info(
            f"item {out.item_id}: {out.dims_mm[0]:.0f}x{out.dims_mm[1]:.0f}x{out.dims_mm[2]:.0f} mm"
            f" K={out.k:.2f} at ({out.position.x:.2f}, {out.position.y:.2f})"
        )


def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
