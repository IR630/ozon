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
import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from ros_msgs.msg import ItemClassification, ItemMeasurement

from src.item_tracking import ItemTracker
from src.perception import (
    FX,
    FY,
    SIDE_SYNC_MAX_DT_S,
    measure_items,
    save_items_overlay,
    side_cloud_from_frame,
)


def _stamp_s(stamp):
    """ROS Time message -> float seconds (sim clock), for inter-frame dt."""
    return stamp.sec + stamp.nanosec * 1e-9

# Aggregation states remembered for the debug overlay; same spirit as
# aggregation.MAX_TRACKED_ITEMS — bounded, ample for a sequential belt.
_MAX_OVERLAY_STATES = 32


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception")
        self.fx = None  # from CameraInfo; None -> perception.py defaults
        self.fy = None
        self.cx = None  # principal point from CameraInfo; None -> image center
        self.cy = None
        # Side head (feat/two-cameras): its own intrinsics and the last frame it sent,
        # buffered as (depth64, stamp_s). None until it publishes; if it never does (or
        # its stream drops), on_depth fuses nothing and the node is bit-identical to the
        # single-camera main — the branch's hard availability contract.
        self.side_fx = None
        self.side_fy = None
        self.side_cx = None
        self.side_cy = None
        self._side_frame = None
        self.tracker = ItemTracker()
        # Opt-in: freeze the exact depth frames this node measures, to build the
        # day-11 validation set from real Gazebo frames (Karpathy #1). Off unless
        # PERCEPTION_DUMP_DIR is set, so production runs are untouched.
        self._dump_dir = os.environ.get("PERCEPTION_DUMP_DIR")
        self._dump_n = 0
        # Last classifier verdict per item_id, shown on the dumped overlay so the
        # frame carries the aggregation STATE, not just the geometry (day 9 debt).
        self._agg_state = {}
        self.pub = self.create_publisher(ItemMeasurement, "/item/measurement", 10)
        self.create_subscription(Image, "/camera/depth_image", self.on_depth, 10)
        self.create_subscription(CameraInfo, "/camera/camera_info", self.on_info, 10)
        self.create_subscription(Image, "/camera_side/depth_image", self.on_side_depth, 10)
        self.create_subscription(CameraInfo, "/camera_side/camera_info", self.on_side_info, 10)
        self.create_subscription(
            ItemClassification, "/item/classification", self.on_classification, 10)

    def on_classification(self, msg):
        if msg.item_id not in self._agg_state and len(self._agg_state) >= _MAX_OVERLAY_STATES:
            self._agg_state.pop(next(iter(self._agg_state)))
        self._agg_state[msg.item_id] = f"{msg.category} conf={msg.confidence:.2f}"

    def on_info(self, msg):
        self.fx = float(msg.k[0])  # K = [fx 0 cx; 0 fy cy; 0 0 1]
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def on_side_info(self, msg):
        self.side_fx = float(msg.k[0])
        self.side_fy = float(msg.k[4])
        self.side_cx = float(msg.k[2])
        self.side_cy = float(msg.k[5])

    def on_side_depth(self, msg):
        if msg.encoding != "32FC1":
            self.get_logger().error(f"expected 32FC1 side depth, got {msg.encoding}", once=True)
            return
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        self._side_frame = (depth.astype(np.float64), _stamp_s(msg.header.stamp))

    def _side_points_for(self, top_stamp_s):
        """World flank cloud from the buffered side frame, or None (no fresh side data).

        None whenever the side stream is absent or stale, so measure_items falls back to
        top-only and the published measurement is bit-identical to the single-camera main.
        """
        if self._side_frame is None:
            return None
        depth_side, side_stamp_s = self._side_frame
        dt_s = side_stamp_s - top_stamp_s
        if abs(dt_s) > SIDE_SYNC_MAX_DT_S:
            return None
        fx = self.side_fx if self.side_fx is not None else FX
        fy = self.side_fy if self.side_fy is not None else FY
        return side_cloud_from_frame(depth_side, dt_s, fx, fy, self.side_cx, self.side_cy)

    def on_depth(self, msg):
        if msg.encoding != "32FC1":
            self.get_logger().error(f"expected 32FC1 depth, got {msg.encoding}", once=True)
            return
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)

        kwargs = {}
        if self.fx is not None:
            kwargs = {"fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy}
        depth64 = depth.astype(np.float64)
        side_points = self._side_points_for(_stamp_s(msg.header.stamp))
        measurements = measure_items(depth64, side_points_world_m=side_points, **kwargs)
        item_ids = self.tracker.update([measurement.position_m for measurement in measurements])
        if self._dump_dir and measurements:
            self._dump_frame(depth64, measurements, item_ids)
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


    def _dump_frame(self, depth64, measurements, item_ids):
        """Save one measured depth frame as a uint16-mm PNG (load_depth_png format),
        tagged with the frame index and the K of its largest item for easy picking,
        plus the human-readable overlay with per-item id and aggregation state."""
        import os as _os

        from src.perception import save_depth_png

        _os.makedirs(self._dump_dir, exist_ok=True)
        k = max(measurements, key=lambda m: m.dims_mm[0] * m.dims_mm[1]).k
        path = _os.path.join(self._dump_dir, f"depth_{self._dump_n:03d}_k{k:.2f}.png")
        # Unicode-safe (a participant may run under a non-ASCII username); the
        # overlay below is already Unicode-safe, the depth PNG must match.
        save_depth_png(depth64, path)
        tagged = [(item_id, m, self._agg_state.get(item_id))
                  for item_id, m in zip(item_ids, measurements)]
        save_items_overlay(
            depth64, tagged,
            _os.path.join(self._dump_dir, f"overlay_{self._dump_n:03d}.png"))
        self._dump_n += 1


def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
