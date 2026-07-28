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

from src.constants import (
    CAMERA_FRAME_PERIOD_S,
    CAMERA_SIDE_NEG_Y_POSE_M,
    CAMERA_SIDE_POS_Y_POSE_M,
    CAMERA_SIDE_STALE_FRAMES,
)
from src.item_tracking import ItemTracker
from src.multiview import (
    compensate_belt_travel,
    crop_to_item,
    fuse_dims_mm,
    world_cloud_from_depth,
)
from src.perception import FX, FY, measure_items, save_items_overlay

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
        self.tracker = ItemTracker()
        # Opt-in: freeze the exact depth frames this node measures, to build the
        # day-11 validation set from real Gazebo frames (Karpathy #1). Off unless
        # PERCEPTION_DUMP_DIR is set, so production runs are untouched.
        self._dump_dir = os.environ.get("PERCEPTION_DUMP_DIR")
        self._dump_n = 0
        # Last classifier verdict per item_id, shown on the dumped overlay so the
        # frame carries the aggregation STATE, not just the geometry (day 9 debt).
        self._agg_state = {}
        # Latest frame from each side head, keyed by its rig pose: {pose: (t, depth)}.
        # The top head drives the pipeline, so side frames only ever wait here to be
        # picked up by the next top frame — there is no second detector.
        self._side_frames = {}
        self.pub = self.create_publisher(ItemMeasurement, "/item/measurement", 10)
        self.create_subscription(Image, "/camera/depth_image", self.on_depth, 10)
        self.create_subscription(CameraInfo, "/camera/camera_info", self.on_info, 10)
        self.create_subscription(
            ItemClassification, "/item/classification", self.on_classification, 10)
        for topic, pose in (("/camera_side_neg_y/depth_image", CAMERA_SIDE_NEG_Y_POSE_M),
                            ("/camera_side_pos_y/depth_image", CAMERA_SIDE_POS_Y_POSE_M)):
            self.create_subscription(
                Image, topic, lambda msg, pose=pose: self.on_side_depth(msg, pose), 10)

    @staticmethod
    def _stamp_s(header):
        return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9

    def on_side_depth(self, msg, pose):
        """Park one side head's frame until the next top frame consumes it.

        Nothing is measured here on purpose: a side view has no belt plane to
        segment against, so giving it its own detector would put three
        disagreeing detectors in a cell that needs one.
        """
        if msg.encoding != "32FC1":
            self.get_logger().error(
                f"side head: expected 32FC1 depth, got {msg.encoding}", once=True)
            return
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        self._side_frames[pose] = (self._stamp_s(msg.header),
                                   np.nan_to_num(depth, nan=0.0, posinf=0.0,
                                                 neginf=0.0).astype(np.float64))

    def _side_world_clouds(self, t_top, fx, fy, cx, cy):
        """World clouds of the side heads, belt-travel compensated. ONCE PER FRAME.

        Backprojection and travel compensation depend on the head and the frame,
        never on which body we are about to measure, so they are hoisted out of the
        per-item loop. They used to sit inside it, and the cost was (bodies x full
        frame): with one body that is 46 ms, but sensor noise is exactly what makes
        the body count explode — 249 bodies at sigma 10 mm turned a 42 ms
        backprojection into 10.5 s per frame, on the rig with side heads only. The
        cropping that DOES depend on the item stays per item, below.

        A head that has gone quiet for more than STALE_FRAMES periods is dropped
        rather than extrapolated: at 1 m/s a stale frame is not slightly wrong, it
        is describing a different piece of belt. Losing a head must degrade the
        measurement to the top view, never fail the node.
        """
        clouds = []
        for pose, (t_side, depth) in self._side_frames.items():
            dt = t_top - t_side
            if abs(dt) > CAMERA_SIDE_STALE_FRAMES * CAMERA_FRAME_PERIOD_S:
                self.get_logger().warn(
                    f"side head {pose[0]} stale by {dt * 1000:.0f} ms, dropped",
                    throttle_duration_sec=5.0)
                continue
            pts = world_cloud_from_depth(depth, pose, fx, fy, cx, cy)
            clouds.append(compensate_belt_travel(pts, dt))
        return clouds

    def on_classification(self, msg):
        if msg.item_id not in self._agg_state and len(self._agg_state) >= _MAX_OVERLAY_STATES:
            self._agg_state.pop(next(iter(self._agg_state)))
        self._agg_state[msg.item_id] = f"{msg.category} conf={msg.confidence:.2f}"

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
        depth64 = depth.astype(np.float64)
        measurements = measure_items(depth64, **kwargs)
        item_ids = self.tracker.update([measurement.position_m for measurement in measurements])
        if self._dump_dir and measurements:
            self._dump_frame(depth64, measurements, item_ids)
        t_top = self._stamp_s(msg.header)
        fx = self.fx if self.fx is not None else FX
        fy = self.fy if self.fy is not None else FY
        # `and measurements` is not a micro-optimisation. Hoisting the backprojection
        # out of the per-item loop made it unconditional, and the belt is EMPTY most
        # of the time between items — so every idle frame paid one full-frame
        # backprojection PER SIDE HEAD (~45 ms each, 15 Hz, forever), and a 3-head rig
        # paid it twice. Nothing to fuse into means nothing to backproject.
        side_world = (self._side_world_clouds(t_top, fx, fy, self.cx, self.cy)
                      if self._side_frames and measurements else [])
        for item_id, measurement in zip(item_ids, measurements):
            dims_mm = measurement.dims_mm
            n_side = 0
            if side_world:
                clouds = [crop_to_item(pts, measurement.position_m, measurement.dims_mm)
                          for pts in side_world]
                n_side = sum(1 for c in clouds if c is not None and len(c) >= 4)
                dims_mm = fuse_dims_mm(dims_mm, clouds, measurement.position_m)
            out = ItemMeasurement()
            out.header.stamp = msg.header.stamp  # measurement time = frame time
            out.header.frame_id = "world"
            out.item_id = item_id
            out.dims_mm = [float(d) for d in dims_mm]
            # K stays TOP-ONLY, deliberately. A side head sees the end circle of
            # anything lying down and would route every prone body to D.
            out.k = measurement.k
            out.confidence = 1.0  # classifier aggregation computes decision confidence
            out.position.x, out.position.y, out.position.z = measurement.position_m
            self.pub.publish(out)
            # heads=N is not decoration. A rig whose side streams never arrive
            # degrades silently to the top view and still prints a plausible
            # measurement, so a whole census could be reported as three-camera
            # while being single-camera. The count makes that visible in the log.
            # K to FOUR decimals, not two. The flatness gate clamps a thick round
            # body's K to exactly ROUND_K_THRESHOLD, and the category rule then
            # tests that same boundary strictly — so "0.80" in a log is ambiguous
            # between the safe clamped value and the miss just above it. The two
            # multi-seed misses both printed K=0.80 and routed D, and no log we
            # own can say by how much. Four decimals cost nothing and are the
            # difference between a diagnosis and a guess. Parsers read [\d.]+.
            self.get_logger().info(
                f"item {out.item_id}: {out.dims_mm[0]:.0f}x{out.dims_mm[1]:.0f}x"
                f"{out.dims_mm[2]:.0f} mm K={out.k:.4f} at "
                f"({out.position.x:.2f}, {out.position.y:.2f}) heads={1 + n_side}"
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
