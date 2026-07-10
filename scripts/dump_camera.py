# -*- coding: utf-8 -*-
"""Dump RGB and depth frames from the simulated camera to PNG files.

Runs INSIDE the Docker environment (needs rclpy, cv_bridge, the running
world and the ros_gz bridge — see docker/ and sim/bridge.yaml):
    python3 scripts/dump_camera.py --out /ws/out --frames 5
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraDumper(Node):
    def __init__(self, out_dir, max_frames):
        super().__init__("camera_dumper")
        self.bridge = CvBridge()
        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.max_frames = max_frames
        self.counts = {"rgb": 0, "depth": 0}
        self.create_subscription(Image, "/camera/image", lambda m: self.save(m, "rgb"), 10)
        self.create_subscription(Image, "/camera/depth_image", lambda m: self.save(m, "depth"), 10)

    def save(self, msg, kind):
        if self.counts[kind] >= self.max_frames:
            return
        img = self.bridge.imgmsg_to_cv2(msg)
        if kind == "depth":
            # 32FC1 meters -> 16-bit PNG in millimeters, NaN/inf -> 0
            img = np.nan_to_num(img, nan=0.0, posinf=0.0, neginf=0.0)
            img = (img * 1000.0).clip(0, 65535).astype(np.uint16)
        path = self.out / f"{kind}_{self.counts[kind]:03d}.png"
        cv2.imwrite(str(path), img)
        self.counts[kind] += 1
        self.get_logger().info(f"saved {path}")

    def done(self):
        return all(c >= self.max_frames for c in self.counts.values())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="out")
    parser.add_argument("--frames", type=int, default=5)
    args = parser.parse_args()

    rclpy.init()
    node = CameraDumper(args.out, args.frames)
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
