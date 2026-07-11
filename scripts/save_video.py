# -*- coding: utf-8 -*-
"""Record a ROS image topic into an mp4 (day 3, P5: video of the e2e run).

Frames arrive at the sensor's fixed SIM rate, so writing them 1:1 at the same
fps yields a video that plays in sim time no matter how RTF dips under the
extra render load. Runs inside the ROS env; started and stopped (SIGINT/SIGTERM)
by scripts/record_skeleton_video.sh:
    python3 scripts/save_video.py --topic /spectator/image --out run.mp4
"""
import argparse
import signal

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class VideoSaver(Node):
    def __init__(self, topic, out_path, fps, poster):
        super().__init__("video_saver")
        self.bridge = CvBridge()
        self.out_path, self.fps, self.poster = out_path, fps, poster
        self.writer = None
        self.frames = 0
        self.create_subscription(Image, topic, self.on_frame, 10)

    def on_frame(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.writer is None:
            h, w = img.shape[:2]
            self.writer = cv2.VideoWriter(
                self.out_path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h))
            if self.poster:
                cv2.imwrite(self.poster, img)
        self.writer.write(img)
        self.frames += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/spectator/image")
    parser.add_argument("--out", required=True)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--poster", default=None, help="also save the first frame as PNG")
    args = parser.parse_args()

    # the wrapper stops us with SIGTERM/SIGINT; either must release the writer,
    # otherwise the mp4 container ends up truncated
    signal.signal(signal.SIGTERM, signal.default_int_handler)

    rclpy.init()
    node = VideoSaver(args.topic, args.out, args.fps, args.poster)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass  # either way the writer must be released or the mp4 is truncated
    finally:
        if node.writer is not None:
            node.writer.release()
        print(f"{node.frames} frames -> {args.out}", flush=True)


if __name__ == "__main__":
    main()
