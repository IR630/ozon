# -*- coding: utf-8 -*-
"""Record the THREE-HEAD cell as one frame: the scene plus what every head sees.

WHY A SEPARATE SAVER. `scripts/save_video.py` records one image topic and is what
every shipped clip was made with; it stays untouched. The rig needs something it
cannot do — show the heads AND their views at the same instant, because the whole
camera-count argument is about what a head contributes, and no scene shot can
show that. Every clip in `docs/report/video/` was filmed in a ONE-camera world,
so the rig has never been on screen at all.

WHAT DRIVES THE CLOCK. The spectator frame does. The three depth heads are
UNTRIGGERED at 15 Hz (`src/constants.py`, CAMERA_FRAME_PERIOD_S), so a composed
frame carries each head's most recent depth, which is what the node itself works
with — up to a frame period apart, and compensated by belt travel rather than by
synchronisation. The overlay says so on every frame rather than leaving a viewer
to assume a synchronised rig.

DEPTH IS DRAWN THE WAY THE REPORT DRAWS IT. `plot_head_views._colorize` is
imported rather than re-implemented so the figure and the video cannot disagree
about the one convention that matters: black is "the sensor returned nothing",
and it is never a colour on the range ramp.

Started and stopped by scripts/record_rig_video.sh; SIGINT/SIGTERM must release
the writer or the mp4 ends up truncated.
"""
import argparse
import signal
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from plot_head_views import _colorize  # noqa: E402

from src.perception import BELT_DEPTH_M  # noqa: E402

PANEL_W, PANEL_H = 400, 300
SCENE_W = 3 * PANEL_W
FONT = cv2.FONT_HERSHEY_SIMPLEX

# (topic, label) in the order they are drawn, left to right.
HEADS = (("/camera/depth_image", "TOP  z=1.90"),
         ("/camera_side_neg_y/depth_image", "SIDE  y=-0.90"),
         ("/camera_side_pos_y/depth_image", "SIDE  y=+0.90"))


def _panel(depth_m, label):
    """One colorized head panel with its label and its live pixel count."""
    if depth_m is None:
        img = np.zeros((PANEL_H, PANEL_W, 3), np.uint8)
        cv2.putText(img, f"{label}: no frame yet", (12, PANEL_H // 2), FONT, 0.5,
                    (200, 200, 200), 1, cv2.LINE_AA)
        return img
    valid = depth_m > 0.0
    finite = depth_m[valid]
    if finite.size:
        img = _colorize(depth_m, valid, float(np.percentile(finite, 1)),
                        float(np.percentile(finite, 99)))
    else:
        img = np.zeros(depth_m.shape + (3,), np.uint8)
    img = cv2.resize(img, (PANEL_W, PANEL_H), interpolation=cv2.INTER_AREA)
    over_belt = int((valid & (depth_m < BELT_DEPTH_M)).sum())
    cv2.putText(img, label, (10, 24), FONT, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(img, f"{int(valid.sum())} px depth / {over_belt} over belt",
                (10, PANEL_H - 12), FONT, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    return img


class RigVideoSaver(Node):
    def __init__(self, out_path, fps, poster):
        super().__init__("rig_video_saver")
        self.bridge = CvBridge()
        self.out_path, self.fps, self.poster = out_path, fps, poster
        self.writer = None
        self.frames = 0
        self.depths = {topic: None for topic, _label in HEADS}
        for topic, _label in HEADS:
            self.create_subscription(
                Image, topic, lambda msg, t=topic: self.on_depth(msg, t), 10)
        self.create_subscription(Image, "/spectator/image", self.on_scene, 10)

    def on_depth(self, msg, topic):
        if msg.encoding != "32FC1":
            self.get_logger().error(f"{topic}: expected 32FC1, got {msg.encoding}",
                                    once=True)
            return
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        self.depths[topic] = np.nan_to_num(depth, nan=0.0, posinf=0.0,
                                           neginf=0.0).astype(np.float64)

    def on_scene(self, msg):
        scene = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h = int(scene.shape[0] * SCENE_W / scene.shape[1])
        scene = cv2.resize(scene, (SCENE_W, h), interpolation=cv2.INTER_AREA)
        panels = np.hstack([_panel(self.depths[t], label) for t, label in HEADS])
        strip = np.full((34, SCENE_W, 3), 20, np.uint8)
        cv2.putText(strip, "heads are UNTRIGGERED at 15 Hz - each panel is that "
                    "head's latest frame, up to one period apart", (10, 23), FONT,
                    0.5, (230, 230, 230), 1, cv2.LINE_AA)
        frame = np.vstack([scene, strip, panels])

        if self.writer is None:
            fh, fw = frame.shape[:2]
            self.writer = cv2.VideoWriter(
                self.out_path, cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (fw, fh))
            if self.poster:
                cv2.imwrite(self.poster, frame)
        self.writer.write(frame)
        self.frames += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--poster", default=None, help="also save the first frame as PNG")
    args = parser.parse_args()

    signal.signal(signal.SIGTERM, signal.default_int_handler)
    rclpy.init()
    node = RigVideoSaver(args.out, args.fps, args.poster)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.writer is not None:
            node.writer.release()
        print(f"{node.frames} frames -> {args.out}", flush=True)


if __name__ == "__main__":
    main()
