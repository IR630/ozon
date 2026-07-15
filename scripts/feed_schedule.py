#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Emit feed ticks at absolute Gazebo simulation-time delays.

`run_stream.sh` receives gaps in metres and converts them to seconds at the
nominal belt speed.  Waiting those seconds with shell `sleep` is incorrect when
Gazebo runs below real time: the physical gaps shrink with host load.  This
small ROS node watches the bridged `/clock` and prints one line per requested
absolute delay, measured from its first clock sample.

Stdout is a machine contract consumed by the feeder:
    <zero-based index> <observed elapsed simulation seconds>
"""
import math
import sys


def validate_delays(values):
    """Finite, non-negative, non-decreasing absolute delays."""
    delays = [float(value) for value in values]
    if not delays:
        raise ValueError("at least one feed delay is required")
    if any(not math.isfinite(delay) or delay < 0.0 for delay in delays):
        raise ValueError("feed delays must be finite and non-negative")
    if any(back < front for front, back in zip(delays, delays[1:])):
        raise ValueError("feed delays must be non-decreasing")
    return delays


def due_ticks(delays, next_index, elapsed_s):
    """Return all ticks due at `elapsed_s` and the next unread index."""
    due = []
    while next_index < len(delays) and elapsed_s + 1e-9 >= delays[next_index]:
        due.append((next_index, elapsed_s))
        next_index += 1
    return due, next_index


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    try:
        delays = validate_delays(args)
    except ValueError as exc:
        raise SystemExit(f"ABORT: {exc}") from None

    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock

    class FeedSchedule(Node):
        def __init__(self):
            super().__init__("stream_feed_schedule")
            self.start_ns = None
            self.next_index = 0
            self.done = False
            self.create_subscription(Clock, "/clock", self.on_clock, 10)

        def on_clock(self, msg):
            now_ns = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
            if self.start_ns is None:
                self.start_ns = now_ns
            elapsed_s = (now_ns - self.start_ns) / 1e9
            ticks, self.next_index = due_ticks(delays, self.next_index, elapsed_s)
            for index, observed_s in ticks:
                print(f"{index} {observed_s:.6f}", flush=True)
            self.done = self.next_index == len(delays)

    rclpy.init()
    node = FeedSchedule()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
