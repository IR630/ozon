#!/usr/bin/env python3
"""Wait until E-stop zero commands are observed on every actuator topic."""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

TOPICS = ("/conveyor/cmd_vel", "/pusher_c/cmd", "/pusher_d/cmd")


def main():
    rclpy.init()
    node = Node("estop_probe")
    stopped = {topic: False for topic in TOPICS}

    for topic in TOPICS:
        node.create_subscription(
            Float64,
            topic,
            lambda msg, name=topic: stopped.__setitem__(name, msg.data == 0.0),
            10,
        )

    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and not all(stopped.values()):
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
    missing = [topic for topic, seen in stopped.items() if not seen]
    if missing:
        raise SystemExit(f"missing zero command: {', '.join(missing)}")
    print("zero commands observed: belt, pusher C, pusher D")


if __name__ == "__main__":
    main()
