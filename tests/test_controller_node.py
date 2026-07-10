# -*- coding: utf-8 -*-
"""Node-level test for the controller (day 3): classification -> commands.

Runs only where rclpy and the built ros_msgs overlay exist (WSL / ROS CI).
A probe node plays the classifier and listens to the actuator topics. No
Gazebo: the test checks the command protocol, not the physics.
"""
import pytest

rclpy = pytest.importorskip("rclpy")
msgs = pytest.importorskip("ros_msgs.msg")

from std_msgs.msg import Float64  # noqa: E402

from src.constants import BELT_SPEED_M_S  # noqa: E402
from src.controller_node import _PUSH_SPEED_M_S, ControllerNode  # noqa: E402
from src.tracking import PUSHER_X_M  # noqa: E402

ItemClassification = msgs.ItemClassification


def _spin_both(a, b, seconds):
    import time

    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(a, timeout_sec=0.05)
        rclpy.spin_once(b, timeout_sec=0.05)


def _classification(node, item_id, category, x_m):
    msg = ItemClassification()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.item_id = item_id
    msg.category = category
    msg.dims_mm = [300.0, 200.0, 200.0]
    msg.k = 0.5
    msg.confidence = 1.0
    msg.position.x = x_m
    return msg


def test_soft_start_and_c_routing_fire_once():
    rclpy.init()
    try:
        node = ControllerNode()
        probe = rclpy.create_node("probe")
        belt_cmds, pusher_c_cmds = [], []
        probe.create_subscription(Float64, "/conveyor/cmd_vel",
                                  lambda m: belt_cmds.append(m.data), 10)
        probe.create_subscription(Float64, "/pusher_c/cmd",
                                  lambda m: pusher_c_cmds.append(m.data), 10)
        pub = probe.create_publisher(ItemClassification, "/item/classification", 10)
        _spin_both(node, probe, 0.3)  # discovery

        # item just before the C pusher -> fires almost immediately;
        # the duplicate classification (same item_id) must NOT double-fire
        msg = _classification(node, 7, "C", PUSHER_X_M["C"] - 0.15)
        pub.publish(msg)
        pub.publish(msg)
        _spin_both(node, probe, 3.5)

        # soft start: monotonic ramp ending at BELT_SPEED_M_S, never a step to full
        assert belt_cmds, "no belt commands seen"
        assert belt_cmds[0] < BELT_SPEED_M_S / 2
        assert belt_cmds == sorted(belt_cmds)
        assert belt_cmds[-1] == pytest.approx(BELT_SPEED_M_S)

        # exactly one fire (+v), then retract (-v), then stop (0)
        assert pusher_c_cmds.count(_PUSH_SPEED_M_S) == 1, pusher_c_cmds
        assert pusher_c_cmds.count(-_PUSH_SPEED_M_S) == 1, pusher_c_cmds
        assert pusher_c_cmds[-1] == 0.0

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_b_does_not_touch_pushers():
    rclpy.init()
    try:
        node = ControllerNode()
        probe = rclpy.create_node("probe")
        fired = []
        probe.create_subscription(Float64, "/pusher_c/cmd", fired.append, 10)
        probe.create_subscription(Float64, "/pusher_d/cmd", fired.append, 10)
        pub = probe.create_publisher(ItemClassification, "/item/classification", 10)
        _spin_both(node, probe, 0.3)

        pub.publish(_classification(node, 8, "B", 1.5))
        _spin_both(node, probe, 1.5)

        assert fired == []

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()
