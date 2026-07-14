# -*- coding: utf-8 -*-
"""Node-level test for the controller (day 3): classification -> commands.

Runs only where rclpy and the built ros_msgs overlay exist (WSL / ROS CI).
A probe node plays the classifier and listens to the actuator topics. No
Gazebo: the test checks the command protocol, not the physics.
"""
import pytest

rclpy = pytest.importorskip("rclpy")
msgs = pytest.importorskip("ros_msgs.msg")

from std_msgs.msg import Bool, Float64  # noqa: E402

from src.constants import BELT_SPEED_M_S  # noqa: E402
from src.controller_node import (  # noqa: E402
    _COMPLETED_TTL_S,
    _MAX_COMPLETED_ITEMS,
    _PUSH_SPEED_M_S,
    ControllerNode,
)
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

        # item just before the C pusher -> fires almost immediately; the
        # duplicate classification (same item_id) replans the schedule
        # (cancel + reschedule) and must NOT double-fire
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


def test_c_then_b_correction_cancels_pusher():
    # boundary items flip category between frames (Цилиндр K=0.74, Пуфик 489 мм):
    # a fresher B classification must cancel the already scheduled pusher
    rclpy.init()
    try:
        node = ControllerNode()
        probe = rclpy.create_node("probe")
        pusher_c_cmds = []
        probe.create_subscription(Float64, "/pusher_c/cmd",
                                  lambda m: pusher_c_cmds.append(m.data), 10)
        pub = probe.create_publisher(ItemClassification, "/item/classification", 10)
        _spin_both(node, probe, 0.3)

        pub.publish(_classification(node, 9, "C", PUSHER_X_M["C"] - 0.5))
        pub.publish(_classification(node, 9, "B", PUSHER_X_M["C"] - 0.5))
        _spin_both(node, probe, 1.5)

        assert pusher_c_cmds == []

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_b_then_c_correction_fires():
    # the reverse flip: an early B must not lock the item out of a fresher
    # C classification — the pusher still has to fire
    rclpy.init()
    try:
        node = ControllerNode()
        probe = rclpy.create_node("probe")
        pusher_c_cmds = []
        probe.create_subscription(Float64, "/pusher_c/cmd",
                                  lambda m: pusher_c_cmds.append(m.data), 10)
        pub = probe.create_publisher(ItemClassification, "/item/classification", 10)
        _spin_both(node, probe, 0.3)

        pub.publish(_classification(node, 10, "B", PUSHER_X_M["C"] - 0.5))
        pub.publish(_classification(node, 10, "C", PUSHER_X_M["C"] - 0.15))
        _spin_both(node, probe, 2.0)

        assert pusher_c_cmds.count(_PUSH_SPEED_M_S) == 1, pusher_c_cmds

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_c_then_d_flip_cancels_c_and_fires_d_once():
    # zone flip between pushers: the scheduled C timer must be cancelled,
    # only pusher_d fires, and exactly once
    rclpy.init()
    try:
        node = ControllerNode()
        probe = rclpy.create_node("probe")
        pusher_c_cmds, pusher_d_cmds = [], []
        probe.create_subscription(Float64, "/pusher_c/cmd",
                                  lambda m: pusher_c_cmds.append(m.data), 10)
        probe.create_subscription(Float64, "/pusher_d/cmd",
                                  lambda m: pusher_d_cmds.append(m.data), 10)
        pub = probe.create_publisher(ItemClassification, "/item/classification", 10)
        _spin_both(node, probe, 0.3)

        pub.publish(_classification(node, 11, "C", PUSHER_X_M["C"] - 0.5))
        pub.publish(_classification(node, 11, "D", PUSHER_X_M["D"] - 0.15))
        # spin until the full stroke (fire -> retract -> stop) completes, so no
        # late 0.0 command leaks into the next test's subscriptions
        import time
        end = time.monotonic() + 6.0
        while time.monotonic() < end and (not pusher_d_cmds or pusher_d_cmds[-1] != 0.0):
            _spin_both(node, probe, 0.2)

        assert pusher_c_cmds == [], pusher_c_cmds
        assert pusher_d_cmds.count(_PUSH_SPEED_M_S) == 1, pusher_d_cmds

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_superseded_timers_do_not_accumulate():
    # each C schedules a one-shot timer, the following B cancels it; cancelled
    # timers must not pile up in one_shot_timers (regression: the list grew
    # without bound — every fire/retract/replan leaked a timer reference)
    rclpy.init()
    try:
        node = ControllerNode()
        probe = rclpy.create_node("probe")
        pub = probe.create_publisher(ItemClassification, "/item/classification", 10)
        _spin_both(node, probe, 0.3)  # discovery

        for i in range(1, 21):
            # x a full metre before the pusher -> ~0.9 s fire window; the B that
            # follows cancels the scheduled timer long before it could fire. Spin
            # after each publish so stamps stay fresh (no backlog shrinking the
            # window into a race).
            pub.publish(_classification(node, i, "C", PUSHER_X_M["C"] - 1.0))
            _spin_both(node, probe, 0.03)
            pub.publish(_classification(node, i, "B", PUSHER_X_M["C"] - 1.0))
            _spin_both(node, probe, 0.03)

        assert len(node.one_shot_timers) <= 2, len(node.one_shot_timers)

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_hold_s_parameter_delays_retract():
    # the diverter (same command topics) is a wall, not a stroke: the blade must
    # stay engaged while the belt slides the item off (~1.2-1.5 s), so the
    # fire->retract delay is the hold_s parameter. With a long hold the retract
    # (-v) must NOT arrive at the pusher-default _STROKE_S=0.6 s after the fire.
    from rclpy.parameter import Parameter

    rclpy.init()
    try:
        node = ControllerNode(parameter_overrides=[Parameter("hold_s", value=10.0)])
        probe = rclpy.create_node("probe")
        pusher_c_cmds = []
        probe.create_subscription(Float64, "/pusher_c/cmd",
                                  lambda m: pusher_c_cmds.append(m.data), 10)
        pub = probe.create_publisher(ItemClassification, "/item/classification", 10)
        _spin_both(node, probe, 0.3)

        pub.publish(_classification(node, 12, "C", PUSHER_X_M["C"] - 0.15))
        _spin_both(node, probe, 2.0)  # fire ~0 s in; default would retract at 0.6 s

        assert pusher_c_cmds.count(_PUSH_SPEED_M_S) == 1, pusher_c_cmds
        assert -_PUSH_SPEED_M_S not in pusher_c_cmds, pusher_c_cmds

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


def test_a_late_diverter_decision_is_marked_missed_without_actuation():
    """A delayed C/D decision that arrives past its blade cannot be recovered.

    Firing late would hit the following product or move an empty mechanism. The
    conservative reaction is a terminal MISSED state with no actuator command;
    repeated late frames for the same item must remain deduplicated.
    """
    rclpy.init()
    try:
        node = ControllerNode()
        probe = rclpy.create_node("probe_late_decision")
        pusher_c_cmds = []
        probe.create_subscription(Float64, "/pusher_c/cmd",
                                  lambda m: pusher_c_cmds.append(m.data), 10)
        pub = probe.create_publisher(ItemClassification, "/item/classification", 10)
        _spin_both(node, probe, 0.3)

        too_late = _classification(node, 19, "C", PUSHER_X_M["C"] + 0.01)
        pub.publish(too_late)
        pub.publish(too_late)
        _spin_both(node, probe, 0.6)

        assert 19 in node.done_items
        assert 19 not in node.pending
        assert 19 not in node.fired
        assert pusher_c_cmds == []

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_emergency_stop_cancels_motion_and_reset_soft_starts_belt():
    rclpy.init()
    try:
        node = ControllerNode()
        probe = rclpy.create_node("probe_estop")
        belt_cmds, pusher_c_cmds, pusher_d_cmds = [], [], []
        probe.create_subscription(Float64, "/conveyor/cmd_vel",
                                  lambda m: belt_cmds.append(m.data), 10)
        probe.create_subscription(Float64, "/pusher_c/cmd",
                                  lambda m: pusher_c_cmds.append(m.data), 10)
        probe.create_subscription(Float64, "/pusher_d/cmd",
                                  lambda m: pusher_d_cmds.append(m.data), 10)
        classification_pub = probe.create_publisher(
            ItemClassification, "/item/classification", 10)
        estop_pub = probe.create_publisher(Bool, "/emergency_stop", 10)
        _spin_both(node, probe, 0.3)

        # Schedule a future push, then stop before its timer can fire.
        classification_pub.publish(
            _classification(node, 20, "C", PUSHER_X_M["C"] - 1.0))
        _spin_both(node, probe, 0.1)
        estop_pub.publish(Bool(data=True))
        _spin_both(node, probe, 1.2)

        assert node.emergency_stopped
        assert node.pending == {}
        assert belt_cmds[-1] == 0.0
        assert pusher_c_cmds == [0.0], pusher_c_cmds
        assert pusher_d_cmds == [0.0], pusher_d_cmds

        # Classifications are ignored while latched. Reset is explicit and the
        # belt must ramp again instead of jumping straight to full speed.
        classification_pub.publish(
            _classification(node, 21, "D", PUSHER_X_M["D"] - 0.15))
        _spin_both(node, probe, 0.3)
        assert pusher_d_cmds == [0.0], pusher_d_cmds

        reset_at = len(belt_cmds)
        estop_pub.publish(Bool(data=False))
        _spin_both(node, probe, 0.7)
        resumed = belt_cmds[reset_at:]
        assert not node.emergency_stopped
        assert resumed
        assert 0.0 < resumed[0] < BELT_SPEED_M_S / 2
        assert resumed == sorted(resumed)

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_a_stalled_item_latches_the_cell_instead_of_being_fired_at():
    """An item the camera still sees but that stops advancing is wedged.

    The belt runs at a known speed, so an item under the camera must move. The
    safe reaction is to latch the stop and name the cause — never to keep firing
    a mechanism at goods that are stuck.
    """
    from rclpy.parameter import Parameter

    rclpy.init()
    try:
        node = ControllerNode(parameter_overrides=[
            Parameter("jam_timeout_s", value=0.5)])  # keep the test quick
        probe = rclpy.create_node("probe_jam")
        belt_cmds, fired = [], []
        probe.create_subscription(Float64, "/conveyor/cmd_vel",
                                  lambda m: belt_cmds.append(m.data), 10)
        probe.create_subscription(Float64, "/pusher_c/cmd",
                                  lambda m: fired.append(m.data), 10)
        classification_pub = probe.create_publisher(
            ItemClassification, "/item/classification", 10)
        _spin_both(node, probe, 0.3)

        # Same item, same x, frame after frame: the belt is moving, it is not.
        stuck_x_m = PUSHER_X_M["C"] - 1.0
        for _ in range(6):
            classification_pub.publish(_classification(node, 50, "C", stuck_x_m))
            _spin_both(node, probe, 0.2)

        assert node.emergency_stopped, "a stalled item did not latch the cell"
        assert belt_cmds[-1] == 0.0
        assert 0.0 not in [c for c in fired if c == _PUSH_SPEED_M_S], fired
        assert node.pending == {}, "a push was still scheduled at a jammed item"

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_an_item_riding_the_belt_normally_is_not_a_jam():
    """The anchor re-arms on every real advance, so a moving item never trips it."""
    from rclpy.parameter import Parameter

    rclpy.init()
    try:
        node = ControllerNode(parameter_overrides=[
            Parameter("jam_timeout_s", value=0.5)])
        probe = rclpy.create_node("probe_no_jam")
        classification_pub = probe.create_publisher(
            ItemClassification, "/item/classification", 10)
        _spin_both(node, probe, 0.3)

        # Belt speed is 1 m/s and the frames are 0.2 s apart: 0.2 m of travel each.
        x_m = PUSHER_X_M["C"] - 2.0
        for _ in range(6):
            classification_pub.publish(_classification(node, 51, "C", x_m))
            _spin_both(node, probe, 0.2)
            x_m += 0.2

        assert not node.emergency_stopped, "a normally moving item was called a jam"
        assert 51 in node.pending, "the push for a healthy item was dropped"

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_an_announced_feed_the_camera_never_confirms_latches_feed_jam():
    """A jam BEFORE the camera is invisible to detect_jam (no measurements ever
    arrive), so the infeed announces every feed on /infeed/fed and the watchdog
    latches the safe stop when the camera never confirms the item."""
    from rclpy.parameter import Parameter

    from std_msgs.msg import Empty

    rclpy.init()
    try:
        node = ControllerNode(parameter_overrides=[
            Parameter("feed_timeout_s", value=0.6)])
        probe = rclpy.create_node("probe_feed_jam")
        fed_pub = probe.create_publisher(Empty, "/infeed/fed", 10)
        _spin_both(node, probe, 0.3)

        fed_pub.publish(Empty())
        _spin_both(node, probe, 1.5)  # past the 0.6 s budget + watchdog period

        assert node.emergency_stopped, "a vanished feed did not latch the cell"

        # Reset must clear the stale promise, or the first watchdog tick after
        # the reset would latch the cell straight back down.
        node.on_emergency_stop(Bool(data=False))
        _spin_both(node, probe, 1.0)
        assert node.feed_deadlines == []
        assert not node.emergency_stopped

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_an_announced_feed_that_reaches_the_camera_is_not_a_feed_jam():
    from rclpy.parameter import Parameter

    from std_msgs.msg import Empty

    rclpy.init()
    try:
        node = ControllerNode(parameter_overrides=[
            Parameter("feed_timeout_s", value=0.6)])
        probe = rclpy.create_node("probe_feed_ok")
        fed_pub = probe.create_publisher(Empty, "/infeed/fed", 10)
        classification_pub = probe.create_publisher(
            ItemClassification, "/item/classification", 10)
        _spin_both(node, probe, 0.3)

        fed_pub.publish(Empty())
        _spin_both(node, probe, 0.1)
        # the fed item shows up: its FIRST (new-id) classification pays the debt
        classification_pub.publish(_classification(node, 61, "B", 1.2))
        _spin_both(node, probe, 1.5)

        assert node.feed_deadlines == []
        assert not node.emergency_stopped, "an arrived feed was called a feed jam"

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_emergency_stop_freezes_an_engaged_diverter_blade_instead_of_parking_it():
    """On the POSITION-driven diverter, 0.0 is not "stop" — it is "go home".

    Zeroing the topic on E-stop would swing an engaged blade back across the belt,
    possibly out from under the item leaning on it: motion during an emergency
    stop. The blade must instead be re-commanded to the angle it already holds.
    The velocity-driven pusher keeps its old behaviour (0.0 = paddle freezes) and
    is covered by the test above.
    """
    from rclpy.parameter import Parameter

    rclpy.init()
    try:
        node = ControllerNode(parameter_overrides=[
            Parameter("engage_cmd", value=0.90),      # engaged blade angle, rad
            Parameter("retract_cmd", value=0.0),      # parked angle
            Parameter("estop_hold_mechanism", value=True),
            Parameter("hold_s", value=10.0),          # still engaged when we stop it
        ])
        probe = rclpy.create_node("probe_estop_hold")
        blade_cmds = []
        probe.create_subscription(Float64, "/pusher_c/cmd",
                                  lambda m: blade_cmds.append(m.data), 10)
        classification_pub = probe.create_publisher(
            ItemClassification, "/item/classification", 10)
        estop_pub = probe.create_publisher(Bool, "/emergency_stop", 10)
        _spin_both(node, probe, 0.3)

        # Fire the blade: it is now a wall across the belt at 0.90 rad.
        classification_pub.publish(
            _classification(node, 40, "C", PUSHER_X_M["C"] - 0.05))
        _spin_both(node, probe, 0.6)
        assert blade_cmds == [0.90], blade_cmds

        estop_pub.publish(Bool(data=True))
        _spin_both(node, probe, 1.0)

        assert node.emergency_stopped
        # The blade was NOT sent home: its commanded angle never left 0.90.
        assert blade_cmds[-1] == 0.90, blade_cmds
        assert 0.0 not in blade_cmds, blade_cmds

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_second_item_to_same_zone_is_not_cut_short_by_the_first_retract():
    # Multi-item flow (PLAN-WEEK2 day 10): "действие одного товара не отменяет
    # другое". Two items routed to the SAME zone closer together than hold_s.
    # The mechanism is shared, and the retract used to be scheduled hold_s after
    # EACH fire — so item 1's retract yanked the paddle/blade back while item 2
    # was still being pushed, and item 2 rode on to the belt end (a misroute that
    # the single-item census could never show). The mechanism must stay engaged
    # until hold_s after the LAST fire on that zone.
    import time

    rclpy.init()
    try:
        node = ControllerNode()          # default hold_s = 0.6 s
        probe = rclpy.create_node("probe")
        cmds = []                        # (t, value) on /pusher_c/cmd
        t0 = time.monotonic()
        probe.create_subscription(
            Float64, "/pusher_c/cmd",
            lambda m: cmds.append((time.monotonic() - t0, m.data)), 10)
        pub = probe.create_publisher(ItemClassification, "/item/classification", 10)
        _spin_both(node, probe, 0.3)

        # item 1 fires ~0.05 s in, item 2 ~0.35 s in: 0.3 s apart, well inside
        # the 0.6 s hold (at 1 m/s that is two items 0.3 m apart on the belt)
        pub.publish(_classification(node, 21, "C", PUSHER_X_M["C"] - 0.15))
        pub.publish(_classification(node, 22, "C", PUSHER_X_M["C"] - 0.45))
        _spin_both(node, probe, 1.8)

        fires = [t for t, v in cmds if v == _PUSH_SPEED_M_S]
        retracts = [t for t, v in cmds if v == -_PUSH_SPEED_M_S]
        assert len(fires) == 2, f"both items must be pushed: {cmds}"
        assert retracts, f"the mechanism must return: {cmds}"
        held_s = retracts[0] - fires[-1]
        assert held_s >= node.hold_s - 0.1, (
            f"retract {held_s:.2f}s after the last fire, hold_s={node.hold_s}: "
            f"item 2's push was cut short by item 1's retract — {cmds}")

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_two_items_schedule_c_and_d_without_cross_fire():
    rclpy.init()
    try:
        node = ControllerNode()
        probe = rclpy.create_node("probe_two_items")
        pusher_c_cmds, pusher_d_cmds = [], []
        probe.create_subscription(Float64, "/pusher_c/cmd",
                                  lambda m: pusher_c_cmds.append(m.data), 10)
        probe.create_subscription(Float64, "/pusher_d/cmd",
                                  lambda m: pusher_d_cmds.append(m.data), 10)
        pub = probe.create_publisher(ItemClassification, "/item/classification", 10)
        _spin_both(node, probe, 0.3)

        pub.publish(_classification(node, 30, "C", PUSHER_X_M["C"] - 0.15))
        pub.publish(_classification(node, 31, "D", PUSHER_X_M["D"] - 0.15))
        _spin_both(node, probe, 2.0)

        assert pusher_c_cmds.count(_PUSH_SPEED_M_S) == 1, pusher_c_cmds
        assert pusher_d_cmds.count(_PUSH_SPEED_M_S) == 1, pusher_d_cmds
        assert node.fired[30] == "C"
        assert node.fired[31] == "D"

        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_completed_item_history_has_ttl_and_hard_bound():
    rclpy.init()
    try:
        node = ControllerNode()
        node.fired = {1: "C", 2: "D"}
        node.done_items = {3}
        node._completed_at = {
            1: 0.0,
            2: _COMPLETED_TTL_S,
            3: _COMPLETED_TTL_S,
        }

        node._prune_completed(now_s=_COMPLETED_TTL_S + 1.0)
        assert 1 not in node.fired
        assert node.fired == {2: "D"}
        assert node.done_items == {3}

        # Paused/broken sim time cannot make terminal state grow forever.
        node._completed_at = {
            item_id: 100.0 for item_id in range(_MAX_COMPLETED_ITEMS + 2)}
        node.done_items = set(node._completed_at)
        node._prune_completed(now_s=100.0)
        assert len(node._completed_at) == _MAX_COMPLETED_ITEMS
        assert 0 not in node.done_items
        assert 1 not in node.done_items

        node.destroy_node()
    finally:
        rclpy.shutdown()
