# -*- coding: utf-8 -*-
"""Controller ROS 2 node: ItemClassification -> belt + pusher commands (day 3).

The routing brain of the skeleton. On startup it soft-starts the belt (a 0->1
m/s step launches round items — docs/decisions.md 2026-07-11). For each
classified item: B — nothing (the belt carries it to its end), C/D — schedule
the matching staggered pusher via dead-reckoning (src.tracking.plan_push).

The pusher schedule is REPLANNED on every fresh classification (15 Hz) until
the fire moment: dead-reckoning assumes constant belt speed, which is false
while the item crosses the camera during the start ramp — the freshest
measurement is taken at full speed close to the pusher and self-corrects that.

MUST run with use_sim_time:=true and the /clock bridge: camera stamps are
Gazebo sim time, and the fire timer counts on the same clock. A sanity check
rejects measurements older than a second (wall/sim clock mix fails silently
otherwise — Karpathy #6).

Runs inside the ROS 2 environment (needs rclpy and the built ros_msgs overlay):
    python3 -m src.controller_node --ros-args -p use_sim_time:=true
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from ros_msgs.msg import ItemClassification

from src.constants import BELT_SPEED_M_S
from src.tracking import plan_push

_RAMP_STEPS = 8          # soft-start fractions of BELT_SPEED_M_S
_RAMP_PERIOD_S = 0.3
_PUSH_SPEED_M_S = 2.5    # ejection speed: item must LAND on the zone patch
_STROKE_S = 0.6          # 1.5 m stroke at 2.5 m/s
_MAX_STAMP_AGE_S = 1.0


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller")
        self.belt_pub = self.create_publisher(Float64, "/conveyor/cmd_vel", 10)
        self.pusher_pubs = {
            "C": self.create_publisher(Float64, "/pusher_c/cmd", 10),
            "D": self.create_publisher(Float64, "/pusher_d/cmd", 10),
        }
        self.fired = {}                 # item_id -> zone; irreversible once fired
        self.done_items = set()         # decided B / missed (log dedupe; a fresher C/D overrides)
        self.pending = {}               # item_id -> scheduled fire timer (replannable)
        self.one_shot_timers = set()    # keep one-shot timers alive (Node.timers is taken)
        self.ramp_step = 0
        self.ramp_timer = self.create_timer(_RAMP_PERIOD_S, self.on_ramp)
        self.create_subscription(ItemClassification, "/item/classification",
                                 self.on_classification, 10)

    def on_ramp(self):
        self.ramp_step += 1
        self.belt_pub.publish(Float64(data=BELT_SPEED_M_S * self.ramp_step / _RAMP_STEPS))
        if self.ramp_step >= _RAMP_STEPS:
            self.ramp_timer.cancel()
            self.get_logger().info(f"belt at {BELT_SPEED_M_S} m/s (soft-start done)")

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_classification(self, msg):
        if msg.item_id in self.fired:
            # too late to act, but a contradiction must be visible in the log
            if msg.category != self.fired[msg.item_id]:
                self.get_logger().warn(
                    f"item {msg.item_id}: classified {msg.category} AFTER firing "
                    f"pusher_{self.fired[msg.item_id].lower()} — possible mis-sort",
                    throttle_duration_sec=1.0)
            return

        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age_s = self.now_s() - stamp_s
        if not 0.0 <= age_s <= _MAX_STAMP_AGE_S:
            self.get_logger().error(
                f"item {msg.item_id}: stamp age {age_s:.2f}s — wall/sim clock mix? "
                "run with use_sim_time:=true and the /clock bridge")
            return

        try:
            plan = plan_push(msg.category, msg.position.x, stamp_s)
        except ValueError as e:
            if msg.item_id not in self.pending and msg.item_id not in self.done_items:
                self.done_items.add(msg.item_id)  # never even scheduled
                self.get_logger().error(f"item {msg.item_id}: MISSED — {e}")
            return
        if plan is None:
            old = self.pending.pop(msg.item_id, None)  # C/D -> B flip: drop the push
            if old is not None:
                self._retire(old)
            if msg.item_id not in self.done_items:
                self.done_items.add(msg.item_id)
                self.get_logger().info(f"item {msg.item_id}: B — rides to belt end")
            return

        # (re)schedule: cancel the previous plan, trust the freshest measurement
        zone, fire_at_s = plan
        self.done_items.discard(msg.item_id)  # B -> C/D flip: the push is back on
        old = self.pending.pop(msg.item_id, None)
        if old is not None:
            self._retire(old)
        delay_s = max(fire_at_s - self.now_s(), 0.0)
        self.get_logger().info(
            f"item {msg.item_id}: {zone} — firing pusher_{zone.lower()} in {delay_s:.2f}s")
        self.pending[msg.item_id] = self.one_shot(
            delay_s, lambda z=zone, i=msg.item_id: self.fire(z, i))

    def fire(self, zone, item_id):
        self.pending.pop(item_id, None)
        self.fired[item_id] = zone
        self.get_logger().info(
            f"item {item_id}: pusher_{zone.lower()} FIRED at t={self.now_s():.2f}s")
        self.pusher_pubs[zone].publish(Float64(data=_PUSH_SPEED_M_S))
        self.one_shot(_STROKE_S, lambda z=zone: self.retract(z))

    def retract(self, zone):
        self.pusher_pubs[zone].publish(Float64(data=-_PUSH_SPEED_M_S))
        self.one_shot(_STROKE_S, lambda z=zone: self.pusher_pubs[z].publish(Float64(data=0.0)))

    def one_shot(self, delay_s, callback):
        """rclpy timers repeat; wrap to fire once (sim-time aware)."""
        timer = None

        def cb():
            # drop our ref to the fired timer so one_shot_timers stays bounded;
            # don't destroy_timer from inside its own callback — it's reclaimed on
            # destroy_node(). Superseded timers are freed eagerly via _retire.
            timer.cancel()
            self.one_shot_timers.discard(timer)
            callback()

        timer = self.create_timer(max(delay_s, 1e-3), cb)
        self.one_shot_timers.add(timer)
        return timer

    def _retire(self, timer):
        """Cancel and free a superseded one-shot (replan / C-D->B flip). Called
        outside the timer's own callback, so destroy_timer is safe here."""
        timer.cancel()
        self.one_shot_timers.discard(timer)
        self.destroy_timer(timer)


def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
