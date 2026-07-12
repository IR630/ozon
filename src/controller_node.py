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
from std_msgs.msg import Bool, Float64

from ros_msgs.msg import ItemClassification

from src.constants import BELT_SPEED_M_S
from src.tracking import ACTUATION_LATENCY_S, plan_push

_RAMP_STEPS = 8          # soft-start fractions of BELT_SPEED_M_S
_RAMP_PERIOD_S = 0.3
_PUSH_SPEED_M_S = 2.5    # ejection speed: item must LAND on the zone patch
_STROKE_S = 0.6          # 1.5 m stroke at 2.5 m/s
_MAX_STAMP_AGE_S = 1.0
_COMPLETED_TTL_S = 30.0  # late camera frames are irrelevant after leaving the cell
_MAX_COMPLETED_ITEMS = 256  # safety bound if sim time is paused / malformed


class ControllerNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__("controller", **node_kwargs)
        # How long the actuator stays engaged after fire before the return
        # command. The pusher is a stroke: +v, then -v after _STROKE_S. The
        # diverter answers the SAME topics but is a wall: the blade must stay
        # across the belt while the belt slides the item off its edge (~1.2-1.5 s
        # for box_400), so the hold is a mechanism parameter, not a constant —
        # run_skeleton.sh sets it for the diverter world via skeleton.launch.py.
        self.hold_s = float(self.declare_parameter("hold_s", _STROKE_S).value)
        # How long BEFORE the item reaches the actuator line to command it.
        # Pusher: its 0.1 s paddle response. Diverter: ~0.5 s — the blade tip
        # sweeps across the belt while engaging, so the wall must be formed
        # before the item's front edge enters the sweep zone (src.tracking).
        self.fire_lead_s = float(
            self.declare_parameter("fire_lead_s", ACTUATION_LATENCY_S).value)
        # What "engage" and "return" MEAN on the mechanism's command topic. The
        # pusher is velocity-driven (m/s, +stroke then -stroke). The diverter is
        # POSITION-driven (rad): a velocity-commanded blade held against its 0.95
        # limit stopped accepting commands altogether and stayed across the belt
        # for the rest of the episode (docs/decisions.md, day 10) — so its world
        # runs a JointPositionController and these carry the engage angle and the
        # parked angle. Defaults keep the pusher exactly as it was.
        self.engage_cmd = float(
            self.declare_parameter("engage_cmd", _PUSH_SPEED_M_S).value)
        self.retract_cmd = float(
            self.declare_parameter("retract_cmd", -_PUSH_SPEED_M_S).value)
        # What an E-STOP means for the mechanism. For the velocity-driven pusher,
        # "stop" is 0.0 — the paddle freezes mid-stroke. For the POSITION-driven
        # diverter, 0.0 is not "stop", it is "go to the parked angle": an E-stop
        # would SWING the blade home, possibly out from under the item leaning on
        # it. Motion during an emergency stop defeats the whole point, so a
        # positional mechanism is instead re-commanded to the angle it already
        # holds — it freezes exactly where it is.
        self.estop_hold_mechanism = bool(
            self.declare_parameter("estop_hold_mechanism", False).value)
        self.mech_cmd = {"C": 0.0, "D": 0.0}  # last command sent to each mechanism
        self.belt_pub = self.create_publisher(Float64, "/conveyor/cmd_vel", 10)
        self.pusher_pubs = {
            "C": self.create_publisher(Float64, "/pusher_c/cmd", 10),
            "D": self.create_publisher(Float64, "/pusher_d/cmd", 10),
        }
        self.fired = {}                 # item_id -> zone; irreversible once fired
        self.done_items = set()         # decided B / missed (log dedupe; a fresher C/D overrides)
        self._completed_at = {}         # item_id -> sim time; bounds long-running streams
        self.pending = {}               # item_id -> scheduled fire timer (replannable)
        self.returning = {}             # zone -> pending retract/stop timer of the SHARED mechanism
        self.one_shot_timers = set()    # keep one-shot timers alive (Node.timers is taken)
        self.emergency_stopped = False  # latched until an explicit False command
        self.ramp_step = 0
        self.ramp_timer = self.create_timer(_RAMP_PERIOD_S, self.on_ramp)
        self.create_subscription(ItemClassification, "/item/classification",
                                 self.on_classification, 10)
        self.create_subscription(Bool, "/emergency_stop",
                                 self.on_emergency_stop, 10)

    def on_ramp(self):
        if self.emergency_stopped:
            return
        self.ramp_step += 1
        self.belt_pub.publish(Float64(data=BELT_SPEED_M_S * self.ramp_step / _RAMP_STEPS))
        if self.ramp_step >= _RAMP_STEPS:
            self.ramp_timer.cancel()
            self.get_logger().info(f"belt at {BELT_SPEED_M_S} m/s (soft-start done)")

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_classification(self, msg):
        if self.emergency_stopped:
            return
        self._prune_completed()
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
            plan = plan_push(msg.category, msg.position.x, stamp_s,
                             actuation_latency_s=self.fire_lead_s)
        except ValueError as e:
            if msg.item_id not in self.pending and msg.item_id not in self.done_items:
                self._remember_completed(msg.item_id)  # never even scheduled
                self.get_logger().error(f"item {msg.item_id}: MISSED — {e}")
            return
        if plan is None:
            old = self.pending.pop(msg.item_id, None)  # C/D -> B flip: drop the push
            if old is not None:
                self._retire(old)
            if msg.item_id not in self.done_items:
                self._remember_completed(msg.item_id)
                self.get_logger().info(f"item {msg.item_id}: B — rides to belt end")
            return

        # (re)schedule: cancel the previous plan, trust the freshest measurement
        zone, fire_at_s = plan
        self.done_items.discard(msg.item_id)  # B -> C/D flip: the push is back on
        self._completed_at.pop(msg.item_id, None)
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
        if self.emergency_stopped:
            return
        # Items are independent, but the MECHANISM is shared. A second item
        # routed to the same zone within hold_s used to inherit the first item's
        # return chain: that retract (scheduled hold_s after the FIRST fire)
        # pulled the paddle/blade back mid-push, and item 2 rode on to the belt
        # end. Drop the stale return chain — the mechanism stays engaged until
        # hold_s after the LAST fire on this zone.
        self._cancel_return(zone)
        self.fired[item_id] = zone
        self._remember_completed(item_id, fired=True)
        self.get_logger().info(
            f"item {item_id}: pusher_{zone.lower()} FIRED at t={self.now_s():.2f}s")
        self.command_mechanism(zone, self.engage_cmd)
        self.returning[zone] = self.one_shot(self.hold_s, lambda z=zone: self.retract(z))

    def retract(self, zone):
        self.returning.pop(zone, None)  # this timer is the one firing right now
        if self.emergency_stopped:
            return
        # The mechanism's RETURN was invisible in the log — only its fire was —
        # so a diverter blade that stayed across the belt for a whole episode
        # looked exactly like a working one until a stream of items ran into it
        # (day 10). Log both ends of the stroke.
        self.get_logger().info(
            f"pusher_{zone.lower()} RETRACT at t={self.now_s():.2f}s")
        self.command_mechanism(zone, self.retract_cmd)
        self.returning[zone] = self.one_shot(_STROKE_S, lambda z=zone: self.stop(z))

    def stop(self, zone):
        self.returning.pop(zone, None)
        self.get_logger().info(f"pusher_{zone.lower()} STOP at t={self.now_s():.2f}s")
        self.command_mechanism(zone, 0.0)

    def command_mechanism(self, zone, value):
        """Every mechanism command goes through here, so the controller always
        knows what it is currently holding — which is what an E-stop must keep."""
        self.mech_cmd[zone] = value
        self.pusher_pubs[zone].publish(Float64(data=value))

    def _cancel_return(self, zone):
        """Drop a pending retract/stop of `zone` (a fresh item took the mechanism).

        Safe from fire(): the timer retired here belongs to the PREVIOUS item's
        return chain, never to the callback currently executing.
        """
        timer = self.returning.pop(zone, None)
        if timer is not None:
            self._retire(timer)

    def _remember_completed(self, item_id, fired=False):
        if not fired:
            self.done_items.add(item_id)
        # Refresh insertion order when a state changes; dict order gives a cheap
        # deterministic oldest-first cap without assuming numeric ID order.
        self._completed_at.pop(item_id, None)
        self._completed_at[item_id] = self.now_s()
        self._prune_completed()

    def _prune_completed(self, now_s=None):
        """Forget terminal items after TTL; never touches pending/active timers."""
        now_s = self.now_s() if now_s is None else float(now_s)
        expired = [item_id for item_id, completed_s in self._completed_at.items()
                   if now_s - completed_s > _COMPLETED_TTL_S]
        overflow = max(len(self._completed_at) - _MAX_COMPLETED_ITEMS, 0)
        expired.extend(list(self._completed_at)[:overflow])
        for item_id in dict.fromkeys(expired):
            self._completed_at.pop(item_id, None)
            self.fired.pop(item_id, None)
            self.done_items.discard(item_id)

    def on_emergency_stop(self, msg):
        """Latch an immediate safe stop; False explicitly rearms soft-start."""
        requested = bool(msg.data)
        if requested == self.emergency_stopped:
            return

        self.emergency_stopped = requested
        if requested:
            self.ramp_timer.cancel()
            for timer in tuple(self.one_shot_timers):
                self._retire(timer)
            self.pending.clear()
            self.returning.clear()  # its timers were just retired — no stale refs
            self.belt_pub.publish(Float64(data=0.0))
            for zone in self.pusher_pubs:
                # freeze, do not re-park: on a positional mechanism 0.0 would SWING
                # the blade home mid-emergency (see estop_hold_mechanism)
                halt = self.mech_cmd[zone] if self.estop_hold_mechanism else 0.0
                self.command_mechanism(zone, halt)
            self.get_logger().error("E-STOP active: belt and mechanisms stopped")
            return

        self.ramp_step = 0
        self.ramp_timer.reset()
        self.get_logger().warn("E-STOP reset: restarting belt with soft-start")

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
