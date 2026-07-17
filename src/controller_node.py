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
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64

from ros_msgs.msg import ItemClassification

from src.constants import (
    BELT_SPEED_M_S,
    DIVERTER_FEEDBACK_MAX_RAD,
    DIVERTER_FEEDBACK_MIN_RAD,
    DIVERTER_PARK_TOL_RAD,
)
from src.tracking import ACTUATION_LATENCY_S, plan_push

_RAMP_STEPS = 8          # soft-start fractions of BELT_SPEED_M_S
_RAMP_PERIOD_S = 0.3
_PUSH_SPEED_M_S = 2.5    # ejection speed: item must LAND on the zone patch
_STROKE_S = 0.6          # 1.5 m stroke at 2.5 m/s
_RECOVERY_POLL_S = 0.1
_MAX_MECHANISM_FEEDBACK_AGE_S = 0.5
_STARTUP_PARK_STABLE_S = 0.2
_MAX_STAMP_AGE_S = 1.0
_COMPLETED_TTL_S = 30.0  # late camera frames are irrelevant after leaving the cell
_MAX_COMPLETED_ITEMS = 256  # safety bound if sim time is paused / malformed
# Jam detection. At BELT_SPEED_M_S the item crosses 1 m every second, so 5 cm is
# well under a single camera frame's travel and far above pose jitter: an item
# that has not made even that in _JAM_TIMEOUT_S is not moving, it is wedged.
_JAM_PROGRESS_M = 0.05
_JAM_TIMEOUT_S = 2.0
# Feed watchdog. A jam BEFORE the camera is invisible to detect_jam: the wedged
# item never yields a measurement (the census `feed_jam` class — e.g. the pouf
# stuck at x=-1.47). The cell has no infeed sensor, so the infeed ANNOUNCES every
# item it places on the belt (/infeed/fed) and the controller expects the camera
# to report a NEW item within this budget. Spawn (x=-1.5) to full camera view is
# ~3 m at BELT_SPEED_M_S (~3 s, both runners feed onto a belt already at speed);
# 8 s is a 2.5x margin, and the announce lands AFTER the spawn, so CLI latency
# only ever pushes the deadline later — never a false trip.
_FEED_TIMEOUT_S = 8.0
_FEED_CHECK_PERIOD_S = 0.5


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
        # it. With fresh post-command feedback, re-command the measured angle.
        # Without that proof, stop the belt, retain the last target and report
        # that the physical hold is unverified.
        self.estop_hold_mechanism = bool(
            self.declare_parameter("estop_hold_mechanism", False).value)
        self.mech_cmd = {"C": 0.0, "D": 0.0}  # last command sent to each mechanism
        # The positional diverter exposes its real joint angles through the
        # Gazebo->ROS bridge. A target is not a position: during a return the
        # target is already 0 while the blade is still crossing the belt.
        self.mech_position = {}
        self.mech_feedback_seq = {"C": 0, "D": 0}
        self.mech_feedback_at = {}
        self.mech_feedback_stamp_ns = {}
        self.mech_command_feedback_seq = {"C": 0, "D": 0}
        self.mech_command_at_ns = {"C": -1, "D": -1}
        # A reset temporarily replaces the verified E-stop hold with a park
        # target. If another stop arrives before the next sample, restore this
        # last measured hold instead of blindly continuing the park motion.
        self.estop_verified_hold = {}
        # A controller can restart while the Gazebo world keeps an old 1 m/s
        # belt target. It may start a positional cell only after both blades are
        # freshly observed parked; an engaged startup requires operator reset.
        self.startup_interlock_complete = not self.estop_hold_mechanism
        self.startup_park_commanded = False
        self.startup_parked_since_s = None
        self.startup_parked_feedback_seq = {}
        # How long an item may fail to advance before the cell calls it a jam. The
        # soft-start ramp alone takes 2.4 s, but an item only reaches the camera at
        # full belt speed, so the default needs no ramp allowance.
        self.jam_timeout_s = float(
            self.declare_parameter("jam_timeout_s", _JAM_TIMEOUT_S).value)
        self.jam_anchor = {}            # item_id -> (x_m, stamp_s) of its last real progress
        self.feed_timeout_s = float(
            self.declare_parameter("feed_timeout_s", _FEED_TIMEOUT_S).value)
        # Announced feeds awaiting their first camera sighting. FIFO deadlines:
        # belt items do not overtake, so the oldest announcement is always the
        # next one to arrive. Tracker ids grow monotonically within a session,
        # so "new item" is simply an id above every id seen so far.
        self.feed_deadlines = []
        self.max_item_id_seen = 0
        self.belt_pub = self.create_publisher(Float64, "/conveyor/cmd_vel", 10)
        if self.estop_hold_mechanism:
            # Best-effort immediate stop for a controller restarted inside a
            # live Gazebo world. The watchdog repeats it after DDS discovery.
            self.belt_pub.publish(Float64(data=0.0))
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
        # An E-stop may cancel the return timer while a blade/pusher is physically
        # engaged. The explicit reset must park those zones BEFORE belt soft-start,
        # otherwise the first B item drives into a wall left across the conveyor.
        self.estop_recovery_zones = set()
        self.estop_recovery_feedback_seq = {}
        self.estop_resetting = False
        self.estop_recovery_not_before_s = 0.0
        self.startup_stop_cycles = 0
        self.ramp_step = 0
        self.ramp_timer = self.create_timer(_RAMP_PERIOD_S, self.on_ramp)
        self.estop_recovery_timer = self.create_timer(
            _RECOVERY_POLL_S, self._check_estop_recovery)
        self.estop_recovery_timer.cancel()
        self.create_subscription(ItemClassification, "/item/classification",
                                 self.on_classification, 10)
        self.create_subscription(Bool, "/emergency_stop",
                                 self.on_emergency_stop, 10)
        self.create_subscription(Empty, "/infeed/fed", self.on_infeed, 10)
        for zone in self.pusher_pubs:
            topic = f"/diverter_{zone.lower()}/joint_state"
            self.create_subscription(
                JointState, topic,
                lambda msg, z=zone: self.on_mechanism_state(z, msg), 1)
        self.feed_watchdog = self.create_timer(_FEED_CHECK_PERIOD_S, self.check_infeed)
        self.feedback_watchdog = self.create_timer(
            _RECOVERY_POLL_S, self.check_mechanism_feedback)

    def on_infeed(self, _msg):
        """The infeed just placed one item on the belt; expect it at the camera."""
        if (self.emergency_stopped or self.estop_resetting
                or not self.mechanisms_ready()):
            return
        self.feed_deadlines.append(self.now_s() + self.feed_timeout_s)

    def check_infeed(self):
        """Latch the cell when an announced item never showed up at the camera.

        The reaction mirrors detect_jam: the safe state plus the named cause —
        an infeed wedged under a stuck item must not keep feeding (and a fed
        item the camera silently MISSED must not ride unmeasured either;
        Karpathy #6 — fail loudly, not quietly).
        """
        if (self.emergency_stopped or self.estop_resetting
                or not self.mechanisms_ready() or not self.feed_deadlines):
            return
        if self.now_s() < self.feed_deadlines[0]:
            return
        self.get_logger().error(
            f"FEED JAM: an item fed {self.feed_timeout_s:.0f}s ago never reached the "
            "camera — wedged on the infeed or missed by perception; latching emergency stop")
        self.latch_emergency_stop()

    def on_ramp(self):
        if self.emergency_stopped or self.estop_resetting:
            return
        if not self.mechanisms_ready():
            # An Ignition JointController retains its last target across a ROS
            # controller restart. Absence of a positive command is therefore
            # not a stop; repeat an explicit zero until the interlock is proven.
            self.belt_pub.publish(Float64(data=0.0))
            return
        self.ramp_step += 1
        self.belt_pub.publish(Float64(data=BELT_SPEED_M_S * self.ramp_step / _RAMP_STEPS))
        if self.ramp_step >= _RAMP_STEPS:
            self.ramp_timer.cancel()
            self.get_logger().info(f"belt at {BELT_SPEED_M_S} m/s (soft-start done)")

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_classification(self, msg):
        if (self.emergency_stopped or self.estop_resetting
                or not self.mechanisms_ready()):
            return
        if msg.item_id > self.max_item_id_seen:
            self.max_item_id_seen = msg.item_id
            if self.feed_deadlines:
                self.feed_deadlines.pop(0)  # the oldest announced feed has arrived
        self._prune_completed()
        if msg.item_id in self.fired:
            # too late to act, but a contradiction must be visible in the log
            if msg.category != self.fired[msg.item_id]:
                self.get_logger().warn(
                    f"item {msg.item_id}: classified {msg.category} AFTER firing "
                    f"pusher_{self.fired[msg.item_id].lower()} — possible mis-sort",
                    throttle_duration_sec=1.0)
            return

        # age_s is BOTH the wall/sim-clock sanity guard below AND the honest
        # camera->decision latency: the camera frame stamp rides the messages, so
        # now() minus that stamp spans the whole perception->classify->decide chain
        # on ONE sim-clock (no cross-process log-emission jitter). It is logged at
        # the route commit so scripts/measure_throughput.py can report it directly.
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        age_s = self.now_s() - stamp_s
        if not 0.0 <= age_s <= _MAX_STAMP_AGE_S:
            self.get_logger().error(
                f"item {msg.item_id}: stamp age {age_s:.2f}s — wall/sim clock mix? "
                "run with use_sim_time:=true and the /clock bridge")
            return

        if self.detect_jam(msg.item_id, msg.position.x, stamp_s):
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
                self.get_logger().info(
                    f"item {msg.item_id}: B — rides to belt end "
                    f"(cam->decision {age_s:.3f}s)")
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
            f"item {msg.item_id}: {zone} — firing pusher_{zone.lower()} in {delay_s:.2f}s "
            f"(cam->decision {age_s:.3f}s)")
        self.pending[msg.item_id] = self.one_shot(
            delay_s, lambda z=zone, i=msg.item_id: self.fire(z, i))

    def fire(self, zone, item_id):
        self.pending.pop(item_id, None)
        if self.emergency_stopped or not self.mechanisms_ready():
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

    def detect_jam(self, item_id, x_m, stamp_s):
        """A jam is an item the camera still sees that has stopped advancing.

        The belt runs at a known constant speed, so an item under the camera MUST
        move — roughly BELT_SPEED_M_S per second. One that does not is wedged
        (against a rail, a blade, or a stalled belt), and the cell must not keep
        feeding items into it. The reaction is deliberately the safe one: latch
        the emergency stop, name the cause, and wait for a human — an automatic
        re-fire at a jammed item is how a sorter breaks the goods it is sorting.

        Progress resets the anchor, so a normally moving item never trips this.
        Returns True when a jam was declared (the caller must stop handling msg).

        LIMITATION: this only sees jams INSIDE the camera window. An item wedged on
        the infeed never reaches perception at all (the census `feed_jam` class,
        e.g. the pouf at x=-1.47) and is invisible here — those are caught by the
        feed watchdog instead (on_infeed/check_infeed): the infeed announces every
        item it places and the camera must confirm it within feed_timeout_s.
        """
        anchor = self.jam_anchor.get(item_id)
        if anchor is None:
            self.jam_anchor[item_id] = (x_m, stamp_s)
            return False

        anchor_x_m, anchor_s = anchor
        if x_m - anchor_x_m >= _JAM_PROGRESS_M:
            self.jam_anchor[item_id] = (x_m, stamp_s)  # moving: re-anchor
            return False
        if stamp_s - anchor_s < self.jam_timeout_s:
            return False                                # not stalled long enough yet

        stalled_s = stamp_s - anchor_s
        self.get_logger().error(
            f"JAM: item {item_id} advanced {x_m - anchor_x_m:+.3f} m in {stalled_s:.1f}s "
            f"at x={x_m:.2f} (belt runs {BELT_SPEED_M_S} m/s) — latching emergency stop")
        self.latch_emergency_stop()
        return True

    def latch_emergency_stop(self):
        """The safe state, whether a human or the jam detector asked for it."""
        self.on_emergency_stop(Bool(data=True))

    def command_mechanism(self, zone, value, *, require_new_feedback=False):
        """Every mechanism command goes through here, so the controller always
        knows its last target and which feedback can prove the resulting state."""
        target_changed = abs(value - self.mech_cmd[zone]) > 1e-9
        self.mech_cmd[zone] = value
        # A sample received before this target cannot prove where the moving
        # blade is now. A redundant re-publish does not invalidate a newer
        # measurement unless this is an explicit startup/reset ownership gate.
        if target_changed or require_new_feedback:
            self.mech_command_feedback_seq[zone] = self.mech_feedback_seq[zone]
            self.mech_command_at_ns[zone] = self.get_clock().now().nanoseconds
        self.pusher_pubs[zone].publish(Float64(data=value))

    def on_mechanism_state(self, zone, msg):
        """Remember a positional blade's measured angle, never its old target."""
        if not msg.position:
            return
        position = float(msg.position[0])
        if (not math.isfinite(position)
                or not DIVERTER_FEEDBACK_MIN_RAD
                <= position <= DIVERTER_FEEDBACK_MAX_RAD):
            self.get_logger().error(
                f"diverter {zone}: invalid joint position {position}; expected "
                f"{DIVERTER_FEEDBACK_MIN_RAD}..{DIVERTER_FEEDBACK_MAX_RAD} rad")
            if self.estop_hold_mechanism:
                self.latch_emergency_stop()
            return
        stamp_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )
        # /clock and JointState are separate bridge topics, so a state from the
        # next simulation tick can arrive first. Drop it at receipt: retaining
        # it would let the same old packet become "causal" when /clock catches
        # up, without any new sensor acquisition.
        if not 0 <= stamp_ns <= self.get_clock().now().nanoseconds:
            return
        self.mech_position[zone] = position
        self.mech_feedback_seq[zone] += 1
        self.mech_feedback_at[zone] = self.now_s()
        self.mech_feedback_stamp_ns[zone] = stamp_ns
        self._check_startup_interlock()

    def _joint_feedback_fresh(self, zone):
        received_s = self.mech_feedback_at.get(zone)
        if received_s is None:
            return False
        age_s = self.now_s() - received_s
        return 0.0 <= age_s <= _MAX_MECHANISM_FEEDBACK_AGE_S

    def _joint_feedback_after_command(self, zone):
        """Prove acquisition happened after the target, not merely its callback."""
        stamp_ns = self.mech_feedback_stamp_ns.get(zone, -1)
        now_ns = self.get_clock().now().nanoseconds
        return (
            self._joint_feedback_fresh(zone)
            and self.mech_feedback_seq[zone]
            > self.mech_command_feedback_seq[zone]
            and self.mech_command_at_ns[zone] < stamp_ns <= now_ns
        )

    def _check_startup_interlock(self):
        """Start automatically only when both positional blades are freshly parked."""
        if (self.startup_interlock_complete or not self.estop_hold_mechanism
                or self.emergency_stopped or self.estop_resetting):
            return
        # Two watchdog publications leave a discovery margin and make zero the
        # first observable belt command even if C/D feedback arrives instantly.
        if (self.startup_stop_cycles < 2
                or not all(self._joint_feedback_fresh(zone)
                           for zone in self.pusher_pubs)):
            self.startup_parked_since_s = None
            self.startup_parked_feedback_seq.clear()
            return
        unparked = [
            zone for zone in sorted(self.pusher_pubs)
            if abs(self.mech_position[zone] - self.retract_cmd)
            > DIVERTER_PARK_TOL_RAD
        ]
        if unparked:
            self.startup_parked_since_s = None
            self.startup_parked_feedback_seq.clear()
            zones = "/".join(unparked)
            self.get_logger().error(
                f"STARTUP INTERLOCK: mechanism {zones} is not parked; "
                "latching E-stop until operator clear/reset")
            self.latch_emergency_stop()
            return
        if not self.startup_park_commanded:
            # Take ownership from any position target retained by a still-live
            # Gazebo controller, then require feedback newer than these commands.
            for zone in sorted(self.pusher_pubs):
                self.command_mechanism(
                    zone, self.retract_cmd, require_new_feedback=True)
            self.startup_park_commanded = True
            self.startup_parked_since_s = None
            self.startup_parked_feedback_seq.clear()
            return
        if any(not self._joint_feedback_after_command(zone)
               for zone in self.pusher_pubs):
            self.startup_parked_since_s = None
            self.startup_parked_feedback_seq.clear()
            return
        if self.startup_parked_since_s is None:
            self.startup_parked_since_s = self.now_s()
            self.startup_parked_feedback_seq = dict(self.mech_feedback_seq)
            return
        if (self.now_s() - self.startup_parked_since_s
                < _STARTUP_PARK_STABLE_S
                or any(self.mech_feedback_seq[zone]
                       <= self.startup_parked_feedback_seq[zone]
                       for zone in self.pusher_pubs)):
            return
        self.startup_interlock_complete = True
        self.get_logger().info(
            "startup interlock: fresh C/D feedback confirms both blades parked")

    def check_mechanism_feedback(self):
        """A running positional cell fails stopped if either angle stream disappears."""
        if not self.estop_hold_mechanism:
            return
        if not self.startup_interlock_complete:
            self.belt_pub.publish(Float64(data=0.0))
            self.startup_stop_cycles += 1
            self._check_startup_interlock()
            return
        if self.emergency_stopped or self.estop_resetting:
            return
        stale = [
            zone for zone in sorted(self.pusher_pubs)
            if not self._joint_feedback_fresh(zone)
        ]
        if not stale:
            return
        zones = "/".join(stale)
        self.get_logger().error(
            f"JOINT FEEDBACK LOST: mechanism {zones}; latching emergency stop")
        self.latch_emergency_stop()

    def mechanisms_ready(self):
        """Velocity pushers need no angle; positional blades need a live interlock."""
        return (not self.estop_hold_mechanism
                or (self.startup_interlock_complete
                    and all(self._joint_feedback_fresh(zone)
                            for zone in self.pusher_pubs)))

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
            self.jam_anchor.pop(item_id, None)

    def on_emergency_stop(self, msg):
        """Latch an immediate safe stop; False explicitly rearms soft-start."""
        requested = bool(msg.data)
        if requested:
            if self.emergency_stopped and not self.estop_resetting:
                return
            was_resetting = self.estop_resetting
            if not was_resetting:
                self.estop_verified_hold.clear()
            self.emergency_stopped = True
            active_zones = set(self.returning) | self.estop_recovery_zones
            active_zones.update(
                zone for zone, command in self.mech_cmd.items()
                if abs(command) > 1e-9
            )
            active_zones.update(
                zone for zone, position in self.mech_position.items()
                if abs(position - self.retract_cmd) > DIVERTER_PARK_TOL_RAD
            )
            if self.estop_hold_mechanism:
                # A positional reset proves the WHOLE transport path clear, not
                # only the zone whose last target happened to look active.
                active_zones.update(self.pusher_pubs)
            self.estop_recovery_zones = active_zones
            self.estop_recovery_feedback_seq.clear()
            self.estop_resetting = False
            self.estop_recovery_timer.cancel()
            self.ramp_timer.cancel()
            for timer in tuple(self.one_shot_timers):
                self._retire(timer)
            self.pending.clear()
            self.returning.clear()  # its timers were just retired — no stale refs
            self.belt_pub.publish(Float64(data=0.0))
            for zone in self.pusher_pubs:
                # freeze, do not re-park: on a positional mechanism 0.0 would SWING
                # the blade home mid-emergency (see estop_hold_mechanism)
                can_hold_measured = (
                    self.estop_hold_mechanism
                    and self._joint_feedback_after_command(zone)
                )
                if can_hold_measured:
                    halt = self.mech_position[zone]
                    self.estop_verified_hold[zone] = halt
                elif (self.estop_hold_mechanism and was_resetting
                      and zone in self.estop_verified_hold):
                    halt = self.estop_verified_hold[zone]
                    self.get_logger().error(
                        f"E-STOP mechanism {zone}: no fresh feedback after its "
                        "parking command; restoring last verified hold, "
                        "physical hold unverified")
                else:
                    halt = self.mech_cmd[zone] if self.estop_hold_mechanism else 0.0
                    if self.estop_hold_mechanism:
                        self.get_logger().error(
                            f"E-STOP mechanism {zone}: no fresh feedback after its "
                            "last command; retaining target, physical hold unverified")
                self.command_mechanism(
                    zone, halt, require_new_feedback=self.estop_hold_mechanism)
            self.get_logger().error(
                "E-STOP active: belt stopped; mechanism hold commands issued")
            return

        if not self.emergency_stopped or self.estop_resetting:
            return

        # Drop the stall anchors: they were taken before the stop, so the belt's
        # own standstill is baked into them and the first frame after the reset
        # would be read as a fresh jam, latching the cell straight back down.
        # Same for the feed deadlines: they were promised for a belt that then
        # stood still, so the first watchdog tick after reset would re-latch.
        self.jam_anchor.clear()
        self.feed_deadlines.clear()
        unavailable_feedback = [
            zone for zone in sorted(self.estop_recovery_zones)
            if (self.estop_hold_mechanism
                and not self._joint_feedback_after_command(zone))
        ]
        if unavailable_feedback:
            zones = "/".join(unavailable_feedback)
            self.get_logger().error(
                f"E-STOP reset refused: no fresh post-hold joint feedback "
                f"for mechanism {zones}")
            return

        if self.estop_hold_mechanism:
            # Snapshot the latest measured positions atomically BEFORE park
            # replaces their targets. An immediate second E-stop can then undo
            # that blind park command without moving a loaded blade back toward
            # an older angle captured at the first stop.
            self.estop_verified_hold = {
                zone: self.mech_position[zone]
                for zone in self.estop_recovery_zones
            }
        self.estop_resetting = True
        if self.estop_recovery_zones:
            self.estop_recovery_feedback_seq = {
                zone: self.mech_feedback_seq[zone]
                for zone in self.estop_recovery_zones
            }
            for zone in sorted(self.estop_recovery_zones):
                self.command_mechanism(
                    zone, self.retract_cmd, require_new_feedback=True)
            self.estop_recovery_not_before_s = self.now_s() + _STROKE_S
            self.estop_recovery_timer.reset()
            zones = "/".join(sorted(self.estop_recovery_zones))
            self.get_logger().warn(
                f"E-STOP reset: parking mechanism {zones} before belt soft-start")
            return

        self._finish_estop_reset()

    def _check_estop_recovery(self):
        """Release the belt only after fresh feedback proves every blade parked."""
        if not self.estop_resetting:
            self.estop_recovery_timer.cancel()
            return
        if self.now_s() < self.estop_recovery_not_before_s:
            return
        if not self.estop_hold_mechanism:
            self._finish_estop_reset()
            return
        unconfirmed = [
            zone for zone in sorted(self.estop_recovery_zones)
            if (self.mech_feedback_seq[zone]
                <= self.estop_recovery_feedback_seq[zone]
                or not self._joint_feedback_after_command(zone)
                or abs(self.mech_position[zone] - self.retract_cmd)
                > DIVERTER_PARK_TOL_RAD)
        ]
        if unconfirmed:
            return
        self._finish_estop_reset()

    def _finish_estop_reset(self):
        """Finish mechanism recovery, then re-arm the belt's soft-start."""
        self.estop_recovery_timer.cancel()
        for zone in sorted(self.estop_recovery_zones):
            self.command_mechanism(zone, 0.0)
        self.estop_recovery_zones.clear()
        self.estop_recovery_feedback_seq.clear()
        self.estop_verified_hold.clear()
        self.estop_resetting = False
        self.emergency_stopped = False
        self.startup_interlock_complete = True
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
