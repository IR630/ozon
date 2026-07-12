# -*- coding: utf-8 -*-
"""Belt position tracking and pusher lead-time (day 2, P1).

Pure functions: given where and when perception saw an item, predict where it is
now and when to command the pusher so the paddle meets it. The belt runs at a
known constant speed (src.constants.BELT_SPEED_M_S) along +x, so tracking is
dead-reckoning — no per-frame detection needed between camera and pusher.

Positions are meters in the world frame (x along the belt); times are seconds
(ROS stamp). The controller node supplies the item x (from the measurement) and
the pusher x (from sim/worlds/cell.sdf).
"""
from src.constants import BELT_SPEED_M_S

# Scene calibration, mirrors sim/worlds/cell.sdf (single source is the world
# file): pusher paddle x positions. If the world changes, update here.
PUSHER_X_M = {"C": 2.5, "D": 3.0}
# Paddle response: time from the velocity command to the face reaching the
# item's flank (~0.18 m gap at 2.5 m/s, plus controller jitter). Tuned on the
# e2e run (docs/experiments.md).
ACTUATION_LATENCY_S = 0.1


def plan_push(category, item_x_m, seen_stamp_s, actuation_latency_s=ACTUATION_LATENCY_S):
    """Routing decision for one classified item: (pusher, fire_time_s) or None.

    B rides the belt to its end — no push. C and D get the staggered side
    pushers. Raises ValueError (from fire_time) when the item is already past
    its pusher — the caller decides how to report the miss.

    actuation_latency_s is how long before the item reaches the actuator line
    the command must go out. The pusher default covers its paddle response; the
    swing-arm diverter needs a much larger lead (~0.5 s): its blade tip sweeps
    ACROSS the belt while engaging, so the wall must finish forming before the
    item's front edge enters the sweep zone — firing at pusher timing smacks
    the item toward the wrong side (measured peak_accel 134 m/s^2 vs ~9 for a
    pre-formed wall). The controller exposes it as the fire_lead_s parameter.
    """
    if category not in PUSHER_X_M:
        return None
    when = fire_time(item_x_m, seen_stamp_s, PUSHER_X_M[category],
                     actuation_latency_s=actuation_latency_s)
    return category, when


def position_at(item_x_m, seen_stamp_s, t_s, belt_speed_m_s=BELT_SPEED_M_S):
    """Predicted item x at time t, dead-reckoned from the sighting."""
    return item_x_m + belt_speed_m_s * (t_s - seen_stamp_s)


def fire_time(item_x_m, seen_stamp_s, pusher_x_m,
              belt_speed_m_s=BELT_SPEED_M_S, actuation_latency_s=0.0):
    """Wall-clock time to command the pusher so it fires as the item arrives.

    The item reaches the pusher after (pusher_x - item_x)/belt_speed; command it
    earlier by actuation_latency to cover the mechanism's response time. Raises
    ValueError if the item is already at or past the pusher (too late to catch).
    """
    travel_m = pusher_x_m - item_x_m
    if travel_m <= 0:
        raise ValueError(f"item at x={item_x_m} already past pusher x={pusher_x_m}")
    return seen_stamp_s + travel_m / belt_speed_m_s - actuation_latency_s
