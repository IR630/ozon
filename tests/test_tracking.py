# -*- coding: utf-8 -*-
"""Belt tracking, pusher lead-time, push planning (days 2-3). Pure math, runs everywhere."""
import pytest

from src.constants import BELT_SPEED_M_S
from src.tracking import ACTUATION_LATENCY_S, PUSHER_X_M, fire_time, plan_push, position_at


def test_position_dead_reckoning():
    # seen at x=1.5 m, t=10 s, belt 1.0 m/s -> +1 m per second
    assert position_at(1.5, 10.0, 11.0, belt_speed_m_s=1.0) == pytest.approx(2.5)
    assert position_at(1.5, 10.0, 10.5, belt_speed_m_s=1.0) == pytest.approx(2.0)


def test_position_uses_belt_speed_constant():
    # default speed is the single-source constant, not a magic number
    assert position_at(0.0, 0.0, 2.0) == pytest.approx(2.0 * BELT_SPEED_M_S)


def test_fire_time_no_latency():
    # camera x=1.5, pusher x=2.5, 1.0 m/s -> 1.0 s travel
    assert fire_time(1.5, 10.0, 2.5, belt_speed_m_s=1.0) == pytest.approx(11.0)


def test_fire_time_leads_by_actuation_latency():
    assert fire_time(1.5, 10.0, 2.5, belt_speed_m_s=1.0, actuation_latency_s=0.2) == pytest.approx(10.8)


def test_fire_time_scales_with_speed():
    assert fire_time(1.5, 10.0, 2.5, belt_speed_m_s=2.0) == pytest.approx(10.5)


def test_fire_time_item_past_pusher_raises():
    with pytest.raises(ValueError):
        fire_time(3.0, 10.0, 2.5, belt_speed_m_s=1.0)


def test_plan_push_b_rides_to_belt_end():
    assert plan_push("B", 1.5, 10.0) is None


def test_plan_push_c_and_d_use_staggered_pushers():
    # seen at x=1.5, stamp 10 s: C pusher at 2.5 -> fire at 11-latency,
    # D pusher at 3.0 -> fire at 11.5-latency (belt speed from constants)
    zone, when = plan_push("C", 1.5, 10.0)
    assert zone == "C"
    assert when == pytest.approx(10.0 + 1.0 / BELT_SPEED_M_S - ACTUATION_LATENCY_S)
    zone, when = plan_push("D", 1.5, 10.0)
    assert zone == "D"
    assert when == pytest.approx(10.0 + 1.5 / BELT_SPEED_M_S - ACTUATION_LATENCY_S)


def test_plan_push_item_past_pusher_raises():
    with pytest.raises(ValueError):
        plan_push("C", PUSHER_X_M["C"] + 0.1, 10.0)
