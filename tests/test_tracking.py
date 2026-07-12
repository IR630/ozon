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


def test_plan_push_custom_lead_fires_earlier():
    # the diverter's blade sweeps across the belt while engaging, so its wall
    # must form before the item's front edge enters the sweep zone: the same
    # plan, commanded fire_lead_s earlier (controller parameter fire_lead_s)
    _, when_pusher = plan_push("C", 1.5, 10.0)
    _, when_diverter = plan_push("C", 1.5, 10.0, actuation_latency_s=0.5)
    assert when_diverter == pytest.approx(when_pusher - (0.5 - ACTUATION_LATENCY_S))


def test_diverter_fires_before_the_item_enters_the_blade_sweep_zone():
    # Calibration lock, NOT a coincidence: for the diverter the fire is planned
    # against PUSHER_X_M["C"]=2.5 with a 0.5 s lead, which means "command the
    # blade when the item is at x=2.0" — upstream of the sweep zone, where the
    # blade tip sweeps the item's lane (x 2.45..2.61, docs/decisions.md).
    #
    # The blade's PIVOT is at x=3.25 (cell_diverter.sdf), and planning against
    # THAT looks like the obvious fix — it is not: the fire would then move to
    # x=2.75, with the item already inside the sweep zone, and the tip would
    # smack it mid-swing (measured 134 m/s^2, the day-5 blocker).
    _, when = plan_push("C", 1.0, 10.0, actuation_latency_s=0.5)
    fire_x = position_at(1.0, 10.0, when)
    assert fire_x == pytest.approx(2.0)
    assert fire_x < 2.45  # the wall forms before the item reaches the sweep zone
