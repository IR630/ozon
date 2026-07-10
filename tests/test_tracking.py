# -*- coding: utf-8 -*-
"""Belt tracking and pusher lead-time (day 2, P1). Pure math, runs everywhere."""
import pytest

from src.constants import BELT_SPEED_M_S
from src.tracking import fire_time, position_at


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
