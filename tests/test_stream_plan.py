# -*- coding: utf-8 -*-
"""Feed plan of a multi-item stream: the two constraints a stream must respect."""
import pytest

from stream_plan import (
    FIRST_SPAWN_X_M,
    RAMP_TRAVEL_M,
    belt_stroke_m,
    min_gap_between_zones_m,
    min_transport_gap_m,
    plan_stream,
)
from zone_verdict import PAST_MECHANISMS_X

DIVERTER_WORLD = "sim/worlds/cell_diverter.sdf"
# The stream run_stream.sh runs by default, and the one the world is sized for.
DEFAULT_STREAM = ["box_400x400x300:C:0", "pen:C:1.0", "bottle:D:3.5"]

WIDE_STROKE_M = 100.0  # plenty: isolates the constraint under test


def test_a_gap_in_metres_is_fed_as_an_interval_in_simulation_time():
    """The feeder consumes these delays on Gazebo /clock, independent of host load."""
    items = plan_stream(DEFAULT_STREAM, WIDE_STROKE_M)

    assert [(slug, zone) for slug, zone, _ in items] == [
        ("box_400x400x300", "C"), ("pen", "C"), ("bottle", "D")]
    assert [delay for _, _, delay in items] == [0.0, 1.0, 4.5]  # 1 m/s: metres = seconds


def test_same_zone_items_may_ride_nose_to_tail():
    """The blade held for the front item is exactly the wall the back one needs."""
    assert min_gap_between_zones_m("C", "C") == 0.0
    plan_stream(["box_400x400x300:C:0", "pen:C:0.3"], WIDE_STROKE_M)  # no raise


def test_a_fast_follower_cannot_catch_the_slow_pouf_before_the_camera():
    """Worst-case mixed oi1: 4.0 m merged; 5.0 m kept five IDs twice."""
    assert min_transport_gap_m("pouf") == pytest.approx(5.0)
    assert min_transport_gap_m("box_400x400x300") == 0.0

    with pytest.raises(ValueError, match="measured transport needs 5.0 m"):
        plan_stream(["pouf:C:0", "pen:C:4.0"], WIDE_STROKE_M)

    plan_stream(["pouf:C:0", "pen:C:5.0"], WIDE_STROKE_M)


def test_box_then_pouf_keeps_two_ids_and_clears_the_chute_mouth():
    """Gazebo replay: 1.0 m merged oi2; 1.5 m routed all three orientations."""
    assert min_transport_gap_m("box_400x400x300", "pouf") == pytest.approx(1.5)

    with pytest.raises(ValueError, match="pair separation needs 1.5 m"):
        plan_stream(["box_400x400x300:C:0", "pouf:C:1.0"], WIDE_STROKE_M)

    plan_stream(["box_400x400x300:C:0", "pouf:C:1.5"], WIDE_STROKE_M)


def test_a_zone_change_behind_a_fired_blade_is_refused():
    """hold 2.5 s + retract 0.6 s at 1 m/s: a D item 1 m behind a C item is swept into C."""
    assert min_gap_between_zones_m("C", "D") == pytest.approx(3.1)

    with pytest.raises(ValueError, match="zone change needs 3.1 m"):
        plan_stream(["box_400x400x300:C:0", "bottle:D:1.0"], WIDE_STROKE_M)

    plan_stream(["box_400x400x300:C:0", "bottle:D:3.5"], WIDE_STROKE_M)  # no raise


def test_an_item_riding_past_the_mechanisms_holds_nothing_back():
    """B fires nothing, so the item behind it may follow immediately."""
    assert min_gap_between_zones_m("B", "D") == 0.0
    plan_stream(["box_300x200x200:B:0", "bottle:D:0.5"], WIDE_STROKE_M)  # no raise


def test_a_stream_longer_than_the_belt_stroke_is_refused():
    """The slab stops dead at its joint limit — it would halt under the stream."""
    with pytest.raises(ValueError, match="belt travel"):
        plan_stream(DEFAULT_STREAM, stroke_m=6.4)  # the pre-week-2 stroke

    # ...and that old stroke did not even fit ONE item plus a feed interval:
    with pytest.raises(ValueError, match="belt travel"):
        plan_stream(["box_300x200x200:B:0", "bottle:D:3.5"], stroke_m=6.4)


def test_stroke_budget_carries_the_last_item_into_the_terminal_b_band():
    """A plan must not spend the belt before a B item can receive PASS."""
    physically_short_m = (
        RAMP_TRAVEL_M + PAST_MECHANISMS_X - FIRST_SPAWN_X_M - 0.01
    )

    with pytest.raises(ValueError, match="belt travel"):
        plan_stream(["box_300x200x200:B:0"], stroke_m=physically_short_m)


def test_the_diverter_world_can_actually_carry_the_default_stream():
    """The lock on the world: shrink the belt stroke and this run stops being possible."""
    plan_stream(DEFAULT_STREAM, belt_stroke_m(DIVERTER_WORLD))  # no raise

    assert belt_stroke_m(DIVERTER_WORLD) == pytest.approx(20.0)


def test_specs_are_validated():
    with pytest.raises(ValueError, match="slug:zone:gap_m"):
        plan_stream(["box_300x200x200:B"], WIDE_STROKE_M)
    with pytest.raises(ValueError, match="zone must be one of"):
        plan_stream(["box_300x200x200:A:0"], WIDE_STROKE_M)
    with pytest.raises(ValueError, match="gap must be a number"):
        plan_stream(["box_300x200x200:B:0", "pen:C:soon"], WIDE_STROKE_M)
