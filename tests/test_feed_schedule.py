# -*- coding: utf-8 -*-
"""Pure contract of the simulation-clock stream feed scheduler."""
import math

import pytest

from feed_schedule import due_ticks, validate_delays


def test_delay_validation_accepts_a_sim_time_schedule():
    assert validate_delays(["0", "1.0", "4.1", "4.1"]) == [0.0, 1.0, 4.1, 4.1]


@pytest.mark.parametrize(
    "values",
    [[], [0, -1], [0, math.inf], [0, math.nan], [0, 2, 1]],
)
def test_invalid_schedule_is_rejected(values):
    with pytest.raises(ValueError):
        validate_delays(values)


def test_ticks_follow_elapsed_simulation_time_not_callback_count():
    delays = [0.0, 1.0, 4.1]
    ticks, next_index = due_ticks(delays, 0, 0.0)
    assert ticks == [(0, 0.0)]

    ticks, next_index = due_ticks(delays, next_index, 0.9)
    assert ticks == []
    ticks, next_index = due_ticks(delays, next_index, 1.2)
    assert ticks == [(1, 1.2)]

    # A slow callback may cross more than one target; no feed tick is lost.
    ticks, next_index = due_ticks(delays, next_index, 5.0)
    assert ticks == [(2, 5.0)]
    assert next_index == len(delays)
