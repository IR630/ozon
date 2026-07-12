# -*- coding: utf-8 -*-
"""Episode verdict bands — the jury-visible definition of a correct run."""
import pytest

from zone_verdict import in_zone


def test_b_is_the_item_still_on_the_belt_past_the_mechanisms():
    assert in_zone("B", 3.6, 0.0, 0.45)
    assert not in_zone("B", 3.4, 0.0, 0.45)   # not past them yet
    assert not in_zone("B", 3.6, 0.0, 0.10)   # on the floor: it was diverted


def test_c_and_d_are_the_roll_cages_on_the_floor():
    assert in_zone("C", 3.0, 0.9, 0.05)
    assert in_zone("D", 3.5, -0.9, 0.05)
    assert not in_zone("C", 3.0, -0.9, 0.05)  # right cage, wrong side
    assert not in_zone("D", 3.5, 0.9, 0.05)


def test_an_item_still_on_the_belt_is_in_neither_cage():
    """Belt height (z~0.45) fails C and D — a diverted item must reach the floor."""
    assert not in_zone("C", 3.0, 0.9, 0.45)
    assert not in_zone("D", 3.5, -0.9, 0.45)


def test_an_unknown_zone_is_an_error_not_a_silent_false():
    with pytest.raises(ValueError, match="zone must be B, C or D"):
        in_zone("A", 3.0, 0.0, 0.05)
