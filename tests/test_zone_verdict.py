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


def test_the_c_and_d_bands_reach_their_cage_wall_not_just_the_patch():
    """A large item (pouf) diverted into C settles AGAINST the cage wall (y=1.5),
    its CENTRE at y~1.43 — past the flat patch edge (1.3) but contained by the cage.

    The band must follow the CONTAINER (the walls), not the decorative patch, or a
    correctly-sorted big item FAILs by centimetres. pouf oi=1 does exactly this:
    x=3.07 y=1.43 z=0.14, deterministic across replays (docs/experiments.md
    2026-07-13); census #2 passed it only because physics noise landed it at 1.40.
    Past the wall (|y|>1.5) is a genuine escape and must still fail.
    """
    assert in_zone("C", 3.07, 1.433, 0.14)      # pouf oi=1: contained by the cage wall
    assert in_zone("D", 3.5, -1.433, 0.14)
    assert not in_zone("C", 3.0, 1.55, 0.05)    # past the wall (1.5) — escaped
    assert not in_zone("D", 3.5, -1.55, 0.05)


def test_an_item_still_on_the_belt_is_in_neither_cage():
    """Belt height (z~0.45) fails C and D — a diverted item must reach the floor."""
    assert not in_zone("C", 3.0, 0.9, 0.45)
    assert not in_zone("D", 3.5, -0.9, 0.45)


def test_an_unknown_zone_is_an_error_not_a_silent_false():
    with pytest.raises(ValueError, match="zone must be B, C or D"):
        in_zone("A", 3.0, 0.0, 0.05)
