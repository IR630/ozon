# -*- coding: utf-8 -*-
"""The parked blade must read as parked.

Every sample here is copied from a real `ign topic -e` capture of
/world/cell/model/diverter_c/joint_state, because the defect being fixed was
entirely about what Ignition actually prints — protobuf omits a field holding
its default, so a blade at exactly 0 rad has no `position:` line at all.
"""
import pytest

from scripts.joint_angle import angle_from_stream

ENGAGED = """header {
  stamp {
    sec: 9
    nsec: 390000000
  }
}
name: "diverter_c"
id: 33
pose {
  position {
    x: 3.25
    y: 0.28
    z: 0.552
  }
}
joint {
  name: "diverter_joint"
  axis1 {
    xyz {
      z: 1
    }
    limit_upper: 0.95
    position: 0.70859171621384509
    velocity: -6.3645782022447861e-07
  }
}
"""

# The same message with the blade parked: Ignition prints NO position field.
PARKED = """header {
  stamp {
    sec: 42
    nsec: 0
  }
}
name: "diverter_c"
id: 33
pose {
  position {
    x: 3.25
    y: 0.28
    z: 0.552
  }
}
joint {
  name: "diverter_joint"
  axis1 {
    xyz {
      z: 1
    }
    limit_upper: 0.95
  }
}
"""


def test_an_engaged_blade_reads_its_angle():
    assert angle_from_stream(ENGAGED) == pytest.approx(0.7085917162)


def test_a_parked_blade_reads_zero_rather_than_nothing():
    """THE defect. Protobuf drops the field at the default, and the old reader
    took that as 'keep scanning', walking into a later message."""
    assert angle_from_stream(PARKED) == 0.0


def test_a_parked_blade_does_not_borrow_the_next_message_s_angle():
    """The failure mode that made the smoke report 3.638 rad on a blade a
    131-sample trace showed sitting at exactly 0.0: `ign topic -e` streams, so
    the value 'further down' belongs to a different instant."""
    assert angle_from_stream(PARKED + ENGAGED) == 0.0


def test_the_model_pose_is_never_mistaken_for_the_joint_angle():
    """`pose { position { x: 3.25 } }` is a BRACE, not a colon — the blade sits
    at x=3.25 m, and reading that as 3.25 rad would be a plausible wrong answer."""
    assert angle_from_stream(PARKED) != pytest.approx(3.25)


def test_scientific_notation_survives_the_round_trip():
    """The parked blade passes through 6e-08 on its way down; a reader that
    dropped the exponent would report 6 rad."""
    tiny = ENGAGED.replace("position: 0.70859171621384509", "position: 6.021412e-08")
    assert angle_from_stream(tiny) == pytest.approx(6.021412e-08)


def test_no_message_at_all_is_not_reported_as_a_parked_blade():
    """THE trap in fixing the first trap.

    "The sensor said nothing" and "the blade is at zero" must not collapse into
    one answer: the caller gates a BELT RESTART on this value and has a separate
    no-feedback guard that would go dead if a silent topic returned 0.0.
    """
    assert angle_from_stream("") is None
    assert angle_from_stream("timed out\n") is None
