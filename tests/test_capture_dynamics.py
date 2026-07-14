# -*- coding: utf-8 -*-
"""Unit tests for the gentleness metric (scripts/capture_dynamics.py).

Pure Python — no Gazebo. parse_trace is checked on a hand-written slice of real
`ign topic -e` output; compute_peaks on synthetic trajectories with known peaks.
"""
import math

from capture_dynamics import (
    com_offset_m,
    compute_peaks,
    compute_rotational_bound,
    parse_trace,
    parse_trace_full,
)

# A two-message slice in the exact format of /world/cell/dynamic_pose/info, with
# the item moving +x by 0.02 m over 0.02 s (-> 1 m/s). Note conveyor omits y
# (exactly 0), which the parser must default to 0.
_TRACE = """header {
  stamp {
    sec: 9
    nsec: 0
  }
}
pose {
  name: "conveyor"
  id: 8
  position {
    x: 1.70
    z: 0.35
  }
}
pose {
  name: "item"
  id: 50
  position {
    x: 1.800
    y: 0.0
    z: 0.399
  }
}
header {
  stamp {
    sec: 9
    nsec: 20000000
  }
}
pose {
  name: "item"
  id: 50
  position {
    x: 1.820
    y: 0.0
    z: 0.399
  }
}
"""


def test_parse_trace_extracts_item_stamped_positions():
    samples = parse_trace(_TRACE, name="item")
    assert len(samples) == 2
    (t0, x0, y0, z0), (t1, x1, y1, z1) = samples
    assert t0 == 9.0 and abs(t1 - 9.02) < 1e-9
    assert x0 == 1.800 and x1 == 1.820
    assert y0 == 0.0 and z0 == 0.399  # y present-but-zero, z parsed


def test_parse_trace_skips_messages_without_the_item():
    # first message has no item pose -> skipped, only the second counts
    text = 'header {\n stamp { sec: 1 nsec: 0 }\n}\npose { name: "belt" position { x: 0 } }\n' \
           'header {\n stamp { sec: 1 nsec: 500000000 }\n}\npose { name: "item" position { x: 2 y: 1 z: 3 } }\n'
    samples = parse_trace(text, name="item")
    assert samples == [(1.5, 2.0, 1.0, 3.0)]


def _line(n, dt, vx):
    """Synthetic constant-velocity trace: n samples spaced dt, speed vx along x."""
    return [(i * dt, i * dt * vx, 0.0, 0.0) for i in range(n)]


def test_constant_velocity_peak_speed_no_accel():
    peaks = compute_peaks(_line(50, 0.02, 1.0), mass=2.0)
    assert abs(peaks["peak_speed"] - 1.0) < 1e-6
    assert peaks["peak_accel"] < 1e-6          # steady motion -> no acceleration
    assert abs(peaks["peak_impulse"] - 2.0) < 1e-6   # mass * peak_speed


def test_impact_spike_registers_high_accel():
    # rest for 5 samples, then move at 2.5 m/s with no time gap: a launch-like
    # impact (velocity jumps 0 -> 2.5 in one step)
    slow = [(i * 0.02, 0.0, 0.0, 0.0) for i in range(5)]        # t 0.00..0.08, x=0
    fast = [(0.08 + i * 0.02, 2.5 * (i * 0.02), 0.0, 0.0) for i in range(1, 20)]
    peaks = compute_peaks(slow + fast, mass=1.0)
    assert peaks["peak_speed"] >= 2.4
    assert peaks["peak_accel"] > 50.0          # sharp velocity step -> big accel


def test_too_short_trace_is_zero_not_crash():
    assert compute_peaks([(0.0, 0.0, 0.0, 0.0)], mass=5.0) == {
        "peak_speed": 0.0, "peak_accel": 0.0, "peak_impulse": 0.0}


# Real-format slice WITH orientation blocks, as the live topic prints them:
# zero fields are omitted by the protobuf printer, so identity is bare "w: 1"
# and a 180-deg turn about x is bare "x: 1" (w=0 omitted!) — the parser must
# default EVERY component to 0, not assume w=1.
_TRACE_QUAT = """header {
  stamp {
    sec: 3
    nsec: 0
  }
}
pose {
  name: "item"
  id: 50
  position {
    x: 1.0
  }
  orientation {
    w: 1
  }
}
header {
  stamp {
    sec: 3
    nsec: 20000000
  }
}
pose {
  name: "item"
  id: 50
  position {
    x: 1.02
  }
  orientation {
    x: 1
  }
}
"""


def test_parse_trace_full_extracts_orientation_with_omitted_zero_fields():
    samples = parse_trace_full(_TRACE_QUAT, name="item")
    assert len(samples) == 2
    (t0, p0, q0), (t1, p1, q1) = samples
    assert t0 == 3.0 and p0 == (1.0, 0.0, 0.0)
    assert q0 == (0.0, 0.0, 0.0, 1.0)   # identity: only w printed
    assert q1 == (1.0, 0.0, 0.0, 0.0)   # 180 deg about x: w=0 omitted


def test_parse_trace_full_yields_no_quaternion_for_legacy_traces():
    samples = parse_trace_full(_TRACE, name="item")  # old fixture: positions only
    assert len(samples) == 2
    assert all(q is None for _, _, q in samples)


def _spinning_point(omega, r, n, dt):
    """(t, (x,y,z), quat) of a model origin at body offset r from a fixed COM,
    body spinning about world z at constant omega: pure centripetal motion,
    measured origin accel == omega^2*r exactly."""
    samples = []
    for i in range(n):
        t = i * dt
        theta = omega * t
        pos = (r * math.cos(theta), r * math.sin(theta), 0.0)
        quat = (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))
        samples.append((t, pos, quat))
    return samples


def test_rotational_bound_recovers_centripetal_accel_of_spinning_point():
    omega, r = 10.0, 0.2                 # omega^2*r = 20 m/s^2
    samples = _spinning_point(omega, r, n=500, dt=0.002)
    bound = compute_rotational_bound(samples, com_offset_m=r)
    assert bound is not None
    # for pure rotation the measured origin accel IS entirely rotational, so
    # the bound must recover it within numerical-differentiation error
    assert abs(bound["omega_at_peak"] - omega) / omega < 0.05
    assert abs(bound["rotational_bound"] - omega ** 2 * r) / (omega ** 2 * r) < 0.10
    assert abs(bound["peak_accel"] - omega ** 2 * r) / (omega ** 2 * r) < 0.05


def test_rotational_bound_is_zero_without_rotation():
    # constant orientation, accelerating straight line: nothing to attribute
    samples = [(i * 0.002, (0.5 * 3.0 * (i * 0.002) ** 2, 0.0, 0.0),
                (0.0, 0.0, 0.0, 1.0)) for i in range(200)]
    bound = compute_rotational_bound(samples, com_offset_m=0.2)
    assert bound is not None
    assert bound["rotational_bound"] < 0.1   # m/s^2, pure numerical noise


def test_rotational_bound_is_none_for_legacy_traces_without_orientation():
    samples = [(i * 0.02, (i * 0.02, 0.0, 0.0), None) for i in range(50)]
    assert compute_rotational_bound(samples, com_offset_m=0.2) is None


def test_com_offset_is_read_from_the_model_sdf_inertial_pose(tmp_path):
    sdf = tmp_path / "model.sdf"
    sdf.write_text(
        "<sdf><model><link><inertial>\n"
        "  <pose>0.000000 0.000012 0.200000 0 0 0</pose>\n"
        "  <mass>3.0</mass>\n"
        "</inertial></link></model></sdf>", encoding="utf-8")
    assert abs(com_offset_m(str(sdf)) - 0.2) < 1e-6


def test_diverter_gentler_than_pusher_on_synthetic_traces():
    # guided: ramps to 1 m/s; launched: ramps to 2.5 m/s in a third the time
    guided = [(i * 0.02, min(1.0, 0.05 * i) * i * 0.02, 0.0, 0.0) for i in range(60)]
    launched = [(i * 0.02, (2.5 if i > 5 else 0.0) * i * 0.02, 0.0, 0.0) for i in range(60)]
    g = compute_peaks(guided, mass=3.0)
    p = compute_peaks(launched, mass=3.0)
    assert g["peak_speed"] < p["peak_speed"]
    assert g["peak_impulse"] < p["peak_impulse"]
