# -*- coding: utf-8 -*-
"""Unit tests for the gentleness metric (scripts/capture_dynamics.py).

Pure Python — no Gazebo. parse_trace is checked on a hand-written slice of real
`ign topic -e` output; compute_peaks on synthetic trajectories with known peaks.
"""
from capture_dynamics import compute_peaks, parse_trace

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


def test_diverter_gentler_than_pusher_on_synthetic_traces():
    # guided: ramps to 1 m/s; launched: ramps to 2.5 m/s in a third the time
    guided = [(i * 0.02, min(1.0, 0.05 * i) * i * 0.02, 0.0, 0.0) for i in range(60)]
    launched = [(i * 0.02, (2.5 if i > 5 else 0.0) * i * 0.02, 0.0, 0.0) for i in range(60)]
    g = compute_peaks(guided, mass=3.0)
    p = compute_peaks(launched, mass=3.0)
    assert g["peak_speed"] < p["peak_speed"]
    assert g["peak_impulse"] < p["peak_impulse"]
