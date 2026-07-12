# -*- coding: utf-8 -*-
"""Gentleness metric from a Gazebo dynamic-pose trace (day 5, P1).

The 10-point "manipulation quality" criterion penalises impact. The ballistic
pusher flings the item off the belt at 2.5 m/s; the diverter guides it at belt
speed (~1 m/s). To choose the mechanism on numbers, we measure how roughly the
item is handled: peak speed, peak acceleration, and peak linear impulse it takes
during the episode.

Nothing in the sim publishes velocity, so we differentiate the item's pose. The
SceneBroadcaster already streams every dynamic model's pose on
/world/cell/dynamic_pose/info; run_skeleton.sh (CAPTURE_DYNAMICS=1) dumps that
topic to a file with `ign topic -e` and hands it here. The parse/peaks core is
pure Python (no rclpy, no Gazebo) so it unit-tests on Windows against a saved
trace, matching tools/precision_sweep.py's offline style.

    python3 scripts/capture_dynamics.py <trace_file> [--name item] [--mass KG]

Emits one grep-able line for the comparison harness:
    gentleness: peak_speed=.. m/s peak_accel=.. m/s^2 peak_impulse=.. N*s
"""
import argparse
import math
import re

# `ign topic -e` prints protobuf debug text: one `header { stamp { sec nsec } }`
# per message, then several `pose { name position { x y z } }` blocks. Absent
# coords (exact zeros) are omitted by the printer, so every field defaults to 0.
_STAMP_RE = re.compile(r"stamp\s*\{\s*sec:\s*(-?\d+)\s*nsec:\s*(-?\d+)", re.S)
_POSE_RE = re.compile(
    r'pose\s*\{[^{}]*?name:\s*"([^"]*)"'          # model name
    r'.*?position\s*\{([^{}]*)\}',                 # its position block body
    re.S)
_COORD_RE = {c: re.compile(rf"\b{c}:\s*(-?[\d.eE+-]+)") for c in "xyz"}


def _split_messages(text):
    """Yield each message's text. A new message starts at every `header {`."""
    starts = [m.start() for m in re.finditer(r"^header\s*\{", text, re.M)]
    for i, s in enumerate(starts):
        end = starts[i + 1] if i + 1 < len(starts) else len(text)
        yield text[s:end]


def parse_trace(text, name="item"):
    """Extract [(t_s, x, y, z), ...] for model `name`, ordered by the trace.

    t_s is the message sim-time stamp (sec + nsec/1e9). Messages without the
    named model (it did not move that tick) are skipped.
    """
    samples = []
    for msg in _split_messages(text):
        stamp = _STAMP_RE.search(msg)
        if not stamp:
            continue
        t = int(stamp.group(1)) + int(stamp.group(2)) / 1e9
        for pose_name, body in _POSE_RE.findall(msg):
            if pose_name != name:
                continue
            coord = {}
            for c, rx in _COORD_RE.items():
                m = rx.search(body)
                coord[c] = float(m.group(1)) if m else 0.0
            samples.append((t, coord["x"], coord["y"], coord["z"]))
            break
    return samples


def _derivative(series, min_dt):
    """Finite-difference (t, vec3) -> (t_mid, vec3) skipping sub-min_dt steps.

    The pose stream runs at physics rate, so raw step-to-step differences on
    quantised positions are noisy; requiring a minimum dt between the two
    samples denoises the velocity/acceleration without extra libraries.
    """
    out = []
    last_t, last_v = series[0]
    for t, v in series[1:]:
        dt = t - last_t
        if dt < min_dt:
            continue
        deriv = tuple((v[i] - last_v[i]) / dt for i in range(3))
        out.append(((t + last_t) / 2.0, deriv))
        last_t, last_v = t, v
    return out


def compute_peaks(samples, mass=1.0, min_dt=0.01):
    """Peak speed (m/s), acceleration (m/s^2) and impulse (N*s) over the trace.

    peak_impulse = mass * peak_speed is the item's peak linear momentum — the
    "how hard is it flung" number that separates a guided item from a launched
    one. Returns zeros for a trace too short to differentiate.
    """
    if len(samples) < 3:
        return {"peak_speed": 0.0, "peak_accel": 0.0, "peak_impulse": 0.0}
    pos = [(t, (x, y, z)) for (t, x, y, z) in samples]
    vel = _derivative(pos, min_dt)
    if not vel:
        return {"peak_speed": 0.0, "peak_accel": 0.0, "peak_impulse": 0.0}
    peak_speed = max(math.dist(v, (0, 0, 0)) for _, v in vel)
    acc = _derivative(vel, min_dt)
    peak_accel = max((math.dist(a, (0, 0, 0)) for _, a in acc), default=0.0)
    return {
        "peak_speed": peak_speed,
        "peak_accel": peak_accel,
        "peak_impulse": mass * peak_speed,
    }


def main():
    ap = argparse.ArgumentParser(description="Gentleness metric from a pose trace")
    ap.add_argument("trace_file")
    ap.add_argument("--name", default="item")
    ap.add_argument("--mass", type=float, default=1.0)
    args = ap.parse_args()
    with open(args.trace_file, encoding="utf-8", errors="replace") as f:
        text = f.read()
    peaks = compute_peaks(parse_trace(text, args.name), mass=args.mass)
    print(f"gentleness: peak_speed={peaks['peak_speed']:.3f} m/s "
          f"peak_accel={peaks['peak_accel']:.3f} m/s^2 "
          f"peak_impulse={peaks['peak_impulse']:.3f} N*s")


if __name__ == "__main__":
    main()
