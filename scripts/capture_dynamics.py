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
    r'pose\s*\{[^{}]*?name:\s*"([^"]*)"'           # model name
    r'.*?position\s*\{([^{}]*)\}'                  # its position block body
    r'(?:\s*orientation\s*\{([^{}]*)\})?',         # its quaternion, when printed
    re.S)
_COORD_RE = {c: re.compile(rf"\b{c}:\s*(-?[\d.eE+-]+)") for c in "xyzw"}


def _split_messages(text):
    """Yield each message's text. A new message starts at every `header {`."""
    starts = [m.start() for m in re.finditer(r"^header\s*\{", text, re.M)]
    for i, s in enumerate(starts):
        end = starts[i + 1] if i + 1 < len(starts) else len(text)
        yield text[s:end]


def _coords(body, keys):
    """Coordinates named `keys` from one protobuf block body; absent fields are
    exact zeros the printer omitted (this holds for the quaternion too: a
    180-degree turn prints only its unit axis component, w=0 is OMITTED —
    defaulting w to 1 would corrupt exactly the fastest-turning samples)."""
    out = []
    for c in keys:
        m = _COORD_RE[c].search(body)
        out.append(float(m.group(1)) if m else 0.0)
    return tuple(out)


def parse_trace_full(text, name="item"):
    """Extract [(t_s, (x, y, z), quat_or_None), ...] for model `name`.

    t_s is the message sim-time stamp (sec + nsec/1e9). Messages without the
    named model (it did not move that tick) are skipped. quat is (x, y, z, w),
    unit-normalized; None when the trace has no orientation block (a legacy
    dump) or the printed values are degenerate.
    """
    samples = []
    for msg in _split_messages(text):
        stamp = _STAMP_RE.search(msg)
        if not stamp:
            continue
        t = int(stamp.group(1)) + int(stamp.group(2)) / 1e9
        for pose_name, body, quat_body in _POSE_RE.findall(msg):
            if pose_name != name:
                continue
            pos = _coords(body, "xyz")
            quat = None
            if quat_body:
                qx, qy, qz, qw = _coords(quat_body, "xyzw")
                norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
                if 0.5 < norm < 2.0:  # a real unit quaternion, allowing print rounding
                    quat = (qx / norm, qy / norm, qz / norm, qw / norm)
            samples.append((t, pos, quat))
            break
    return samples


def parse_trace(text, name="item"):
    """Extract [(t_s, x, y, z), ...] for model `name`, ordered by the trace."""
    return [(t, x, y, z) for t, (x, y, z), _ in parse_trace_full(text, name)]


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


def _angular_velocity(quats, min_dt):
    """Finite-difference [(t, quat)] -> [(t_mid, omega_vec)] (rad/s, world frame).

    Same min_dt denoising as _derivative. The step rotation is dq = q2 * conj(q1)
    (world-frame composition, matching the world-frame positions being scored);
    q2 is sign-flipped onto q1's hemisphere first — q and -q are the same
    rotation, and the printer is free to switch between them mid-trace.
    """
    out = []
    last_t, last_q = quats[0]
    for t, q in quats[1:]:
        dt = t - last_t
        if dt < min_dt:
            continue
        x1, y1, z1, w1 = last_q
        x2, y2, z2, w2 = q
        if x1 * x2 + y1 * y2 + z1 * z2 + w1 * w2 < 0.0:
            x2, y2, z2, w2 = -x2, -y2, -z2, -w2
        dw = w2 * w1 + x2 * x1 + y2 * y1 + z2 * z1
        dx = -w2 * x1 + x2 * w1 - y2 * z1 + z2 * y1
        dy = -w2 * y1 + x2 * z1 + y2 * w1 - z2 * x1
        dz = -w2 * z1 - x2 * y1 + y2 * x1 + z2 * w1
        v = math.sqrt(dx * dx + dy * dy + dz * dz)
        angle = 2.0 * math.atan2(v, dw)
        scale = angle / (dt * v) if v > 1e-12 else 0.0
        out.append(((t + last_t) / 2.0, (dx * scale, dy * scale, dz * scale)))
        last_t, last_q = t, q
    return out


def compute_rotational_bound(samples, com_offset_m, min_dt=0.01):
    """Upper bound of the rotation-induced share of the measured peak accel.

    The trace is the model ORIGIN — a body-fixed point at |com_offset_m| from the
    centre of mass (the <inertial><pose> build_item_models writes). Rigid-body
    kinematics: a_origin = a_com + alpha x r + omega x (omega x r), so at the
    moment of the measured peak the rotation contributes at most
    (|omega|^2 + |alpha|) * r. The true COM accel then lies within
    peak_accel +- rotational_bound — the answer to "is the peak the goods being
    hit, or the ruler's point swinging around it".

    samples: parse_trace_full output. Returns None when the trace carries no
    usable orientation (legacy dump) or is too short to differentiate.
    """
    quats = [(t, q) for t, _, q in samples if q is not None]
    if len(quats) < 3 or com_offset_m <= 0.0:
        return None
    vel = _derivative([(t, p) for t, p, _ in samples], min_dt)
    acc = _derivative(vel, min_dt) if vel else []
    if not acc:
        return None
    t_peak, a_peak = max(acc, key=lambda ta: math.dist(ta[1], (0, 0, 0)))
    omega = _angular_velocity(quats, min_dt)
    if not omega:
        return None
    alpha = _derivative(omega, min_dt)
    omega_at = math.dist(min(omega, key=lambda tv: abs(tv[0] - t_peak))[1], (0, 0, 0))
    alpha_at = (math.dist(min(alpha, key=lambda tv: abs(tv[0] - t_peak))[1], (0, 0, 0))
                if alpha else 0.0)
    return {
        "peak_accel": math.dist(a_peak, (0, 0, 0)),
        "t_peak": t_peak,
        "omega_at_peak": omega_at,
        "alpha_at_peak": alpha_at,
        "rotational_bound": (omega_at ** 2 + alpha_at) * com_offset_m,
    }


def com_offset_m(sdf_path):
    """|origin -> centre of mass| (m) from the model's <inertial><pose>.

    build_item_models.py writes the hull COM there, about the set_belt_origin
    frame — the same origin the pose trace reports, so this is exactly the
    lever arm r of the rotational terms above."""
    with open(sdf_path, encoding="utf-8") as f:
        m = re.search(r"<inertial>.*?<pose>([^<]+)</pose>", f.read(), re.S)
    if not m:
        return 0.0
    x, y, z = (float(v) for v in m.group(1).split()[:3])
    return math.sqrt(x * x + y * y + z * z)


def main():
    ap = argparse.ArgumentParser(description="Gentleness metric from a pose trace")
    ap.add_argument("trace_file")
    ap.add_argument("--name", default="item")
    ap.add_argument("--mass", type=float, default=1.0)
    ap.add_argument("--model-sdf", default=None,
                    help="item model.sdf; enables the rotational bound on peak_accel "
                         "(lever arm r = its <inertial><pose> COM offset)")
    args = ap.parse_args()
    with open(args.trace_file, encoding="utf-8", errors="replace") as f:
        text = f.read()
    samples = parse_trace_full(text, args.name)
    peaks = compute_peaks([(t, x, y, z) for t, (x, y, z), _ in samples], mass=args.mass)
    line = (f"gentleness: peak_speed={peaks['peak_speed']:.3f} m/s "
            f"peak_accel={peaks['peak_accel']:.3f} m/s^2 "
            f"peak_impulse={peaks['peak_impulse']:.3f} N*s")
    if args.model_sdf:
        r = com_offset_m(args.model_sdf)
        bound = compute_rotational_bound(samples, r)
        if bound is not None:
            line += (f" peak_accel_rotational_bound={bound['rotational_bound']:.3f} m/s^2"
                     f" (r={r:.3f} m, |omega|={bound['omega_at_peak']:.2f} rad/s"
                     f" at t={bound['t_peak']:.2f})")
    print(line)


if __name__ == "__main__":
    main()
