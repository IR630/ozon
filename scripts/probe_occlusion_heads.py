#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Do the side heads win anything where the TOP head lies BY CONSTRUCTION?

WHY THIS EXISTS. On the organizers' 11 items the second and third heads buy
nothing measurable: by the organizers' own tolerance the rig scores 19/23 with
two heads and 19/23 with one (`scripts/census_tolerance.py`, 22.07). That is not
evidence that the rig is useless — it is evidence that the CATALOGUE contains no
body the top head gets wrong. A top-down head is wrong by construction only on
bodies whose silhouette is not their footprint: an overhang hiding a narrower
waist, a cavity underneath, a hole through the middle. The catalogue has none.

Occlusion is the one real-line phenomenon our simulator models CORRECTLY — it is
geometry, and the renderer computes it exactly. So this is the only class of
scene where the rig can be justified by our own stand instead of by calculation.

WHAT IT DOES. Runs the two procedural antagonists from `build_probe_items.py`
(`u_bracket` — an overhang with a cavity under it; `ring` — a hollow body with
belt visible through the middle) through the FULL contour twice: once in the
one-head world, once in the three-head world. Both runs measure the same body in
the same seeded resting poses, so the difference between them is the rig and
nothing else. Truth is the probe's analytic AABB — known from the drawing, never
read off our own output.

    python3 scripts/probe_occlusion_heads.py                 # both configs, 4 cells each
    python3 scripts/probe_occlusion_heads.py --logdir runs/occl_x

WHAT IT DOES NOT SHOW. Routing (these probes are not the organizers' items and
their zone is a property of the drawing, not of the sorter's job), throughput, or
anything about noise — the worlds have none. It answers exactly one question:
does a head that sees the flank recover extent the top head cannot see.
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from build_probe_items import PROBES  # noqa: E402
from spawn_orientations import spawn_pose_for_mesh_m  # noqa: E402

from src.classification import measurement_error  # noqa: E402

# The two probes whose TRUE extent is hidden from a downward view, and the resting
# poses in which it is hidden. Named poses come from the probe's own definition:
# every one of them rests on a face, so the body is not balanced on an edge.
ANTAGONISTS = (("u_bracket", "opening_down"), ("u_bracket", "opening_up"),
               ("ring", "flat"), ("ring", "yaw45"))

# (label, world, bridge config). Same contour, same items, different rig.
# MONOTONIC IN HEAD COUNT, so the table reads 1 -> 2 -> 3 and the marginal value of
# each head is the difference between adjacent rows. The two-head rig was missing
# until 25.07, which left the ONE comparison this stand can make honestly — does the
# THIRD head buy anything the second did not — unmeasured: occlusion is geometry and
# the renderer computes it exactly, unlike noise, glare and drift, which our worlds
# do not model at all. The bridge name carries NO `sim/` prefix here: this list is
# consumed by run_skeleton.sh -> launch/skeleton.launch.py, which prefixes it itself
# (scripts/dump_item_frame.sh wants the opposite — the prefixed form).
CONFIGS = (("1 head", "sim/worlds/cell_diverter.sdf", "bridge.yaml"),
           ("2 heads", "sim/worlds/cell_diverter_2cam.sdf", "bridge_2cam.yaml"),
           ("3 heads", "sim/worlds/cell_diverter_3cam.sdf", "bridge_3cam.yaml"))

# `item 1: 347x299x278 mm K=0.70 at (1.87, 0.08) heads=2` — the production
# perception line, the same one census_tolerance.py scores.
_MEASUREMENT_RE = re.compile(
    r"item \d+: (\d+)x(\d+)x(\d+) mm K=([\d.]+).*?(?:heads=(\d+))?$")


def pose_quat(axis, degrees):
    """Unit quaternion (x, y, z, w) of a named probe pose."""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    half = np.radians(float(degrees)) / 2.0
    x, y, z = axis * np.sin(half)
    return (float(x), float(y), float(z), float(np.cos(half)))


def spawn_pose(slug, quat):
    """(spawn z, spawn y) that rest this probe's body on the middle of the belt."""
    return spawn_pose_for_mesh_m(PROBES[slug].build(), quat)


def parse_measurement(text):
    """(dims mm desc, K, heads) of the LAST perception line, or None.

    The last line is the freshest reading before the verdict — the same choice
    census_tolerance.py makes, so the two are comparable.
    """
    found = None
    for line in text.splitlines():
        match = _MEASUREMENT_RE.search(line.strip())
        if match:
            dims = sorted((float(match[1]), float(match[2]), float(match[3])), reverse=True)
            found = (dims, float(match[4]), int(match[5]) if match[5] else 1)
    return found


def run_cell(slug, pose_name, world, bridge, logdir, timeout_s=300):
    """Run one probe pose through the contour; return the episode log text."""
    axis, degrees = next((ax, deg) for name, ax, deg in PROBES[slug].poses if name == pose_name)
    quat = pose_quat(axis, degrees)
    spawn_z, spawn_y = spawn_pose(slug, quat)
    # The three-head rig boots visibly slower (bringup 26.9 s against 19.7 s for two
    # heads, docs/experiments.md 22.07), and the default 60 soft-start polls abort it
    # with "belt never reached full speed" before the controller is up. The earlier
    # three-head censuses raised this for the same reason; a comparison where one rig
    # never starts is not a comparison.
    env = dict(os.environ,
               WORLD=world,
               BRIDGE_CONFIG=bridge,
               SOFT_START_TRIES=os.environ.get("SOFT_START_TRIES", "200"),
               ITEM_MODEL_ROOT="sim/models/probe_items",
               ORIENT_X=f"{quat[0]:.9f}", ORIENT_Y=f"{quat[1]:.9f}",
               ORIENT_Z=f"{quat[2]:.9f}", ORIENT_W=f"{quat[3]:.9f}",
               SPAWN_Z=f"{spawn_z:.6f}", SPAWN_Y=f"{spawn_y:.6f}")
    runner = os.environ.get("SKELETON", "bash scripts/run_skeleton.sh").split()
    log = Path(logdir) / f"{slug}_{pose_name}_{Path(world).stem}.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    # THIS CELL'S NODE LOG, NOT THE MACHINE'S. run_skeleton.sh defaults to the
    # shared /tmp/skeleton_e2e.log, so a cell that timed out used to recover its
    # "measurement" from whatever episode ran last — observed 23.07: a wedged
    # probe cell reported 401x400x302 heads=1 from a two-head census running in
    # another worktree. Per-cell path makes the recovery honest and keeps the
    # dumped log next to the cell it belongs to.
    env["NODE_LOG"] = str(log.with_suffix(".node.log"))
    try:
        done = subprocess.run(runner + [slug, PROBES[slug].expected], cwd=str(ROOT),
                              env=env, capture_output=True, text=True, timeout=timeout_s)
        text = done.stdout + done.stderr
    except subprocess.TimeoutExpired:
        # A wedged episode is a result, not a crash: record it and keep the sweep
        # moving, exactly as run_matrix.sh caps its cells.
        text = "TIMEOUT: episode exceeded the cell ceiling\n"
    text += _node_measurements(env["NODE_LOG"])
    log.write_text(text)
    return text


def _node_measurements(node_log_path=None):
    """Perception lines of the episode that just ended, from the node log.

    run_skeleton.sh echoes only the LAST THREE `item N:` lines, and on a good
    episode those are often the classifier's and the controller's — a cell then
    reads "no measurement in log" while the measurement exists. The path is the
    CELL'S OWN log (run_cell sets NODE_LOG), so this reads that episode and no
    other; the env fallback stays for a caller that drives run_skeleton.sh itself.
    """
    node_log = Path(node_log_path or os.environ.get("NODE_LOG", "/tmp/skeleton_e2e.log"))
    if not node_log.exists():
        return ""
    lines = [line for line in node_log.read_text(errors="replace").splitlines()
             if _MEASUREMENT_RE.search(line.strip())]
    return "".join(f"{line}\n" for line in lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--logdir", default="runs/occlusion_heads")
    parser.add_argument("--timeout", type=int, default=300, help="per-cell ceiling, s")
    args = parser.parse_args(argv)

    print("antagonist scenes: the top head is wrong BY CONSTRUCTION here\n")
    print(f"{'probe / pose':<26} {'rig':<9} {'measured mm':>20} {'truth mm':>18} "
          f"{'side err':>9} {'heads':>6}")

    results = {}
    for slug, pose_name in ANTAGONISTS:
        truth = PROBES[slug].dims_mm
        for label, world, bridge in CONFIGS:
            text = run_cell(slug, pose_name, world, bridge, args.logdir, args.timeout)
            parsed = parse_measurement(text)
            cell = f"{slug} / {pose_name}"
            if parsed is None:
                print(f"{cell:<26} {label:<9} {'no measurement in log':>20}")
                continue
            dims, _k, heads = parsed
            side, _vol = measurement_error(dims, truth)
            results[(slug, pose_name, label)] = side
            print(f"{cell:<26} {label:<9} "
                  f"{'x'.join(f'{d:.0f}' for d in dims):>20} "
                  f"{'x'.join(f'{d:.0f}' for d in truth):>18} "
                  f"{side:8.1f} {heads:6d}")

    print("\ndelta on the ONE question this probe exists to answer:")
    for slug, pose_name in ANTAGONISTS:
        one = results.get((slug, pose_name, "1 head"))
        two = results.get((slug, pose_name, "2 heads"))
        three = results.get((slug, pose_name, "3 heads"))
        if one is None or three is None:
            print(f"  {slug} / {pose_name:<14} incomplete — a cell produced no measurement")
            continue
        verdict = "side heads WIN" if three < one - 1.0 else (
            "no gain" if three <= one + 1.0 else "side heads LOSE")
        # The MARGINAL head, reported separately: the whole rig question is not
        # "do side heads help" but "does the THIRD one help once the second is
        # paid for". 1 mm is the reporting floor — below it two runs of the same
        # rig differ anyway, so a smaller gap is not a gain.
        two_txt = "  2 heads   n/a" if two is None else f"  2 heads {two:6.1f} mm"
        marginal = ("unknown" if two is None else
                    f"3rd head {two - three:+.1f} mm over the 2nd"
                    + ("" if abs(two - three) > 1.0 else " (within the 1 mm floor)"))
        print(f"  {slug} / {pose_name:<14} 1 head {one:6.1f} mm ->{two_txt} -> "
              f"3 heads {three:6.1f} mm   {verdict}; {marginal}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
