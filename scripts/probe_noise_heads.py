#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Does a SECOND or THIRD head buy anything once the depth is noisy — offline.

WHY THIS EXISTS. `scripts/probe_sensor_noise.py` answered "how much noise does the
measurement survive" for the TOP HEAD ALONE, and said so in its own docstring: no
`depth_side_*` frames existed, so the one-vs-two-vs-three-head delta under noise
stayed unmeasured. That delta is the whole case for the rig. Our worlds have no
noise model at all, so in the stand the extra heads win nothing — the top head
never loses. Noise is the ONLY phenomenon we can inject offline that the extra heads
are supposed to defend against, and this probe injects it into EVERY head at once
and pushes the result through the SAME `src.multiview` fusion the ROS node runs.

WHAT THE NUMBER IS AND IS NOT — read before quoting it anywhere:

  * It is a property of the GEOMETRY OF MEASUREMENT under sensor noise: still
    frames, one item, fusion at dt = 0.
  * SYNC IS NOT MODELLED, AND THAT FLATTERS THE RIG. In the contour the heads are
    untriggered at 15 Hz, so two frames can be 66.7 ms apart — 66.7 mm of belt
    travel that `compensate_belt_travel` corrects only to the residual. Here the
    dumped frames carry no usable timestamps, so the clouds are fused as if all
    heads fired together. Every head count above 1 is therefore an OPTIMISTIC
    bound, and the more heads, the more optimistic.
  * It does NOT add up with the census — a census cell is a routing verdict on a
    moving item (the 92 % lesson, `docs/report/methodology_and_limitations.md`).
  * K is untouched: fusion moves dims only, exactly as in production.

Ground truth is the mesh OBB (`analyze_models.analyze_file`), the same truth
`scripts/census_tolerance.py` and `probe_sensor_noise.py` use.

Dump the frames first (this probe never starts Gazebo):

    SIDE=1 WORLD=sim/worlds/cell_diverter_3cam.sdf BRIDGE_CONFIG=sim/bridge_3cam.yaml \
        OUT=runs/frames/helmet_3cam bash scripts/dump_item_frame.sh helmet

    python3 scripts/probe_noise_heads.py runs/frames/helmet_3cam
    python3 scripts/probe_noise_heads.py --trials 20 --seed 7 --sigmas 0,0.001,0.005
"""
from __future__ import annotations

import argparse
import sys
import zlib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from analyze_models import analyze_file  # noqa: E402
from build_item_models import ITEMS, STL_DIR  # noqa: E402
from probe_sensor_noise import add_depth_noise, slug_of_dir  # noqa: E402

from src.classification import (  # noqa: E402
    measurement_error,
    within_measurement_tolerance,
)
from src.constants import (  # noqa: E402
    CAMERA_SIDE_NEG_Y_POSE_M,
    CAMERA_SIDE_POS_Y_POSE_M,
)
from src.multiview import crop_to_item, fuse_dims_mm, world_cloud_from_depth  # noqa: E402
from src.perception import FX, FY, load_depth_png, measure_items  # noqa: E402

# The rigs, in the order a report compares them. The head list is the SIDE heads
# only — the top head is in every configuration and is what finds the item at all.
RIGS = (("1 head (top)", ()),
        ("2 heads (+neg_y)", (("depth_side_neg_y", CAMERA_SIDE_NEG_Y_POSE_M),)),
        ("3 heads (+both)", (("depth_side_neg_y", CAMERA_SIDE_NEG_Y_POSE_M),
                             ("depth_side_pos_y", CAMERA_SIDE_POS_Y_POSE_M))))

SIGMAS_FRAC = (0.0, 0.0005, 0.001, 0.002, 0.005)
DEFAULT_TRIALS = 10
DEFAULT_SEED = 0


def measure_with_rig(top_depth_m, side_depths_m, side_poses):
    """Fused dims (mm) of the largest body, or None if the top head found nothing.

    Mirrors `perception_node.on_depth`: the top head owns detection, position and
    id; the side heads only contribute points inside the box the top head claimed.
    dt = 0 — see the sync caveat in the module docstring.
    """
    measurements = measure_items(top_depth_m)
    if not measurements:
        return None, 0
    measurement = max(measurements, key=lambda m: float(np.prod(m.dims_mm)))
    clouds = []
    for depth_m, pose in zip(side_depths_m, side_poses):
        pts = world_cloud_from_depth(depth_m, pose, FX, FY)
        clouds.append(crop_to_item(pts, measurement.position_m, measurement.dims_mm))
    dims_mm = fuse_dims_mm(measurement.dims_mm, clouds, measurement.position_m)
    return list(dims_mm), len(measurements)


def probe_rig(top_depth_m, side_depths_m, side_poses, truth_dims_mm, sigma, trials, rng):
    """(detections, side errors mm, tolerance passes, bodies) over `trials` draws.

    Noise is drawn INDEPENDENTLY for every head in the draw — one sensor's error
    does not correlate with another's, which is the whole reason a rig can average
    a body out of the noise.
    """
    side_errs, passes, detected, bodies = [], 0, 0, []
    for _ in range(trials):
        noisy_top = add_depth_noise(top_depth_m, sigma, rng)
        noisy_sides = [add_depth_noise(d, sigma, rng) for d in side_depths_m]
        dims_mm, n_bodies = measure_with_rig(noisy_top, noisy_sides, side_poses)
        bodies.append(n_bodies)
        if dims_mm is None:
            continue
        detected += 1
        side, _vol = measurement_error(dims_mm, truth_dims_mm)
        side_errs.append(side)
        passes += within_measurement_tolerance(dims_mm, truth_dims_mm)
    return detected, side_errs, passes, bodies


def load_rig_frames(dump_dir, frame_index=0):
    """(top depth, {name: depth}) of one dumped moment, in metres."""
    tops = sorted(p for p in Path(dump_dir).glob("depth_*.png") if "_side_" not in p.name)
    if not tops:
        raise FileNotFoundError(f"no top depth_*.png in {dump_dir}")
    sides = {}
    for name, _pose in RIGS[-1][1]:
        frames = sorted(Path(dump_dir).glob(f"{name}_*.png"))
        if frames:
            sides[name] = load_depth_png(str(frames[min(frame_index, len(frames) - 1)]))
    top = load_depth_png(str(tops[min(frame_index, len(tops) - 1)]))
    return top, sides


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("dirs", nargs="*", help="dump dirs (default: every runs/frames/*)")
    parser.add_argument("--trials", type=int, default=DEFAULT_TRIALS)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED,
                        help="mandatory reproducibility seed")
    parser.add_argument("--sigmas", default=None,
                        help="comma-separated fractions of range, e.g. 0,0.001,0.005")
    args = parser.parse_args(argv)
    sigmas = ([float(s) for s in args.sigmas.split(",")] if args.sigmas else list(SIGMAS_FRAC))

    dirs = [Path(d) for d in args.dirs] or sorted(
        d for d in (ROOT / "runs" / "frames").glob("*") if d.is_dir())
    if not dirs:
        print("no dump dirs — run scripts/dump_item_frame.sh with SIDE=1 first")
        return 1

    truth = {slug: tuple(float(x) for x in analyze_file(STL_DIR / f"{stem}.stl")["dims"])
             for slug, (stem, _mass) in ITEMS.items()}

    print(f"seed={args.seed} trials={args.trials}  sigma = fraction of RANGE "
          f"(D435-class datasheet: 1-2 %)")
    print("fusion at dt=0 — sync penalty NOT modelled, every rig above 1 head is optimistic\n")
    print(f"{'item / rig':<34} {'sigma':>6} {'found':>7} {'side mm':>16} "
          f"{'in tol':>8} {'bodies':>7}")

    for dump_dir in dirs:
        try:
            top, sides = load_rig_frames(dump_dir)
        except FileNotFoundError as exc:
            print(f"{dump_dir.name:<34} {exc}")
            continue
        slug = slug_of_dir(dump_dir.name)
        for rig_name, heads in RIGS:
            if any(name not in sides for name, _pose in heads):
                print(f"{dump_dir.name + ' / ' + rig_name:<34} "
                      f"no side frames dumped — rig skipped")
                continue
            side_depths = [sides[name] for name, _pose in heads]
            side_poses = [pose for _name, pose in heads]
            for sigma in sigmas:
                # crc32, NOT hash(): Python randomizes string hashing per process,
                # so a hash-seeded generator would make this report unreproducible.
                rng = np.random.default_rng(
                    [args.seed, zlib.crc32(f"{dump_dir.name}{rig_name}".encode()),
                     int(sigma * 1e6)])
                detected, side_errs, passes, bodies = probe_rig(
                    top, side_depths, side_poses, truth[slug], sigma, args.trials, rng)
                side_txt = (f"{np.median(side_errs):6.1f} ({np.max(side_errs):5.1f} max)"
                            if side_errs else "  -- lost --   ")
                print(f"{dump_dir.name + ' / ' + rig_name:<34} {100 * sigma:5.2f}% "
                      f"{detected:3d}/{args.trials:<3d} {side_txt:>16} "
                      f"{passes:4d}/{args.trials:<3d} {np.median(bodies):6.1f}")
            print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
