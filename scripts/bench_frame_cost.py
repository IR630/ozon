#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""What one frame costs the rig, split into the part hardware can move and the part it cannot.

WHY THIS EXISTS. The per-frame numbers the camera decision rests on (32 / 51 / 72 ms
for 1 / 2 / 3 heads, docs/cameras-under-noise-brief.md) were taken ad hoc on a stand
with no GPU, and the conclusion drawn from them — "the third head does not fit the
66.7 ms camera tick" — is the single hardest limit on the shipped rig. Before that
conclusion is revisited on faster hardware, the measurement has to answer a question
the old one could not: WHICH PART OF THE COST IS THE RENDERER AND WHICH IS OURS.

That split decides whether a GPU changes anything at all:

  * `measure_items` and the side-head backprojection are NUMPY ON THE CPU. No GPU
    executes them. Their cost is a property of the code and the CPU, and it is what
    a different expert stand will inherit.
  * The renderer (Gazebo/ogre) is the part a GPU offloads. Under llvmpipe it burns
    the same cores perception needs, so it inflates the numbers above INDIRECTLY,
    through core contention — not by being inside the frame path.

So a GPU can only ever buy back the contention term. This script measures the
numpy path with nothing else running, which is the number that transfers.

WHY THE CPU NUMBER IS THE ONE THAT DECIDES. The organizers' stand "будет с GPU, но
модель, объём памяти, драйвер и CUDA пока не определены" and they require a
"проверяемый fallback: CPU-сценарий" (docs/md/organizer_faq_2026-07-16.md). A verdict
of "it fits the tick" earned on one particular fast card is therefore not a verdict
about their stand. Decide on the CPU/headless path; quote the GPU figure as an upper
bound and say which card produced it.

    python3 scripts/bench_frame_cost.py runs/frames/plate_3cam
    python3 scripts/bench_frame_cost.py runs/frames/plate_3cam --sigma 0.01 --repeats 20

The frame period is `CAMERA_TICK_MS` below: 640x480 at 15 Hz, from CLAUDE.md.
"""
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from probe_sensor_noise import add_depth_noise  # noqa: E402

from src.constants import (  # noqa: E402
    CAMERA_SIDE_NEG_Y_POSE_M,
    CAMERA_SIDE_POS_Y_POSE_M,
)
from src.multiview import crop_to_item, fuse_dims_mm, world_cloud_from_depth  # noqa: E402
from src.perception import HFOV_RAD, load_depth_png, measure_items  # noqa: E402

# 640x480 at 15 Hz (CLAUDE.md). A frame path slower than this cannot keep up with
# the camera, and a verdict for a moving item then arrives after the diverter.
CAMERA_TICK_MS = 1000.0 / 15.0

# Side heads in rig order, exactly as perception_node.py subscribes them.
SIDE_HEADS = (("depth_side_neg_y", CAMERA_SIDE_NEG_Y_POSE_M),
              ("depth_side_pos_y", CAMERA_SIDE_POS_Y_POSE_M))


def load_rig_frames(dump_dir, frame_index=0):
    """One top frame and whichever side frames the dump carries (metres)."""
    dump_dir = Path(dump_dir)
    tops = sorted(dump_dir.glob("depth_0*.png"))
    if not tops:
        raise SystemExit(f"ABORT: no top depth frames in {dump_dir}")
    top = load_depth_png(str(tops[min(frame_index, len(tops) - 1)]))
    sides = {}
    for name, _pose in SIDE_HEADS:
        frames = sorted(dump_dir.glob(f"{name}_*.png"))
        if frames:
            sides[name] = load_depth_png(str(frames[min(frame_index, len(frames) - 1)]))
    return top, sides


def _side_clouds(sides, heads):
    """Backprojected side clouds, with each head's OWN focal length.

    Mirrors perception_node._side_world_clouds, including the detail that cost a
    silent bug there: the side heads are 320x240 while the top is 640x480, so the
    top head's fx/fy must NOT be reused for them.
    """
    clouds = []
    for name, pose in heads:
        depth = sides[name]
        h_side, w_side = depth.shape
        fx_side = fy_side = (w_side / 2.0) / np.tan(HFOV_RAD / 2.0)
        clouds.append(world_cloud_from_depth(depth, pose, fx_side, fy_side,
                                             w_side / 2.0, h_side / 2.0))
    return clouds


def time_frame(top, sides, heads, repeats):
    """Median ms for the top measurement and for the side work, timed apart.

    Median, not mean: one scheduler hiccup on a busy box otherwise moves the number
    more than the thing being measured. Reported separately because they answer
    different questions — the top cost is paid by EVERY rig including the shipped
    one-head fallback, the side cost is what the extra heads actually add.
    """
    top_ms, side_ms = [], []
    for _ in range(repeats):
        t0 = time.perf_counter()
        measurements = measure_items(top)
        t1 = time.perf_counter()
        top_ms.append((t1 - t0) * 1000.0)

        if not heads:
            side_ms.append(0.0)
            continue
        # `and measurements` mirrors the node: an EMPTY belt backprojects nothing.
        t2 = time.perf_counter()
        if measurements:
            clouds_world = _side_clouds(sides, heads)
            for measurement in measurements:
                cropped = [crop_to_item(pts, measurement.position_m, measurement.dims_mm)
                           for pts in clouds_world]
                fuse_dims_mm(measurement.dims_mm, cropped, measurement.position_m)
        t3 = time.perf_counter()
        side_ms.append((t3 - t2) * 1000.0)
    return statistics.median(top_ms), statistics.median(side_ms), len(measurements)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("dump_dir", help="frame dump from scripts/dump_item_frame.sh")
    parser.add_argument("--sigma", type=float, default=0.003 / 1.5,
                        help="depth noise as a FRACTION of range; default is the "
                             "3 mm at 1.5 m the census used")
    parser.add_argument("--sigma-mm", type=float, default=None,
                        help="noise in mm at the nominal 1.5 m range (overrides --sigma)")
    parser.add_argument("--repeats", type=int, default=15)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--frame-index", type=int, default=0)
    parser.add_argument("--side-scale", type=int, default=1,
                        help="upscale side frames by this factor per axis. The "
                             "published 72 ms was measured when the side heads were "
                             "640x480; 1500acb cut them to 320x240 NINE HOURS LATER, "
                             "so --side-scale 2 reconstructs the rig that number "
                             "describes and lets one machine compare both")
    args = parser.parse_args(argv)

    sigma_frac = (args.sigma_mm / 1000.0 / 1.5) if args.sigma_mm is not None else args.sigma
    sigma_mm_at_range = sigma_frac * 1.5 * 1000.0

    top_clean, sides_clean = load_rig_frames(args.dump_dir, args.frame_index)
    if args.side_scale != 1:
        # Nearest-neighbour on purpose: this reconstructs the COST of a larger side
        # frame (backprojection is per-pixel), not a sharper picture of the item.
        sides_clean = {name: np.repeat(np.repeat(depth, args.side_scale, axis=0),
                                       args.side_scale, axis=1)
                       for name, depth in sides_clean.items()}
    rng = np.random.default_rng(args.seed)
    top = add_depth_noise(top_clean, sigma_frac, rng) if sigma_frac else top_clean
    sides = {name: (add_depth_noise(depth, sigma_frac, rng) if sigma_frac else depth)
             for name, depth in sides_clean.items()}

    available = tuple((name, pose) for name, pose in SIDE_HEADS if name in sides)
    rigs = [("1 head (top)", ())]
    for i in range(1, len(available) + 1):
        rigs.append((f"{i + 1} heads", available[:i]))

    print(f"dump: {args.dump_dir}  frame {args.frame_index}  "
          f"sigma {sigma_mm_at_range:.1f} mm @1.5 m ({sigma_frac * 100:.2f} % of range)  "
          f"repeats {args.repeats}")
    print(f"camera tick: {CAMERA_TICK_MS:.1f} ms\n")
    print(f"{'rig':14}{'top ms':>10}{'sides ms':>10}{'total ms':>10}"
          f"{'% of tick':>12}  fits?")
    for label, heads in rigs:
        top_ms, side_ms, n_items = time_frame(top, sides, heads, args.repeats)
        total = top_ms + side_ms
        print(f"{label:14}{top_ms:>10.1f}{side_ms:>10.1f}{total:>10.1f}"
              f"{total / CAMERA_TICK_MS * 100:>11.0f}%  "
              f"{'yes' if total <= CAMERA_TICK_MS else 'NO'}")
    print(f"\nitems found in the frame: {n_items}")
    print("This is the NUMPY path only — no renderer runs here. A GPU does not "
          "execute it,\nso these milliseconds are what any expert stand inherits "
          "from the CPU it has.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
