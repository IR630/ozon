#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""What a LOST DEPTH RETURN costs the measurement — and whether a side head saves it.

WHY THIS EXISTS. The rig on the real line is justified by glare, black film,
transparent wrap and grazing specular sidewalls. None of them exist in our worlds:
`sim/worlds/*.sdf` renders every surface as a perfect return, so the case for extra
heads stayed a calculation with no measured number under it. We cannot render the
CAUSE — but all four causes have the SAME consequence on a depth sensor: no return
over part of the item, while the item is physically there. That consequence can be
injected into a saved frame exactly, so this probe punches the hole into the frames
`scripts/dump_item_frame.sh` already dumped and pushes the result through the SAME
`src.perception` + `src.multiview` the ROS node runs.

ZEROS, NOT "AVERAGE DEPTH". A dropped return is absence, and every stage of our
pipeline already agrees on how absence looks: the node itself maps NaN/inf to 0
before measuring (`src/perception_node.py:133` and `:93`), `_item_mask` admits only
`depth_m > 0`, and `world_cloud_from_depth` keeps only finite positive pixels. So
writing 0 into the frame is not a modelling choice — it is byte-for-byte what the
node sees when the sensor gives up on a pixel.

TWO MODES, BECAUSE THEY ARE PHYSICALLY DIFFERENT AND FAIL DIFFERENTLY. A specular
highlight or a black label is ONE CONNECTED patch (`blob`); a rough dark film or a
weak return at range is INDEPENDENT PER-PIXEL failure over the whole body
(`speckle`). The same missing AREA reaches segmentation as one hole in one body or
as a body eaten into crumbs, and `_MIN_ITEM_PX`/the h-maxima split answer those two
very differently.

THE HOLE IS PUNCHED IN THE TOP HEAD ONLY, AND THAT FLATTERS THE RIG. A glare is
view-dependent — that is the whole argument for extra heads — but matte black
packaging swallows the return from EVERY angle. Here the side heads always see a
perfect body, i.e. they work at their best case; a real rig does worse.

THREE OUTCOMES, AND THE THIRD ONE IS WHY THE PROBE EXISTS:
  * measured, inside the organizers' tolerance — the dropout was swallowed;
  * measured, OUTSIDE the tolerance — a quiet lie, the dangerous outcome;
  * NOT FOUND AT ALL — a quiet total failure: nothing crashes, the node simply says
    nothing and the item rides on unsorted.

SIDE HEADS CANNOT TOUCH THE THIRD OUTCOME, AND THAT IS BY CONTRACT, NOT BY LUCK.
`perception_node._side_clouds` crops every side cloud to `measurement.position_m`
and `measurement.dims_mm` (`src/perception_node.py:96-115`, used at `:150`), and the
only publication of a measurement lives inside `on_depth` (`:163`). No top
measurement means no crop box, so no side contribution can be computed and no
message can be published — with one head or with ten. This probe prints that in the
rig column instead of hiding it.

WHAT THE NUMBERS ARE AND ARE NOT — read before quoting them anywhere:
  * They are a property of the GEOMETRY OF MEASUREMENT: one still frame, one item
    at rest, fusion at dt = 0 (no belt-travel penalty, see probe_noise_heads).
  * They are NOT a misroute rate and DO NOT ADD UP with the census — a census cell
    is a routing verdict on a moving item (the 92 % lesson,
    `docs/report/methodology_and_limitations.md`).
  * The dropout is modelled GEOMETRICALLY, not physically: a fraction of the mask
    area, with no dependence on incidence angle, material, wavelength or range, and
    no partial/degraded returns — a pixel is either perfect or absent.

Single-head dumps are the normal case: the side heads are used only if the dump
actually carries `depth_side_*` frames (that needs Gazebo, which this probe never
starts).

    python3 scripts/probe_depth_dropout.py                      # every runs/frames/*
    python3 scripts/probe_depth_dropout.py runs/frames/bottle_oi0
    python3 scripts/probe_depth_dropout.py --trials 20 --seed 7 --fracs 0,0.4,0.8
"""
from __future__ import annotations

import argparse
import re
import sys
import zlib
from pathlib import Path
from typing import NamedTuple

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from analyze_models import analyze_file  # noqa: E402
from build_item_models import ITEMS, STL_DIR  # noqa: E402
from probe_noise_heads import RIGS, load_rig_frames  # noqa: E402
from probe_sensor_noise import slug_of_dir  # noqa: E402

from src.classification import (  # noqa: E402
    measurement_error,
    within_measurement_tolerance,
)
from src.multiview import crop_to_item, fuse_dims_mm, world_cloud_from_depth  # noqa: E402
from src.perception import (  # noqa: E402
    BELT_DEPTH_M,
    FX,
    FY,
    MASK_MARGIN_M,
    _item_mask,
    measure_items,
)

# Fractions of the ITEM MASK left without a depth return. 0 is the control row: it
# must reproduce the frame's untouched measurement exactly. The top of the range is
# deliberately absurd for a glare and realistic for black or transparent packaging,
# where the return can fail over the whole body.
DROPOUT_FRACS = (0.0, 0.1, 0.2, 0.3, 0.5, 0.7)
MODES = ("blob", "speckle")
DEFAULT_TRIALS = 5
DEFAULT_SEED = 0

# Rig dumps are named <slug>_3cam, a suffix `probe_sensor_noise.slug_of_dir` does not
# know (it predates the rig). Peeled here rather than by editing that module: the
# frozen core and the probes built on it stay untouched before the submission.
_RIG_SUFFIX_RE = re.compile(r"_\dcam$")


def slug_of_dump(dir_name):
    """Catalogue slug behind a dump dir name, rig dumps included (`bag_3cam` -> `bag`)."""
    return slug_of_dir(_RIG_SUFFIX_RE.sub("", dir_name))


def drop_item_returns(depth_m, frac, mode, rng):
    """(depth frame with `frac` of the ITEM's returns dropped, fraction actually dropped).

    The hole goes INSIDE the item mask only: a highlight sits on the product, not on
    the belt, and dropping belt pixels would test a different failure. The mask is
    the production one (`_item_mask` with the node's belt plane and margin), so the
    pixels removed here are exactly the pixels segmentation would have used.

    `blob`: the `frac * area` mask pixels NEAREST a random mask pixel — the
    intersection of a disc with the mask, i.e. one connected patch of the size asked
    for (a specular highlight, a black label). The area is exact.
    `speckle`: every mask pixel fails independently with probability `frac` (a rough
    dark film), so the realized fraction is binomial around `frac`.

    Dropped pixels are set to 0.0 — see the module docstring: that is what the node
    already feeds perception for a NaN, and both `_item_mask` and
    `world_cloud_from_depth` read it as "no data".
    """
    out = np.array(depth_m, dtype=float, copy=True)
    if frac <= 0.0:
        return out, 0.0
    ys, xs = np.nonzero(_item_mask(out, BELT_DEPTH_M, MASK_MARGIN_M))
    if not len(xs):
        return out, 0.0
    if mode == "blob":
        n_drop = min(int(round(frac * len(xs))), len(xs))
        anchor = int(rng.integers(len(xs)))
        d2 = (xs - xs[anchor]) ** 2 + (ys - ys[anchor]) ** 2
        # stable sort: ties at equal radius resolve identically on every run, so a
        # row of the report is reproducible from the seed alone.
        hit = np.argsort(d2, kind="stable")[:n_drop]
    elif mode == "speckle":
        hit = np.flatnonzero(rng.random(len(xs)) < frac)
    else:
        raise ValueError(f"unknown dropout mode {mode!r} (expected one of {MODES})")
    out[ys[hit], xs[hit]] = 0.0
    return out, len(hit) / len(xs)


class DropoutResult(NamedTuple):
    """One (item, mode, fraction) cell: the three outcomes and the side-head delta.

    `in_tol`/`out_tol`/`lost` are counted for the SHIPPING configuration — the top
    head alone — because that is the contour we submit. `rescued`/`regressed` are the
    side heads' contribution on the draws where a measurement existed at all; the
    `lost` draws cannot be rescued (module docstring: no measurement, no crop box).
    """

    in_tol: int
    out_tol: int
    lost: int
    side_errs: tuple      # worst per-side error (mm) of every measured draw
    bodies: tuple         # bodies segmented per draw: tells over-segmentation apart
    rescued: int          # top out of tolerance -> fused inside it
    regressed: int        # top inside tolerance -> fused out of it
    dropped: tuple        # realized dropout fraction of the mask, per draw


def measure_under_dropout(top_depth_m, side_depths_m, side_poses, frac, mode, rng):
    """(top-only dims, fused dims, bodies, realized fraction); dims are None if lost.

    Mirrors `perception_node.on_depth`: the top head owns detection, position and id;
    the side heads only add points inside the box the top head claimed. Unlike
    `probe_noise_heads.measure_with_rig` this returns BOTH the top-only and the fused
    dims from ONE segmentation — the rescue column compares them on the same draw.
    """
    holed, dropped = drop_item_returns(top_depth_m, frac, mode, rng)
    measurements = measure_items(holed)
    if not measurements:
        return None, None, 0, dropped
    measurement = max(measurements, key=lambda m: float(np.prod(m.dims_mm)))
    clouds = [crop_to_item(world_cloud_from_depth(depth_m, pose, FX, FY),
                           measurement.position_m, measurement.dims_mm)
              for depth_m, pose in zip(side_depths_m, side_poses)]
    fused = list(fuse_dims_mm(measurement.dims_mm, clouds, measurement.position_m))
    return list(measurement.dims_mm), fused, len(measurements), dropped


def probe_dropout(top_depth_m, side_depths_m, side_poses, truth_dims_mm,
                  frac, mode, trials, rng):
    """`DropoutResult` over `trials` draws of the dropout at one fraction and mode.

    A draw where perception finds nothing is the REAL outcome of a dropout, so it is
    counted (`lost`), never skipped — skipping it would make a heavy dropout look
    harmless, which is the exact opposite of what this probe is for.
    """
    in_tol = out_tol = lost = rescued = regressed = 0
    side_errs, bodies, dropped = [], [], []
    for _ in range(trials):
        top_dims, fused_dims, n_bodies, frac_done = measure_under_dropout(
            top_depth_m, side_depths_m, side_poses, frac, mode, rng)
        bodies.append(n_bodies)
        dropped.append(frac_done)
        if top_dims is None:
            lost += 1
            continue
        top_ok = within_measurement_tolerance(top_dims, truth_dims_mm)
        fused_ok = within_measurement_tolerance(fused_dims, truth_dims_mm)
        in_tol += top_ok
        out_tol += not top_ok
        rescued += (not top_ok) and fused_ok
        regressed += top_ok and not fused_ok
        side_errs.append(measurement_error(top_dims, truth_dims_mm)[0])
    return DropoutResult(in_tol, out_tol, lost, tuple(side_errs), tuple(bodies),
                         rescued, regressed, tuple(dropped))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("dirs", nargs="*", help="dump dirs (default: every runs/frames/*)")
    parser.add_argument("--trials", type=int, default=DEFAULT_TRIALS,
                        help="dropout draws per (frame, mode, fraction)")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED,
                        help="mandatory reproducibility seed")
    parser.add_argument("--fracs", default=None,
                        help="comma-separated mask fractions to drop, e.g. 0,0.25,0.5")
    parser.add_argument("--modes", default=",".join(MODES),
                        help=f"comma-separated dropout modes ({'/'.join(MODES)})")
    args = parser.parse_args(argv)
    fracs = [float(f) for f in args.fracs.split(",")] if args.fracs else list(DROPOUT_FRACS)
    modes = args.modes.split(",")

    dirs = [Path(d) for d in args.dirs] or sorted(
        d for d in (ROOT / "runs" / "frames").glob("*") if d.is_dir())
    if not dirs:
        print("no dump dirs — run scripts/dump_item_frame.sh first")
        return 1

    truth = {slug: tuple(float(x) for x in analyze_file(STL_DIR / f"{stem}.stl")["dims"])
             for slug, (stem, _mass) in ITEMS.items()}

    print(f"seed={args.seed} trials={args.trials} modes={','.join(modes)}  "
          "drop = fraction of the ITEM MASK with no depth return")
    print("truth = mesh OBB; tolerance = organizers' rule (5 mm/side OR 10 % volume)")
    print("outcomes are the SHIPPING config (top head alone); rig column = side-head delta\n")
    print(f"{'item / mode':<32} {'drop':>5} {'in tol':>7} {'out tol':>8} {'lost':>6} "
          f"{'side mm':>16} {'bodies':>7}  rig")

    first_loss, probed = {}, []
    lost_trials = side_rescues = side_regressions = 0
    for dump_dir in dirs:
        try:
            top, sides = load_rig_frames(dump_dir)
        except FileNotFoundError as exc:
            print(f"{dump_dir.name:<32} {exc}")
            continue
        slug = slug_of_dump(dump_dir.name)
        # Every side head the dump happens to carry. A single-head dump is the normal
        # case, not an error: the rig column then says so on every row.
        heads = [(name, pose) for name, pose in RIGS[-1][1] if name in sides]
        side_depths = [sides[name] for name, _pose in heads]
        side_poses = [pose for _name, pose in heads]
        for mode in modes:
            for frac in fracs:
                # crc32, NOT hash(): Python randomizes string hashing per process, so
                # a hash-seeded generator would make this report unreproducible.
                rng = np.random.default_rng(
                    [args.seed, zlib.crc32(f"{dump_dir.name}{mode}".encode()),
                     int(frac * 1e6)])
                res = probe_dropout(top, side_depths, side_poses, truth[slug],
                                    frac, mode, args.trials, rng)
                key = f"{dump_dir.name} / {mode}"
                if key not in probed:
                    probed.append(key)
                if res.lost and key not in first_loss:
                    first_loss[key] = frac
                lost_trials += res.lost
                side_rescues += res.rescued
                side_regressions += res.regressed
                if not heads:
                    rig = f"no side heads ({len(sides)} side frames in dump)"
                elif res.lost == args.trials:
                    rig = f"{len(heads)} side heads, NO DETECTION to attach them to"
                else:
                    rig = f"{len(heads)} side heads: +{res.rescued} rescued / -{res.regressed}"
                side_txt = (f"{np.median(res.side_errs):6.1f} ({np.max(res.side_errs):5.1f} max)"
                            if res.side_errs else "  -- lost --   ")
                print(f"{key:<32} {100 * frac:4.0f}% {res.in_tol:3d}/{args.trials:<3d} "
                      f"{res.out_tol:4d}/{args.trials:<3d} {res.lost:2d}/{args.trials:<3d} "
                      f"{side_txt:>16} {np.median(res.bodies):6.1f}  {rig}")
            print()

    print("first dropout fraction that LOST the item at least once (top head alone):")
    for key in sorted(first_loss):
        if first_loss[key] <= 0.0:
            # The control row itself found nothing, so nothing here is attributable to
            # the dropout: that dump's frame carries no measurable item to begin with
            # (an item entering/leaving the view is rejected by _find_items on purpose).
            print(f"  {key:<32} already lost at 0 % — control row empty, dropout NOT attributable")
        else:
            print(f"  {key:<32} {100 * first_loss[key]:.0f} %")
    for key in sorted(probed):
        if key not in first_loss:
            print(f"  {key:<32} never — survived every fraction probed")
    print(f"\nlost draws over the whole run: {lost_trials}; side-head rescues among them: 0")
    print("  — by contract, not by luck: src/perception_node.py:96-115 crops every side")
    print("    cloud to a measurement that, in a lost draw, does not exist.")
    print(f"side-head effect on MEASURED draws: +{side_rescues} rescued / "
          f"-{side_regressions} regressed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
