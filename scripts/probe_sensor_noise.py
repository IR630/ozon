#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""How much depth noise does our MEASUREMENT survive — offline, on dumped frames.

WHY THIS EXISTS. `sim/worlds/*.sdf` contains no noise model at all: every number
this project reports was measured on perfect depth. A D435-class sensor is quoted
at 1-2 % of range, so the honest question "at what sigma does the measurement stop
meeting the organizers' tolerance (5 mm per side OR 10 % by volume, the looser one
governs)" cannot be answered by our stand at all. This probe answers it the only
way available offline: take the frames `scripts/dump_item_frame.sh` already dumped,
add Gaussian noise proportional to range, and push the result through the SAME
`src.perception` the ROS node runs.

WHAT THE NUMBER IS AND IS NOT — read before quoting it anywhere:

  * It is a property of the GEOMETRY OF MEASUREMENT: one still frame, one head,
    one item at rest under the camera.
  * It is NOT a property of the CONTOUR. This probe does not model the belt, the
    crop, view fusion, or the tracker. The 92 % lesson in
    `docs/report/methodology_and_limitations.md` is exactly this trap: an offline
    probe reported a number the contour then did not reproduce.
  * It therefore DOES NOT ADD UP with the census. A census cell is a routing
    verdict on a moving item; a row here is a measurement error on a still one.
  * Head configurations are NOT covered BY THIS PROBE: it noises and measures the
    top frame only. Rig dumps do exist under `runs/frames/*_3cam` (they carry
    `depth_side_*` frames); the one-vs-two-vs-three-head delta under noise is
    measured by `scripts/probe_noise_heads.py`, which reads them, not here.

Ground truth is the mesh OBB (`analyze_models.analyze_file`), the same truth
`scripts/census_tolerance.py` uses, so the two are at least measuring error
against the same reference.

    python3 scripts/probe_sensor_noise.py                    # every runs/frames/* dir
    python3 scripts/probe_sensor_noise.py runs/frames/bottle_oi0
    python3 scripts/probe_sensor_noise.py --trials 20 --seed 7
"""
from __future__ import annotations

import argparse
import re
import sys
import zlib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from analyze_models import analyze_file  # noqa: E402
from build_item_models import ITEMS, STL_DIR  # noqa: E402

from src.classification import (  # noqa: E402
    measurement_error,
    within_measurement_tolerance,
)
from src.perception import load_depth_png, measure_items  # noqa: E402

# Fractions of RANGE, not absolute metres: a depth sensor's error grows with
# distance, and the D435 datasheet quotes it that way. 0 is the control row —
# it must reproduce the frame's noise-free measurement exactly.
SIGMAS_FRAC = (0.0, 0.005, 0.01, 0.02)
DEFAULT_TRIALS = 10
DEFAULT_SEED = 0

# Dump dirs are named <slug>[_oi<N>][_dyn][_node][_<N>cam] in any order the caller
# happened to use, so suffixes are peeled one at a time rather than matched in a
# fixed order. `_<N>cam` marks a rig dump and postdates the rest: without it this
# module raised on exactly the three dirs that carry side frames, i.e. the only
# ones the head-count comparison can use.
_DIR_SUFFIX_RE = re.compile(r"(_node|_dyn|_oi\d+|_\d+cam)$")


def add_depth_noise(depth_m, sigma_frac, rng):
    """Depth frame with Gaussian noise of sigma = sigma_frac * range (metres).

    Pixels with no return stay at 0: a sensor that saw nothing does not start
    seeing noise, and `src.perception` reads 0 as "no data". Negative draws are
    clipped to 0 for the same reason — a depth behind the lens is not a reading.
    """
    depth_m = np.asarray(depth_m, dtype=float)
    noisy = depth_m + rng.normal(0.0, sigma_frac * depth_m)
    return np.where(depth_m > 0.0, np.clip(noisy, 0.0, None), 0.0)


def slug_of_dir(dir_name):
    """Item slug behind a dump directory name (`bag_oi1_node` -> `bag`).

    Raises on anything not in the catalogue: silently probing a directory whose
    item we cannot name would produce error numbers against the wrong truth.
    """
    slug = dir_name
    while slug not in ITEMS and _DIR_SUFFIX_RE.search(slug):
        slug = _DIR_SUFFIX_RE.sub("", slug)
    if slug not in ITEMS:
        raise ValueError(f"cannot map dump dir {dir_name!r} to a catalogue item (got {slug!r})")
    return slug


def _largest(measurements):
    """The measurement of the biggest body in the frame, or None if nothing was found."""
    if not measurements:
        return None
    return max(measurements, key=lambda m: float(np.prod(m.dims_mm)))


def probe_frame(depth_m, truth_dims_mm, sigma_frac, trials, rng, bodies=None):
    """(detected, side errors mm, volume errors, tolerance passes) over `trials` draws.

    A draw where perception finds nothing is a REAL outcome of noise (the item
    stops separating from the belt), so it is counted as a lost detection rather
    than skipped — dropping it would make high sigma look better than it is.

    `bodies` (an optional list) collects how many separate bodies each draw
    produced. That count is what tells an ACCURACY collapse apart from a
    SEGMENTATION collapse: one body with a growing error is the sensor losing
    precision, several bodies per frame is the belt itself crossing the 5 mm
    segmentation margin and being measured as product.
    """
    side_errs, vol_errs, passes, detected = [], [], 0, 0
    for _ in range(trials):
        measurements = measure_items(add_depth_noise(depth_m, sigma_frac, rng))
        if bodies is not None:
            bodies.append(len(measurements))
        measurement = _largest(measurements)
        if measurement is None:
            continue
        detected += 1
        side, vol = measurement_error(measurement.dims_mm, truth_dims_mm)
        side_errs.append(side)
        vol_errs.append(vol)
        passes += within_measurement_tolerance(measurement.dims_mm, truth_dims_mm)
    return detected, side_errs, vol_errs, passes


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("dirs", nargs="*", help="dump dirs (default: every runs/frames/*)")
    parser.add_argument("--trials", type=int, default=DEFAULT_TRIALS,
                        help="noise draws per (frame, sigma)")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED,
                        help="mandatory reproducibility seed")
    parser.add_argument("--frames", type=int, default=1,
                        help="depth frames per dir (the first N)")
    parser.add_argument("--sigmas", default=None,
                        help="comma-separated fractions of range, e.g. 0,0.001,0.003")
    args = parser.parse_args(argv)
    sigmas = ([float(s) for s in args.sigmas.split(",")] if args.sigmas else list(SIGMAS_FRAC))

    dirs = [Path(d) for d in args.dirs] or sorted(
        d for d in (ROOT / "runs" / "frames").glob("*") if d.is_dir())
    if not dirs:
        print("no dump dirs — run scripts/dump_item_frame.sh first")
        return 1

    truth = {slug: tuple(float(x) for x in analyze_file(STL_DIR / f"{stem}.stl")["dims"])
             for slug, (stem, _mass) in ITEMS.items()}

    print(f"seed={args.seed} trials={args.trials} frames/dir={args.frames}  "
          f"sigma = fraction of RANGE (D435-class datasheet: 1-2 %)")
    print("truth = mesh OBB; tolerance = organizers' rule (5 mm/side OR 10 % volume)\n")
    print(f"{'item':<22} {'sigma':>6} {'found':>7} {'side mm':>16} {'vol %':>8} "
          f"{'in tol':>8} {'bodies':>7}")

    worst_ok_sigma = {}
    for dump_dir in dirs:
        frames = sorted(dump_dir.glob("depth_*.png"))[:args.frames]
        if not frames:
            print(f"{dump_dir.name:<22} no depth_*.png")
            continue
        slug = slug_of_dir(dump_dir.name)
        depths = [load_depth_png(str(f)) for f in frames]
        for sigma in sigmas:
            # One generator per (dir, sigma) seeded from the run seed: the rows of a
            # report stay reproducible even if a dir is probed on its own.
            # crc32, NOT hash(): Python randomizes string hashing per process, so a
            # hash-seeded generator would make this report unreproducible.
            rng = np.random.default_rng(
                [args.seed, zlib.crc32(dump_dir.name.encode()), int(sigma * 1e6)])
            detected = passes = 0
            side_errs, vol_errs, bodies = [], [], []
            for depth_m in depths:
                d, s, v, p = probe_frame(depth_m, truth[slug], sigma, args.trials, rng,
                                         bodies=bodies)
                detected += d
                passes += p
                side_errs += s
                vol_errs += v
            total = args.trials * len(depths)
            side_txt = (f"{np.median(side_errs):6.1f} ({np.max(side_errs):5.1f} max)"
                        if side_errs else "  -- lost --   ")
            vol_txt = f"{100 * np.median(vol_errs):7.1f}" if vol_errs else "      --"
            print(f"{dump_dir.name:<22} {100 * sigma:5.2f}% {detected:3d}/{total:<3d} "
                  f"{side_txt:>16} {vol_txt:>8} {passes:4d}/{total:<3d} "
                  f"{np.median(bodies):6.1f}")
            if passes == total:
                worst_ok_sigma[dump_dir.name] = sigma
        print()

    print("last sigma where EVERY draw stayed inside the tolerance:")
    for name in sorted(d.name for d in dirs):
        ok = worst_ok_sigma.get(name)
        print(f"  {name:<22} {'none — already out at sigma=0' if ok is None else f'{100 * ok:.2f} %'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
