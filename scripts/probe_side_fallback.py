#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Can a SIDE head find the item BY ITSELF when the top head goes blind — offline.

WHY THIS EXISTS. `docs/report/cameras.md` §8 T3 proves from three places in the
node that the shipped rig has no redundancy at ANY head count: `on_side_depth`
only parks a frame (`src/perception_node.py:80-94`), `_side_clouds` crops each
side cloud to a `measurement` that a blinded top head never produced (`:96-115`),
and the cell's single `publish` sits inside the loop over TOP detections (`:163`).
`probe_depth_dropout.py:314` prints "side-head rescues: 0" as a statement about
that contract, not as a measurement. So comparing a 2-head rig against a 3-head
rig today compares two ways of decorating a one-camera system.

This probe measures the missing precondition: give a side head its own detector
and ask whether it finds the item at all. It never starts Gazebo — it punches the
same depth-return dropout `probe_depth_dropout.py` uses into the TOP frame of a
live `*_3cam` dump and runs the side frames through their own segmentation.

THE DETECTOR, AND WHY EVERY PIECE OF IT IS THE SHAPE IT IS. The design pass of
25.07 (`docs/plan-line-readiness.md`, "Что закрыто проектным прогоном") settled
four questions with numbers, and this is the script that makes them reproducible:

  * SEGMENT IN THE HEAD'S RASTER, not in world XY. A helmet dome breaks into two
    world-XY clusters (near and far rim, an invisible cavity between them) and
    becomes two phantom items; in the raster it is one connected component.
  * THE PRIMITIVE IS THE ITEM PRISM, not "closer than the belt". A side head
    grazes the belt plane, so "depth less than the belt's depth minus a margin"
    degenerates at the horizon: the ray runs parallel to the plane and 8 mm of
    height becomes metres of depth. The prism — belt strip, capped in height —
    has no such singularity.
  * THE THRESHOLDS CARRY OVER UNCHANGED. `SIDE_BELT_MARGIN_M` = 8 mm is already
    derived for a grazing head, and the design pass measured the belt
    reconstructing 0.67 mm above itself with the [2, 8) mm band holding 0-43
    points out of ~200 000 — an empty gap between belt and goods. `_MIN_ITEM_PX`
    = 24 carries over too: bare belt gives 0 px and the pen gives 113-164.
  * BORDER COMPONENTS ARE REJECTED, and this is not decoration. The top head
    deliberately drops an item touching the frame border (`src/perception.py:348`)
    because it is riding into or out of view. In a live run that is probably a
    MORE common reason for "no top detection" than any dropout, so a fallback
    without the same rule would "rescue" exactly the garbage the top head threw
    away on purpose.

`_body_obb_dims_mm` is NOT called here. Two independent reasons: it dominates the
cost (13.6-72.1 ms against 10-17 ms without it), and `cameras.md` §8 T4 proved
the relief gate that is supposed to guard it stands open on every grazing cloud
(0.86 against 0.0). Dims come from the production shadow box only — the same
primitive `src.multiview.fuse_dims_mm` builds at its lines 128-132.

K IS PUBLISHED AS 0.0, AND THE USUAL SAFETY ARGUMENT DOES NOT APPLY HERE. A side
head sees the end circle of anything lying down and would route every prone body
to D, so it must not claim K. Of the available values only 0.0 fails to invent a
verdict D. The plan argued this is free because `ItemAggregator` takes the MEDIAN
K over an item's frames — but that holds only while fallback frames are a
minority. Black film, a specular wrap and transparent packaging blind the top
head for the WHOLE passage, which is the case this feature exists for, and then
every frame is a fallback frame and the median is 0.0. So: a side fallback can
restore DIMENSIONS and can never restore the D verdict. The verdict-class table
below counts that, instead of leaving it as prose.

WHAT THE NUMBERS ARE AND ARE NOT — read before quoting them anywhere:
  * a property of the GEOMETRY OF MEASUREMENT on still frames, one item at rest,
    heads fused at dt = 0 (no sync penalty — that flatters every rig above one);
  * NOT a misroute rate, and they DO NOT ADD UP with the census (the 92 % lesson);
  * the dropout is injected into the TOP head only, so the side heads work at
    their best case, exactly as in `probe_depth_dropout.py`;
  * three items of eleven, one resting pose each — enough to reject "won't fly",
    not enough to calibrate a threshold.

KNOWN LIMITS OF THE DETECTOR ITSELF, named rather than discovered later:
  * no `_split_touching`. For a side head two items metres apart along the belt
    project onto the SAME region — a downstream body hides the one behind it —
    so splitting them needs association across heads, which is arbitration, which
    is not decided yet. One item per frame is what the dumps carry;
  * the XY hull of one flank underestimates the across-belt dimension of a FLAT
    face: a side head sees one plane of a box and reads its depth as ~0. Rounded
    bodies (bag, helmet) do not show this; the dumps contain no box, so this
    script cannot measure it. It is the reason the two-opposing-flanks column
    exists.

    python3 scripts/probe_side_fallback.py
    python3 scripts/probe_side_fallback.py runs/frames/helmet_3cam --trials 20
"""
from __future__ import annotations

import argparse
import sys
import time
import zlib
from pathlib import Path
from typing import NamedTuple

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(ROOT))

from analyze_models import analyze_file  # noqa: E402
from build_item_models import ITEMS, STL_DIR  # noqa: E402
from probe_depth_dropout import drop_item_returns, slug_of_dump  # noqa: E402
from probe_noise_heads import RIGS, load_rig_frames  # noqa: E402

from src.classification import (  # noqa: E402
    classify_conservative,
    measurement_error,
    within_measurement_tolerance,
)
from src.constants import (  # noqa: E402
    BELT_HALF_WIDTH_M,
    CATEGORY_B,
    CATEGORY_C,
    CATEGORY_D,
    MAX_DIMS_MM,
    SIDE_BELT_MARGIN_M,
)
from src.multiview import world_cloud_from_depth  # noqa: E402
from src.perception import (  # noqa: E402
    BELT_DEPTH_M,
    BELT_TOP_Z_M,
    CAMERA_X_M,
    FX,
    FY,
    IMG_H,
    IMG_W,
    _MIN_ITEM_PX,
    _obb_dims_px,
    measure_items,
)

# Same range gate `world_cloud_from_depth` applies. Stated explicitly because this
# module pairs its output with raster indices and the two filters must agree
# pixel for pixel (locked by test_the_world_cloud_pairs_with_its_raster_pixels).
MAX_RANGE_M = 5.0

# Ceiling of the item prism: nothing taller than the sorter's largest admissible
# dimension is goods — above that is the gantry, a roll cage or the opposite
# head's housing (photographed in docs/report/img/head_views_3cam.png).
ITEM_CEILING_M = max(MAX_DIMS_MM) / 1000.0

# Along-belt bound: the window the TOP head could have worked in, so the fallback
# cannot claim an item the cell never had a chance to route. Derived, not chosen —
# the image's vertical axis maps to world x for a straight-down head, so the
# half-extent at belt range is (IMG_H / 2) / FY * BELT_DEPTH_M.
TOP_VIEW_HALF_X_M = (IMG_H / 2.0) / FY * BELT_DEPTH_M
# ... and the image's horizontal axis maps to world y. Kept for the report: at
# 0.87 m it is WIDER than the belt, which is why the lateral bound below is the
# belt edge and not the top head's cone.
TOP_VIEW_HALF_Y_M = (IMG_W / 2.0) / FX * BELT_DEPTH_M

DROPOUT_FRACS = (0.0, 0.5, 0.9, 0.99)
MODES = ("blob", "speckle")
DEFAULT_TRIALS = 10
DEFAULT_SEED = 0

# Verdict classes, ordered by what an error COSTS on the line rather than by how
# often it happens. This taxonomy is the deliverable of the cost-of-error section
# (docs/report/rig_decision.md) and is defined here, before the runs, so the runs
# cannot be graded against a scale invented after seeing them.
CLASS_OK = "верно"          # measured verdict == reference verdict
CLASS_TO_C = "→C"           # to manual handling: item survives, we pay human minutes
CLASS_TO_D = "→D"           # to re-packing: also a human step, also recoverable
CLASS_TO_B = "→B"           # to the main sorter: a jam, a stopped line, a manual dig
CLASS_NONE = "отказ"        # nothing published at all
CLASS_INSANE = "невалид"    # dims outside the physical sanity range: loud, not silent
COST_ORDER = (CLASS_TO_B, CLASS_INSANE, CLASS_NONE, CLASS_TO_D, CLASS_TO_C, CLASS_OK)


class SideDetection(NamedTuple):
    """One body a single side head found on its own."""

    dims_mm: list        # three dims, mm, sorted descending
    n_pixels: int        # size of the raster component it came from
    points_m: np.ndarray  # its world points, for fusing two flanks


def side_world_points(depth_m, pose, fx=FX, fy=FY, cx=None, cy=None):
    """(world points (N, 3) m, pixel rows, pixel cols) of every valid pixel.

    The points come from production `world_cloud_from_depth` — this function adds
    nothing to the geometry, it only keeps the raster coordinates the production
    call throws away, because segmentation has to happen in the raster.
    """
    depth_m = np.asarray(depth_m, dtype=np.float64)
    valid = np.isfinite(depth_m) & (depth_m > 0.0) & (depth_m < MAX_RANGE_M)
    vs, us = np.nonzero(valid)
    pts = world_cloud_from_depth(depth_m, pose, fx, fy, cx, cy, max_range_m=MAX_RANGE_M)
    return pts, vs, us


def item_prism_mask(depth_m, pose, fx=FX, fy=FY, cx=None, cy=None):
    """(raster mask of goods pixels, world points, raster->point index).

    A pixel is goods if its world point stands in the prism over the belt:
    between SIDE_BELT_MARGIN_M and the sorter's tallest admissible item, inside
    the belt edges, and inside the along-belt window the top head works in.
    """
    pts, vs, us = side_world_points(depth_m, pose, fx, fy, cx, cy)
    mask = np.zeros(depth_m.shape, dtype=bool)
    index = np.full(depth_m.shape, -1, dtype=np.int64)
    if not len(pts):
        return mask, pts, index
    index[vs, us] = np.arange(len(pts))
    height_m = pts[:, 2] - BELT_TOP_Z_M
    inside = ((height_m >= SIDE_BELT_MARGIN_M)
              & (height_m <= ITEM_CEILING_M)
              & (np.abs(pts[:, 1]) <= BELT_HALF_WIDTH_M)
              & (np.abs(pts[:, 0] - CAMERA_X_M) <= TOP_VIEW_HALF_X_M))
    mask[vs[inside], us[inside]] = True
    return mask, pts, index


def side_dims_mm(points_m):
    """Dims (mm, desc) of one side cloud: the production shadow box, no body-OBB.

    Same primitive as `src.multiview.fuse_dims_mm` lines 128-132 — convex hull of
    the belt-plane footprint, oriented box on it, height as the cloud's rise above
    the belt — with the body-OBB branch deliberately absent (module docstring).
    """
    from scipy.spatial import ConvexHull, QhullError

    xy_mm = points_m[:, :2] * 1000.0
    try:
        hull = ConvexHull(xy_mm)
    except QhullError:
        return None
    long_mm, short_mm, _dir = _obb_dims_px(xy_mm[hull.vertices])
    dz_mm = float(points_m[:, 2].max() - BELT_TOP_Z_M) * 1000.0
    return sorted([float(long_mm), float(short_mm), dz_mm], reverse=True)


def find_side_items(depth_m, pose, fx=FX, fy=FY, cx=None, cy=None):
    """Every body this side head can find on its own, largest component first.

    The three rejections are the top head's own, transplanted (`_find_items`,
    `src/perception.py:317-352`): a component under `_MIN_ITEM_PX` is noise, and a
    component touching the frame border is an item riding into or out of view and
    yields garbage dims. Without the border rule the fallback would publish
    exactly what the top head discards on purpose.
    """
    from scipy.ndimage import label

    mask, pts, index = item_prism_mask(depth_m, pose, fx, fy, cx, cy)
    labels, count = label(mask, structure=np.ones((3, 3), dtype=int))
    h, w = depth_m.shape
    found = []
    for component_id in range(1, count + 1):
        component = labels == component_id
        n_px = int(component.sum())
        if n_px < _MIN_ITEM_PX:
            continue
        ys, xs = np.nonzero(component)
        if xs.min() == 0 or ys.min() == 0 or xs.max() == w - 1 or ys.max() == h - 1:
            continue
        dims = side_dims_mm(pts[index[component]])
        if dims is None:
            continue
        found.append(SideDetection(dims, n_px, pts[index[component]]))
    return sorted(found, key=lambda d: d.n_pixels, reverse=True)


def fuse_flanks(detections):
    """Dims (mm, desc) from several heads' clouds of the SAME item, or None.

    Provisional association rule, and it is named as provisional: with one item in
    the frame every head's largest component is that item. Two items would need a
    real arbitration rule, which this session has not decided.

    This is the one place where a THIRD head can do something a second head cannot
    under an independent detector: a single flank's footprint collapses the
    across-belt dimension of a flat face, and two opposing flanks restore it.
    """
    clouds = [d.points_m for d in detections if d is not None]
    if not clouds:
        return None
    return side_dims_mm(np.vstack(clouds))


def verdict_class(dims_mm, k, reference_category):
    """Which cost class this measurement's routing falls in (CLASS_* above)."""
    if dims_mm is None:
        return CLASS_NONE
    try:
        category = classify_conservative(dims_mm, k)
    except ValueError:
        return CLASS_INSANE
    if category == reference_category:
        return CLASS_OK
    return {CATEGORY_B: CLASS_TO_B, CATEGORY_C: CLASS_TO_C, CATEGORY_D: CLASS_TO_D}[category]


class Cell(NamedTuple):
    """One (dump, mode, fraction) cell of the report."""

    top_found: int          # draws where the top head produced a measurement
    top_in_tol: int         # ... and it was inside the organizers' tolerance
    lost: int               # draws where the top head found nothing
    rescued: int            # lost draws where a side head found the item
    rescued_in_tol: int     # ... and its dims were inside tolerance
    contradicted: int       # top measured but OUT of tolerance, side inside it
    silent_lies: int        # top measured, out of tolerance, top head confident
    side_classes: dict      # verdict class -> count, over the side path
    side_errs: tuple        # per-side error (mm) of every side measurement
    ms: tuple               # wall time (ms) of the side path per draw


def probe_cell(top_depth_m, side_frames, truth_dims_mm, reference_category,
               frac, mode, trials, rng):
    """One report cell: `trials` draws of the dropout, both paths measured on each.

    Both paths run on EVERY draw, not only on the lost ones. The fallback the plan
    designs fires only when the top head found nothing, but the dropout probe
    already measured that a big body almost never disappears — it keeps a live
    detection and lies by 114-265 mm. A detector that only fires on absence can
    never contradict that lie, so the contradiction column measures what an
    always-on side detector would have been worth.
    """
    top_found = top_in_tol = lost = rescued = rescued_in_tol = 0
    contradicted = silent_lies = 0
    classes, side_errs, ms = {}, [], []
    for _ in range(trials):
        holed, _dropped = drop_item_returns(top_depth_m, frac, mode, rng)
        measurements = measure_items(holed)
        top_dims = None
        if measurements:
            top = max(measurements, key=lambda m: float(np.prod(m.dims_mm)))
            top_dims = list(top.dims_mm)

        started = time.perf_counter()
        per_head = [find_side_items(depth_m, pose) for depth_m, pose in side_frames]
        side_dims = fuse_flanks([heads[0] for heads in per_head if heads])
        ms.append((time.perf_counter() - started) * 1000.0)

        # k = 0.0: the side path owns no evidence of roundness (module docstring).
        klass = verdict_class(side_dims, 0.0, reference_category)
        classes[klass] = classes.get(klass, 0) + 1
        side_ok = side_dims is not None and within_measurement_tolerance(
            side_dims, truth_dims_mm)
        if side_dims is not None:
            side_errs.append(measurement_error(side_dims, truth_dims_mm)[0])

        if top_dims is None:
            lost += 1
            rescued += side_dims is not None
            rescued_in_tol += side_ok
            continue
        top_found += 1
        top_ok = within_measurement_tolerance(top_dims, truth_dims_mm)
        top_in_tol += top_ok
        silent_lies += not top_ok
        contradicted += (not top_ok) and side_ok
    return Cell(top_found, top_in_tol, lost, rescued, rescued_in_tol, contradicted,
                silent_lies, classes, tuple(side_errs), tuple(ms))


def phantoms_on_a_belt_without_goods(side_frames):
    """[(head, phantoms found once the goods' own returns are erased)].

    The empty-belt false-positive test, built from the same injection tool: every
    pixel the detector claimed as goods is set to 0 — the byte-exact way a sensor
    reports "no return" — and the detector is run again. Anything it finds now is
    a body invented on a belt whose only item gives nothing back. The erased
    region cannot itself become a phantom: zero depth is not a valid pixel.
    """
    out = []
    for name, depth_m, pose in side_frames:
        blanked = np.array(depth_m, dtype=np.float64, copy=True)
        # Erase the WHOLE prism, not just the components that survived the size and
        # border filters: a fair empty-belt frame must carry no goods returns at
        # all, including the ones the detector rejected.
        mask, _pts, _index = item_prism_mask(blanked, pose)
        blanked[mask] = 0.0
        out.append((name, find_side_items(blanked, pose)))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("dirs", nargs="*", help="dump dirs (default: every runs/frames/*_3cam)")
    parser.add_argument("--trials", type=int, default=DEFAULT_TRIALS,
                        help="dropout draws per (frame, mode, fraction)")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED,
                        help="mandatory reproducibility seed")
    parser.add_argument("--fracs", default=None,
                        help="comma-separated mask fractions to drop, e.g. 0,0.9,0.99")
    parser.add_argument("--modes", default=",".join(MODES),
                        help=f"comma-separated dropout modes ({'/'.join(MODES)})")
    args = parser.parse_args(argv)
    fracs = [float(f) for f in args.fracs.split(",")] if args.fracs else list(DROPOUT_FRACS)
    modes = args.modes.split(",")

    dirs = [Path(d) for d in args.dirs] or sorted(
        d for d in (ROOT / "runs" / "frames").glob("*_3cam") if d.is_dir())
    if not dirs:
        print("no rig dumps — run runs/g_dump_3cam.sh first (needs Gazebo)")
        return 1

    from pose_sweep import REFERENCE  # heavy import, and only main() needs it

    truth = {slug: tuple(float(x) for x in analyze_file(STL_DIR / f"{stem}.stl")["dims"])
             for slug, (stem, _mass) in ITEMS.items()}

    print(f"seed={args.seed} trials={args.trials} modes={','.join(modes)}  "
          "drop = fraction of the TOP head's ITEM MASK with no depth return")
    print("truth = mesh OBB; tolerance = organizers' rule (5 mm/side OR 10 % volume)")
    print("side path = INDEPENDENT detection per head, prism in the head's raster, k=0.0")
    print("fusion at dt=0 — sync penalty NOT modelled, every head above one is optimistic\n")
    print(f"{'item / mode':<28} {'drop':>5} {'top ok':>8} {'lost':>7} {'rescued':>9} "
          f"{'contra':>7} {'side mm':>16} {'ms':>6}  verdicts")

    totals = {"lost": 0, "rescued": 0, "rescued_in_tol": 0,
              "lies": 0, "contradicted": 0}
    classes_all = {}
    for dump_dir in dirs:
        try:
            top, sides = load_rig_frames(dump_dir)
        except FileNotFoundError as exc:
            print(f"{dump_dir.name:<28} {exc}")
            continue
        heads = [(name, pose) for name, pose in RIGS[-1][1] if name in sides]
        if not heads:
            print(f"{dump_dir.name:<28} no depth_side_* frames — not a rig dump, skipped")
            continue
        slug = slug_of_dump(dump_dir.name)
        side_frames = [(sides[name], pose) for name, pose in heads]
        for mode in modes:
            for frac in fracs:
                # crc32, NOT hash(): Python randomizes string hashing per process,
                # so a hash-seeded generator would make this report unreproducible.
                rng = np.random.default_rng(
                    [args.seed, zlib.crc32(f"{dump_dir.name}{mode}".encode()),
                     int(frac * 1e6)])
                cell = probe_cell(top, side_frames, truth[slug], REFERENCE[slug],
                                  frac, mode, args.trials, rng)
                totals["lost"] += cell.lost
                totals["rescued"] += cell.rescued
                totals["rescued_in_tol"] += cell.rescued_in_tol
                totals["lies"] += cell.silent_lies
                totals["contradicted"] += cell.contradicted
                for name, n in cell.side_classes.items():
                    classes_all[name] = classes_all.get(name, 0) + n
                verdicts = " ".join(f"{name}={cell.side_classes[name]}"
                                    for name in COST_ORDER if name in cell.side_classes)
                side_txt = (f"{np.median(cell.side_errs):6.1f} "
                            f"({np.max(cell.side_errs):5.1f} max)"
                            if cell.side_errs else "  -- lost --   ")
                print(f"{dump_dir.name + ' / ' + mode:<28} {100 * frac:4.0f}% "
                      f"{cell.top_in_tol:3d}/{cell.top_found:<4d} "
                      f"{cell.lost:3d}/{args.trials:<3d} "
                      f"{cell.rescued_in_tol:3d}/{cell.rescued:<5d} "
                      f"{cell.contradicted:3d}/{cell.silent_lies:<3d} {side_txt:>16} "
                      f"{np.median(cell.ms):5.1f}  {verdicts}")
            print()

    print("ONE FLANK AGAINST TWO — the only mechanism by which a THIRD head can beat")
    print("a second one under independent detection: a single flank's footprint has no")
    print("across-belt extent, and under the SHIPPED fusion this cannot show up at all.")
    print(f"{'item / heads':<40} {'px':>7} {'dims mm':>26} {'err mm':>7} {'ms':>6}")
    for dump_dir in dirs:
        try:
            _top, sides = load_rig_frames(dump_dir)
        except FileNotFoundError:
            continue
        heads = [(name, pose) for name, pose in RIGS[-1][1] if name in sides]
        if not heads:
            continue
        truth_dims = truth[slug_of_dump(dump_dir.name)]
        best = []
        for name, pose in heads:
            started = time.perf_counter()
            found = find_side_items(sides[name], pose)
            ms = (time.perf_counter() - started) * 1000.0
            if not found:
                print(f"{dump_dir.name + ' / ' + name:<40} {'-':>7} {'lost':>26} "
                      f"{'-':>7} {ms:5.1f}")
                continue
            best.append(found[0])
            err = measurement_error(found[0].dims_mm, truth_dims)[0]
            dims_txt = "x".join(f"{d:.0f}" for d in found[0].dims_mm)
            print(f"{dump_dir.name + ' / ' + name:<40} {found[0].n_pixels:7d} "
                  f"{dims_txt:>26} {err:7.1f} {ms:5.1f}")
        fused = fuse_flanks(best)
        if fused is not None:
            err = measurement_error(fused, truth_dims)[0]
            dims_txt = "x".join(f"{d:.0f}" for d in fused)
            truth_txt = "x".join(f"{d:.0f}" for d in truth_dims)
            print(f"{dump_dir.name + ' / ALL FLANKS':<40} {'':>7} {dims_txt:>26} "
                  f"{err:7.1f}   (truth {truth_txt})")
    print()

    print("EMPTY BELT: goods' own returns erased, detector re-run — anything found is invented")
    for dump_dir in dirs:
        try:
            _top, sides = load_rig_frames(dump_dir)
        except FileNotFoundError:
            continue
        named = [(name, sides[name], pose) for name, pose in RIGS[-1][1] if name in sides]
        for name, phantoms in phantoms_on_a_belt_without_goods(named):
            sizes = ", ".join(str(p.n_pixels) for p in phantoms) or "-"
            print(f"  {dump_dir.name + ' / ' + name:<40} {len(phantoms)} phantom(s)  px: {sizes}")

    print("\nGATE (declared in docs/plan-line-readiness.md BEFORE this probe was written):")
    lost, rescued = totals["lost"], totals["rescued"]
    frac_back = rescued / lost if lost else float("nan")
    frac_ok = totals["rescued_in_tol"] / rescued if rescued else float("nan")
    print(f"  measurement returned on {rescued}/{lost} lost draws = {100 * frac_back:.0f} % "
          f"(gate: >= 60 %)")
    print(f"  of those, inside tolerance {totals['rescued_in_tol']}/{rescued} = "
          f"{100 * frac_ok:.0f} % (gate: >= 70 %)")
    print("  baseline today is 0 % — probe_depth_dropout.py:314, by contract")
    print("  DENOMINATOR CAVEAT: the dropout is injected into the TOP head only, so the")
    print(f"  side path is deterministic per dump. These {lost} draws carry {len(dirs)} distinct")
    print("  side measurements, not independent samples — read the flank table above.\n")
    print("ALWAYS-ON value (not part of the gate — the fallback design cannot collect it):")
    print(f"  top head measured but LIED (out of tolerance): {totals['lies']} draws; "
          f"an always-on side detector was inside tolerance on {totals['contradicted']}")
    print("\nverdict classes over the side path, costliest first:")
    for name in COST_ORDER:
        if name in classes_all:
            print(f"  {name:<8} {classes_all[name]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
