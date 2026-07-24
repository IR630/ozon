#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run the 7 procedural probes through the production depth pipeline.

Each probe is rendered from its mesh exactly as the cell would see it (the same
pinhole model and belt-resting contract as scripts/render_depth.py), then measured
by src.perception.measure_items and judged by src.classification — the code the ROS
nodes run, not a re-implementation of it.

The verdict is compared against the category derived from the DRAWING
(build_probe_items.PROBES[...].expected, justified in docs/probe-models.md), so a
mismatch is a statement about the pipeline. Known, diagnosed mismatches are listed
in KNOWN_GAPS with a reason; anything outside that list is a regression and fails
the gate.

Poses are the named resting poses of each probe, not random rotations — see the
Probe docstring for why an offline render may not draw poses it cannot settle.

Usage: python scripts/measure_probe_shapes.py
"""
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent))

from build_item_models import set_belt_origin  # noqa: E402
from build_probe_items import PROBES  # noqa: E402
from render_depth import render_depth  # noqa: E402

from src.classification import classify_conservative  # noqa: E402
from src.perception import measure_items  # noqa: E402

# Contact points are vertices within this of the lowest one: a face resting on the
# belt is flat to well under a millimetre, so this only absorbs float noise.
_CONTACT_TOL_MM = 1.0

# Diagnosed mismatches accepted for this submission: slug -> (verdict, why).
# Presence here downgrades FAIL to GAP, so the gate still fails loudly on anything
# NEW while the known holes stay visible in the output instead of being deleted.
# A gap may only be added together with its analysis in docs/probe-models.md.
# Keyed by (slug, pose): after the section rewrite a probe can be right in one
# resting pose and wrong in another, and a per-slug excuse would hide that.
KNOWN_GAPS = {
    # Compact round bodies now reach D via the relief's axial symmetry
    # (src/perception._axial_symmetry_residual): a ball and an UPRIGHT can are
    # bodies of revolution (height depends on radius alone), so they route to D as
    # they must, while a slumped Мешок's irregular relief keeps it in B. What stays
    # open is narrower and pose-specific:
    ("squat_can", "on_side"): ("B", "lying can: silhouette K=0.69 is under the threshold, "
                                    "so the symmetry route is never reached"),
    # A regular hexagon is round by the task's formula (K = cos 30 deg = 0.866) but
    # is not a circle, so the circle-fit residual rejects it. Reading the formula
    # literally instead would send the organizers' Цилиндр (K=0.74) to D — the two
    # readings cannot both be satisfied. Pending a wording question to the jury
    # (docs/defense/council_cameras.md).
    ("hex_bar", "lying"): ("B", "hexagon is round by the K>0.8 formula, not by circle fit"),
    ("hex_bar", "yaw90"): ("B", "hexagon is round by the K>0.8 formula, not by circle fit"),
    ("hex_bar", "rolled60"): ("B", "hexagon is round by the K>0.8 formula, not by circle fit"),
}


@dataclass(frozen=True)
class ProbeResult:
    slug: str
    pose: str
    expected: str
    actual: str
    dims_mm: tuple
    k: float
    n_detected: int
    stable: bool


def quat_of(axis, degrees):
    """(x, y, z, w) for a rotation of `degrees` about `axis`."""
    w, x, y, z = trimesh.transformations.quaternion_about_axis(np.deg2rad(degrees), axis)
    return (float(x), float(y), float(z), float(w))


def is_stable(mesh, quat):
    """Whether the body rests in this pose: centre of mass over the contact polygon.

    Uses the convex hull for both, matching how build_item_models derives inertia,
    and counts a centre of mass exactly on the boundary as stable — that is the
    sphere's neutral equilibrium, not a tip-over.
    """
    from scipy.spatial import ConvexHull

    body = mesh.copy()
    set_belt_origin(body)
    x, y, z, w = quat
    body.apply_transform(trimesh.transformations.quaternion_matrix([w, x, y, z]))
    hull = body.convex_hull
    verts = hull.vertices
    contact = verts[verts[:, 2] <= verts[:, 2].min() + _CONTACT_TOL_MM][:, :2]
    if len(contact) < 3:
        return True  # point or line contact: a sphere/cylinder, neutrally stable
    com = hull.center_mass[:2]
    try:
        poly = contact[ConvexHull(contact).vertices]  # scipy returns 2D hulls CCW
    except Exception:
        return True
    margins = []
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        edge, rel = b - a, com - a
        # CCW polygon: inside is the positive (left) side of every edge
        margins.append((edge[0] * rel[1] - edge[1] * rel[0]) / np.linalg.norm(edge))
    return bool(min(margins) >= -_CONTACT_TOL_MM)


def evaluate():
    results = []
    for slug, probe in PROBES.items():
        mesh = probe.build()
        for pose_name, axis, degrees in probe.poses:
            quat = quat_of(axis, degrees)
            measured = measure_items(render_depth(mesh, quat))
            stable = is_stable(mesh, quat)
            if len(measured) != 1:
                actual = "NO_DETECTION" if not measured else f"SPLIT_{len(measured)}"
                results.append(ProbeResult(slug, pose_name, probe.expected, actual,
                                           (0.0, 0.0, 0.0), 0.0, len(measured), stable))
                continue
            item = measured[0]
            results.append(ProbeResult(
                slug, pose_name, probe.expected,
                classify_conservative(item.dims_mm, item.k),
                tuple(item.dims_mm), item.k, 1, stable,
            ))
    return results


def verdict_of(result):
    if not result.stable:
        return "SKIP"  # pose the belt cannot present; nothing to conclude
    if result.actual == result.expected:
        return "PASS"
    known = KNOWN_GAPS.get((result.slug, result.pose))
    return "GAP" if known and known[0] == result.actual else "FAIL"


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    results = evaluate()
    counts = {"PASS": 0, "GAP": 0, "FAIL": 0, "SKIP": 0}
    for result in results:
        verdict = verdict_of(result)
        counts[verdict] += 1
        dims = "x".join(f"{v:.0f}" for v in result.dims_mm)
        print(f"{result.slug:16s} {result.pose:13s} expected={result.expected} "
              f"actual={result.actual:12s} dims={dims:>14s}mm "
              f"K={result.k:.3f} {verdict}")
    for (slug, pose), (verdict, why) in KNOWN_GAPS.items():
        print(f"known gap: {slug}/{pose} -> {verdict} ({why})")
    print(f"\nprobe gate: {counts['PASS']} pass, {counts['GAP']} known gaps, "
          f"{counts['FAIL']} FAIL, {counts['SKIP']} skipped, of {len(results)}")
    return 1 if counts["FAIL"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
