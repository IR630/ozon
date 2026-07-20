# -*- coding: utf-8 -*-
"""Build 7 procedural edge-case items that the organizers' 11 do not cover.

WHY. Every empirical gate in src/perception.py (tau band, section rms, elongation,
flatness, mask margin) is a line drawn between two of those 11 models. Eleven points
are enough to draw the line and not enough to learn who it cuts wrongly. These shapes
are chosen so that each one attacks a DIFFERENT gate, and because they are generated
from numbers rather than loaded from CAD, their true category is known analytically —
so a disagreement with the pipeline is evidence about the pipeline, not about a mesh
we had to measure first. Design and expected verdicts: docs/probe-models.md.

Output tree (generated, gitignored): sim/models/probe_items/<slug>/ — a separate root
from the released set, so existing gates over sim/models/items keep their exact
contents. Gazebo scripts take it via the env var they already read:
    ITEM_MODEL_ROOT=sim/models/probe_items ./scripts/run_stream.sh

Run: python scripts/build_probe_items.py
"""
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))

from build_item_models import write_model  # noqa: E402  (one writer for both sets)

OUT_DIR = ROOT / "sim" / "models" / "probe_items"

# Facets on curved surfaces. High enough that faceting does not eat into the
# roundness measurement (a 64-gon has K = cos(pi/64) = 0.9988, i.e. round to
# within 0.2% — well clear of the 0.8 threshold under test).
_SEGMENTS = 64


def _convex_prism(poly_xy, height_mm):
    """Prism of `height_mm` (along Z, based at z=0) over a CONVEX polygon.

    Convexity is what lets the caps be a simple triangle fan; the only concave
    shape in this set (u_bracket) is assembled from boxes instead.
    """
    poly = np.asarray(poly_xy, dtype=float)
    n = len(poly)
    verts = np.vstack([
        np.column_stack([poly, np.zeros(n)]),
        np.column_stack([poly, np.full(n, height_mm)]),
    ])
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces += [[i, j, j + n], [i, j + n, i + n]]      # side wall
    for i in range(1, n - 1):
        faces += [[0, i + 1, i], [n, n + i, n + i + 1]]  # bottom / top fan
    mesh = trimesh.Trimesh(vertices=verts, faces=np.asarray(faces))
    mesh.fix_normals()
    return mesh


def _annulus(r_outer_mm, r_inner_mm, height_mm, segments=_SEGMENTS):
    """Flat ring: a hole through the middle, so the belt is VISIBLE through the item.

    Built explicitly rather than by boolean subtraction: neither manifold3d nor
    shapely is installed (and neither may be added — the solution has to deploy in
    the organizers' environment on the allowed software list).
    """
    ang = np.linspace(0.0, 2.0 * np.pi, segments, endpoint=False)
    circle = np.column_stack([np.cos(ang), np.sin(ang)])
    rings = [
        np.column_stack([circle * r_outer_mm, np.zeros(segments)]),
        np.column_stack([circle * r_outer_mm, np.full(segments, height_mm)]),
        np.column_stack([circle * r_inner_mm, np.zeros(segments)]),
        np.column_stack([circle * r_inner_mm, np.full(segments, height_mm)]),
    ]
    verts = np.vstack(rings)
    ob, ot, ib, it = (s * segments for s in range(4))
    faces = []
    for i in range(segments):
        j = (i + 1) % segments
        faces += [[ob + i, ob + j, ot + j], [ob + i, ot + j, ot + i]]  # outer wall
        faces += [[ib + i, it + j, ib + j], [ib + i, it + i, it + j]]  # inner wall
        faces += [[ot + i, ot + j, it + j], [ot + i, it + j, it + i]]  # top ring
        faces += [[ob + i, ib + j, ob + j], [ob + i, ib + i, ib + j]]  # bottom ring
    mesh = trimesh.Trimesh(vertices=verts, faces=np.asarray(faces))
    mesh.fix_normals()
    return mesh


def _regular_polygon(n_sides, across_flats_mm):
    """Vertices of a regular n-gon given its across-flats size (the wrench size)."""
    apothem = across_flats_mm / 2.0
    r = apothem / np.cos(np.pi / n_sides)
    ang = np.linspace(0.0, 2.0 * np.pi, n_sides, endpoint=False) + np.pi / n_sides
    return np.column_stack([np.cos(ang), np.sin(ang)]) * r


def _squat_can():
    return trimesh.creation.cylinder(radius=55.0, height=95.0, sections=_SEGMENTS)


def _hex_bar():
    bar = _convex_prism(_regular_polygon(6, 46.0), 240.0)
    # stand the bar on its side: length along X, hexagonal section in the YZ plane
    bar.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    return bar


def _u_bracket():
    """Concave C-profile: two arms joined by a back plate, opening toward +Y."""
    parts = [
        trimesh.creation.box(extents=(300.0, 40.0, 60.0)),   # back plate
        trimesh.creation.box(extents=(40.0, 140.0, 60.0)),   # left arm
        trimesh.creation.box(extents=(40.0, 140.0, 60.0)),   # right arm
    ]
    for part, centre in zip(parts, [(150, 20, 30), (20, 110, 30), (280, 110, 30)]):
        part.apply_translation(centre)
    return trimesh.util.concatenate(parts)


def _ring():
    return _annulus(130.0, 85.0, 28.0)


def _thin_sheet():
    return trimesh.creation.box(extents=(320.0, 210.0, 8.0))


def _slab_over_limit():
    return trimesh.creation.box(extents=(400.0, 332.0, 60.0))


def _ball():
    return trimesh.creation.icosphere(subdivisions=3, radius=95.0)


@dataclass(frozen=True)
class Probe:
    """One edge-case item: how to build it, and what the TASK RULES say it is.

    `expected` is derived from the drawing by hand (docs/probe-models.md), never
    read off our own output — that is the whole point of a procedural set.

    `poses` are named RESTING poses (axis, degrees), not random rotations. The
    census draws uniform SO(3) and lets Gazebo settle the body; an offline render
    cannot settle, so a drawn pose is often one no belt ever presents — a 8 mm
    sheet balanced on its edge measured 21 mm thick and "failed" the 10 mm rule
    against a pose that falls over on contact. Every pose listed here rests on a
    face (checked: centre of mass projects inside the contact polygon), so a
    disagreement is about the pipeline, not about an impossible pose.
    """

    build: Callable[[], trimesh.Trimesh]
    mass_kg: float
    expected: str          # B / C / D per the task rules
    dims_mm: tuple         # nominal, sorted descending
    k_analytic: float      # roundness of the governing section, by construction
    attacks: str           # which gate this probe exists to stress
    poses: tuple           # (name, (ax, ay, az), degrees) resting poses to test


_Z, _X = (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)

PROBES = {
    # Round section, but neither flat nor elongated: closes BOTH routes to D.
    # Both resting poses matter: upright hides the circle from the section route,
    # on its side hides it from the silhouette.
    "squat_can": Probe(_squat_can, 0.40, "D", (110, 110, 95), 1.000,
                       "round but neither flat nor elongated: no route to D",
                       (("upright", _Z, 0), ("on_side", _X, 90))),
    # A regular hexagon has K = cos(30 deg) = 0.866 > 0.8, so the jury's own
    # formula calls this bar round — the mirror image of Цилиндр (K=0.74 -> B).
    # rolled60 maps a hex face onto the next one: a different contact, same shape.
    "hex_bar": Probe(_hex_bar, 0.60, "D", (240, 53, 46), 0.866,
                     "polygon that is round by the K>0.8 formula but is not a circle",
                     (("lying", _Z, 0), ("yaw90", _Z, 90), ("rolled60", _X, 60))),
    # Concave outline: the touching-items splitter may cut one product into two.
    "u_bracket": Probe(_u_bracket, 0.90, "B", (300, 180, 60), 0.514,
                       "_split_touching on a concave single item",
                       (("opening_up", _Z, 0), ("opening_down", _X, 180),
                        ("yaw45", _Z, 45))),
    # Belt is visible through the middle: a hole inside the item mask.
    "ring": Probe(_ring, 0.30, "D", (260, 260, 28), 1.000,
                  "segmentation with a hole in the mask",
                  (("flat", _Z, 0), ("yaw45", _Z, 45))),
    # 8 mm tall against a 5 mm mask margin, and 8 < 10 mm minimum -> C.
    "thin_sheet": Probe(_thin_sheet, 0.25, "C", (320, 210, 8), 0.583,
                        "MASK_MARGIN_M: item barely above the belt",
                        (("flat", _Z, 0), ("yaw30", _Z, 30))),
    # 12 mm over the 320 mm limit: ~4 px at this camera, a real accuracy test.
    # Yaw is the point: the OBB fit, not the axis-aligned box, must hold 332 mm.
    "slab_over_limit": Probe(_slab_over_limit, 1.20, "C", (400, 332, 60), 0.529,
                             "measurement accuracy at the 320 mm bound",
                             (("flat", _Z, 0), ("yaw20", _Z, 20))),
    # Same CV trap as the can, plus the only probe aimed at the actuator: it rolls.
    # One pose is enough — a sphere presents the same silhouette from every angle.
    "ball": Probe(_ball, 0.25, "D", (190, 190, 190), 1.000,
                  "flatness/elongation gates + rolling on the belt",
                  (("any", _Z, 0),)),
}


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    for slug, probe in PROBES.items():
        stats = write_model(
            slug, probe.build(), probe.mass_kg,
            f"{slug} — procedural edge-case probe ({probe.attacks}), "
            "generated by scripts/build_probe_items.py",
            out_root=OUT_DIR,
        )
        ext = " × ".join(f"{x:.0f}" for x in stats["extents_mm"])
        print(f"{slug}: AABB {ext} mm, expect {probe.expected}, "
              f"mass {probe.mass_kg} kg, hull {stats['hull_faces']} faces")
    print(f"\n{len(PROBES)} probe models written to {OUT_DIR}")


if __name__ == "__main__":
    main()
