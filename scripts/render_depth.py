#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render a depth frame of an item on the belt, straight from its mesh.

WHY THIS EXISTS. The census only ever tests the 33 poses seed 0 happens to draw, and the
milestone's residual risks are all "this pose did not come up": Тарелка on edge, Шлем's
middle dimension 3 mm from the 320 limit, Мешок's K next to the 0.8 threshold. Those are
claims about poses we have never measured, and a Gazebo cell costs ~30 s — a few hundred
poses is a day of simulator time. Rendered off the mesh, a pose costs milliseconds, so the
question "where does the RULE actually break, and by how much" becomes answerable.

It feeds the REAL perception (src.perception.measure_item), not a re-implementation of it:
the whole point is to exercise the code the cell runs, so what breaks here breaks there.

HOW. The camera looks straight down (cell.sdf: 1.5, 0, 1.9) with the pinhole model
src/perception.py already pins. We sample the mesh surface densely, project every point
through that same model, and keep the NEAREST surface per pixel — a z-buffer, so occlusion
is handled the way the camera handles it. The empty belt fills the rest of the frame.

HONESTY LIMIT (PLAN.md's own rule: synthetic data may not set thresholds until the domain
gap is measured). A rendered frame has no sensor noise, no belt texture and no shadow, and
its surfaces are exactly the STL's. ``pytest -q tests/test_render_depth.py`` measures the
gap against saved real Gazebo frames, and that number — not this renderer — is what says
how far the sweep can be trusted.
"""
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent))

from build_item_models import ITEMS, STL_DIR, set_belt_origin  # noqa: E402

from src.perception import (  # noqa: E402
    BELT_DEPTH_M,
    BELT_TOP_Z_M,
    CAMERA_X_M,
    CAMERA_Y_M,
    CAMERA_Z_M,
    FX,
    FY,
    IMG_H,
    IMG_W,
)

CX, CY = IMG_W / 2.0, IMG_H / 2.0
N_SURFACE_SAMPLES = 400_000  # dense enough that the projected item has no pinholes


def render_depth(mesh, quat, at_x_m=CAMERA_X_M, at_y_m=CAMERA_Y_M):
    """Depth frame (metres, IMG_H x IMG_W) of `mesh` lying on the belt in pose `quat`.

    The mesh is placed exactly as the simulator places it: on the model origin convention
    (build_item_models.set_belt_origin), rotated, then dropped so its LOWEST point rests on
    the belt — the same contract scripts/spawn_orientations.py computes the spawn height
    from. `at_x/y` put the item's body (not its origin) under the camera.
    """
    import trimesh

    mesh = mesh.copy()
    set_belt_origin(mesh)
    x, y, z, w = quat
    mesh.apply_transform(trimesh.transformations.quaternion_matrix([w, x, y, z]))

    # seed=0: surface sampling is the ONLY randomness here, and unseeded it made
    # sweep margins wobble +-3 mm / +-0.02 K between identical runs (Karpathy #5:
    # every experiment reproducible from its seed).
    pts = trimesh.sample.sample_surface(mesh, N_SURFACE_SAMPLES, seed=0)[0] / 1000.0  # mm -> m
    # rest the body on the belt and centre it under the camera
    pts[:, 0] += at_x_m - (pts[:, 0].min() + pts[:, 0].max()) / 2
    pts[:, 1] += at_y_m - (pts[:, 1].min() + pts[:, 1].max()) / 2
    pts[:, 2] += BELT_TOP_Z_M - pts[:, 2].min()

    depth = CAMERA_Z_M - pts[:, 2]
    v = np.rint(CY - (pts[:, 0] - CAMERA_X_M) * FY / depth).astype(int)
    u = np.rint(CX - (pts[:, 1] - CAMERA_Y_M) * FX / depth).astype(int)
    inside = (v >= 0) & (v < IMG_H) & (u >= 0) & (u < IMG_W)

    frame = np.full((IMG_H, IMG_W), BELT_DEPTH_M, dtype=float)
    # z-buffer: the camera sees the NEAREST surface, so keep the smallest depth per pixel
    np.minimum.at(frame, (v[inside], u[inside]), depth[inside])
    return frame


def load_mesh(slug):
    import trimesh

    stem, _ = ITEMS[slug]
    return trimesh.load(str(STL_DIR / f"{stem}.stl"), force="mesh")


def read_resting_quat(pose_txt):
    """(x, y, z, w) of the RESTING pose `ign model --pose` printed next to a frame.

    Parse from the "Pose [ XYZ ] [ RPY ]" header down, not from the first bracket in the
    file: the block opens with "Model: [69]", and reading that id as a pose silently feeds
    the item's POSITION in where its ROTATION belongs. It renders a plausible frame and a
    plausible gap — the first gap table built this way "measured" 31 mm and dK=0.78 and was
    pure fiction. The real gap is 3 mm.
    """
    import re

    body = Path(pose_txt).read_text(encoding="utf-8").split("Pose [")[1]
    rpy = [float(v) for v in re.findall(r"\[([-\d.\s]+)\]", body)[-1].split()]
    from body_pose import quat_from_rpy

    return quat_from_rpy(*rpy)


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: render_depth.py <slug> <out.png>   (identity pose, on the belt)")
    import cv2

    frame = render_depth(load_mesh(sys.argv[1]), (0.0, 0.0, 0.0, 1.0))
    output = Path(sys.argv[2])
    # same encoding as scripts/dump_camera.py writes: 16-bit millimetres
    ok, encoded = cv2.imencode(".png", (frame * 1000.0).astype(np.uint16))
    if not ok:
        raise RuntimeError(f"OpenCV could not encode depth PNG: {output}")
    output.write_bytes(encoded.tobytes())
    print(f"wrote {output}  (item depth {frame.min():.3f} m, belt {BELT_DEPTH_M:.3f} m)")


if __name__ == "__main__":
    main()
