#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""YZ section of the three-head layout: do the side camera bodies fall in the top
head's cone?

The branch brief names this as one of three ways the layout fails SILENTLY — a
side camera body inside the top cone lands in the mask and _find_item returns
None on every frame — and requires looking at it, not only computing it. The
arithmetic says the bodies clear the cone by 234 mm; this draws the same geometry
so the margin can be seen, including the part arithmetic alone hides: the
clearance is a function of MOUNTING HEIGHT, not of |y|. At belt level the cone
already reaches 869 mm and a head at |y| = 900 mm would sit 31 mm from the edge
of frame.

    python3 scripts/plot_three_camera_layout.py [out.png]
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.perception import (  # noqa: E402
    BELT_TOP_Z_M,
    CAMERA_Z_M,
    FX,
    HFOV_RAD,
    IMG_H,
)

SIDE_Y_M = 0.90            # plan-branch-3cameras.md: the two facing heads
SIDE_Z_M = BELT_TOP_Z_M + 0.35
BELT_EDGE_Y_M = 0.25       # constants.py: diverter park clearance is quoted to this
# A compact RGBD housing (D435-class): 90 mm across, 25 mm deep, on a 60 mm bracket.
BODY_W_M, BODY_H_M = 0.090, 0.085

W, H = 1000, 620
Y_SPAN_M = 1.30


def _to_px(y_m, z_m):
    """World (y, z) metres -> image pixels; +y is drawn to the right, +z upward."""
    x = int(W / 2 + y_m / Y_SPAN_M * (W / 2 - 40))
    y = int(H - 60 - (z_m - 0.0) / (CAMERA_Z_M + 0.15) * (H - 110))
    return x, y


def main(argv=None):
    import cv2

    args = list(argv if argv is not None else sys.argv[1:])
    out = Path(args[0]) if args else (
        Path(__file__).resolve().parents[1] / "docs" / "report" / "img"
        / "three_camera_layout_yz.png")

    img = np.full((H, W, 3), 250, np.uint8)
    half_h = np.tan(HFOV_RAD / 2.0)          # horizontal half-angle (image x = world y)

    cam = _to_px(0.0, CAMERA_Z_M)
    # cone edges, drawn down to the belt
    depth_belt = CAMERA_Z_M - BELT_TOP_Z_M
    for sign in (-1, 1):
        edge = _to_px(sign * depth_belt * half_h, BELT_TOP_Z_M)
        cv2.line(img, cam, edge, (200, 170, 120), 2)
    # cone width at the side heads' own height, the number that actually decides
    depth_side = CAMERA_Z_M - SIDE_Z_M
    reach = depth_side * half_h
    a, b = _to_px(-reach, SIDE_Z_M), _to_px(reach, SIDE_Z_M)
    cv2.line(img, a, b, (200, 170, 120), 1, cv2.LINE_AA)

    # belt
    left, right = (_to_px(-BELT_EDGE_Y_M, BELT_TOP_Z_M),
                   _to_px(BELT_EDGE_Y_M, BELT_TOP_Z_M))
    cv2.line(img, left, right, (90, 90, 90), 6)
    cv2.putText(img, "belt", (left[0] - 5, left[1] + 26), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (90, 90, 90), 1, cv2.LINE_AA)

    # top head
    cv2.circle(img, cam, 9, (40, 40, 200), -1)
    cv2.putText(img, f"top  z={CAMERA_Z_M:.2f}", (cam[0] - 60, cam[1] - 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 40, 200), 1, cv2.LINE_AA)

    # side heads, drawn at their real body size
    clearance_mm = (SIDE_Y_M - BODY_W_M / 2 - reach) * 1000.0
    for sign in (-1, 1):
        cy = sign * SIDE_Y_M
        p0 = _to_px(cy - BODY_W_M / 2, SIDE_Z_M + BODY_H_M / 2)
        p1 = _to_px(cy + BODY_W_M / 2, SIDE_Z_M - BODY_H_M / 2)
        cv2.rectangle(img, p0, p1, (30, 140, 30), -1)
        cv2.putText(img, f"side y={cy:+.2f}", (p0[0] - 30, p0[1] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 110, 30), 1, cv2.LINE_AA)

    txt = [
        f"HFOV {HFOV_RAD:.3f} rad, fx {FX:.1f} px, {IMG_H}px tall",
        f"cone at side-head height z={SIDE_Z_M:.2f}: |y| < {reach * 1000:.0f} mm",
        f"cone at belt      z={BELT_TOP_Z_M:.2f}: |y| < {depth_belt * half_h * 1000:.0f} mm",
        f"nearest body edge at |y| = {(SIDE_Y_M - BODY_W_M / 2) * 1000:.0f} mm"
        f"  ->  clearance {clearance_mm:.0f} mm",
        "clearance is set by MOUNTING HEIGHT: at belt level the same head would"
        f" clear by only {(SIDE_Y_M - BODY_W_M / 2 - depth_belt * half_h) * 1000:.0f} mm",
    ]
    for i, line in enumerate(txt):
        cv2.putText(img, line, (18, 26 + i * 21), cv2.FONT_HERSHEY_SIMPLEX,
                    0.52, (30, 30, 30), 1, cv2.LINE_AA)

    out.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv2.imencode(".png", img)       # Unicode-safe write, as elsewhere in repo
    if not ok:
        sys.exit("не удалось закодировать PNG")
    out.write_bytes(buf.tobytes())
    print(f"{out}  |  clearance {clearance_mm:.0f} mm at z={SIDE_Z_M:.2f}, "
          f"{(SIDE_Y_M - BODY_W_M / 2 - depth_belt * half_h) * 1000:.0f} mm at belt level")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
