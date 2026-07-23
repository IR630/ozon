# -*- coding: utf-8 -*-
"""Safety gate: prove sensor <noise> actually reaches the DEPTH channel.

`make_noisy_world.py` bakes `<camera><noise>` into a world, but whether Ignition
applies it to an rgbd_camera's DEPTH image (not just RGB) is a build property, not
a property of the SDF. A world that says "noisy" and measures identically to the
clean one is the quiet lie this project exists to avoid. This compares depth dumps
from a clean run and a noisy run and reports the per-pixel noise floor of each.

The metric is COORDINATE-FREE — it needs no hand-picked belt patch. For each depth
frame it takes the residual from a 3x3 median (a local high-pass), over valid
(non-zero) pixels only. A clean depth image of belt+item is piecewise smooth, so
that residual is ~0 except on object edges; additive per-pixel Gaussian noise of
sigma raises it to ~sigma EVERYWHERE. So:

  noisy_floor ~= injected sigma, clean_floor ~= 0   -> noise reaches depth (PASS)
  noisy_floor ~= clean_floor                        -> it does NOT (FAIL, red flag)

    python3 scripts/verify_noise_reaches_depth.py runs/noise_verify/clean runs/noise_verify/noisy

Depth PNGs are 16-bit millimetres (dump_camera.py / perception_node dump format).
"""
from __future__ import annotations

import glob
import os
import sys

import cv2
import numpy as np


def depth_noise_floor_mm(depth_mm: np.ndarray) -> float | None:
    """Robust per-pixel high-frequency floor of one depth frame, in mm.

    Residual from a 3x3 median over valid pixels; reported as a robust sigma
    (MAD * 1.4826) so a few edge pixels do not dominate. None if too few valid
    pixels to judge.
    """
    valid = depth_mm > 0
    if valid.sum() < 100:
        return None
    smoothed = cv2.medianBlur(depth_mm, 3)
    resid = depth_mm.astype(np.int32) - smoothed.astype(np.int32)
    r = resid[valid].astype(np.float64)
    mad = np.median(np.abs(r - np.median(r)))
    return float(mad * 1.4826)


def floor_over_dir(logdir: str) -> list[float]:
    floors = []
    for path in sorted(glob.glob(os.path.join(logdir, "depth_*.png"))):
        img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if img is None or img.ndim != 2:
            continue
        f = depth_noise_floor_mm(img.astype(np.uint16))
        if f is not None:
            floors.append(f)
    return floors


def main(clean_dir: str, noisy_dir: str) -> int:
    clean = floor_over_dir(clean_dir)
    noisy = floor_over_dir(noisy_dir)
    if not clean or not noisy:
        print(f"ABORT: need depth_*.png in both dirs (clean={len(clean)}, "
              f"noisy={len(noisy)})", file=sys.stderr)
        return 2
    c, n = float(np.median(clean)), float(np.median(noisy))
    print(f"clean  noise floor: median {c:.2f} mm over {len(clean)} frames "
          f"(range {min(clean):.2f}-{max(clean):.2f})")
    print(f"noisy  noise floor: median {n:.2f} mm over {len(noisy)} frames "
          f"(range {min(noisy):.2f}-{max(noisy):.2f})")
    # A real depth-channel injection lifts the floor by many mm; require the noisy
    # floor to clear the clean one by an unambiguous margin (>= 3 mm and >= 3x).
    reaches = (n - c) >= 3.0 and n >= 3.0 * max(c, 0.5)
    print(f"VERDICT: noise {'REACHES' if reaches else 'DOES NOT REACH'} the depth "
          f"channel (noisy {n:.2f} vs clean {c:.2f} mm)")
    return 0 if reaches else 1


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1], sys.argv[2]))
